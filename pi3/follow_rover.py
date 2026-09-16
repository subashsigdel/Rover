"""Simple human follower: webcam -> YOLO person box -> distance -> ESP32 wheels.

    camera frame -> YOLO finds people -> take the biggest box (the nearest person)
    distance = k / box width            (pinhole camera: width x distance stays the same)
    farther than --stop-dist            -> drive toward them, faster the farther, steer to the box centre
    at --stop-dist (40 cm) or closer    -> STOP
    box fills the picture side to side  -> STOP (they are right in front of the rover)
    nobody seen                         -> STOP

Width, not height: the webcam sits low, so a standing person's box touches the top
and bottom of the picture from 1-2 m in and its height stops changing. The width
keeps growing until they are right at the rover.

Calibrate once (k): stand at a measured distance, facing the camera, and hold still.
    ./pi.sh calibrate 1m                        # with the follower running (boot service)
    python follow_rover.py --calibrate 1m       # or directly on the Pi

Run:
    python follow_rover.py --port auto          # drive (finds the ESP32 serial port)
    python follow_rover.py                      # dry run: prints what it would do
    python follow_rover.py --port auto --stop-dist 40cm --max-pwm 140

Distances: 20cm, 1m, 30in, 2ft; a bare number is centimetres.
Live view with the box and distance: http://<pi>:8080/
"""

import argparse
import glob
import json
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
from ultralytics import YOLO

from camera import ROTATIONS, open_camera
from control import PERSON_CLASS, choose_model

HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(HERE, "calibration.json")
CALIBRATION_REQUEST = os.path.join(HERE, "calibrate.request")  # dropped by ./pi.sh calibrate
FILL = 0.90         # box this wide (fraction of the picture) = right in front: stop
CALIB_FRAMES = 10   # steady readings needed to calibrate
UNITS = {"cm": 0.01, "mm": 0.001, "in": 0.0254, "ft": 0.3048, "m": 1.0}


def parse_dist(text):
    """'20cm', '1m', '30in', '2ft' -> metres. A bare number is centimetres."""
    t = str(text).strip().lower()
    for unit in sorted(UNITS, key=len, reverse=True):  # "cm"/"mm" before "m"
        if t.endswith(unit):
            return float(t[:-len(unit)]) * UNITS[unit]
    return float(t) * 0.01


class RoverLink:
    """Sends "M<left>,<right>\\n" (-255..255) to the ESP32.

    The firmware stops the motors when commands stop for 500 ms, and one YOLO
    frame on the Pi 3 takes ~350 ms, so a thread resends the latest command at
    10 Hz - only while it is younger than hold_s, so a hung loop still stops.
    A port is used only if it answers like the rover firmware ("DRIVE TIMEOUT"):
    the AMB82 has the same USB-serial chip.
    """

    def __init__(self, port, hold_s=1.0):
        self.ser, self.port = self._find(port)
        self.hold_s = hold_s
        self.lock = threading.Lock()
        self.cmd, self.cmd_t = (0, 0), 0.0
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _find(port):
        import serial

        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")) if port == "auto" else [port]
        replies = {}
        for p in ports:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout, ser.write_timeout = p, 115200, 0.1, 0.1
            ser.dtr = ser.rts = False  # don't reset the ESP32
            try:
                ser.open()
            except (OSError, serial.SerialException) as e:
                replies[p] = str(e)
                continue
            time.sleep(2.0)  # in case opening reset the board
            ser.reset_input_buffer()
            ser.write(b"M0,0\n")  # moves nothing; the firmware prints DRIVE TIMEOUT 0.5 s later
            seen, deadline = b"", time.time() + 1.5
            while time.time() < deadline and b"DRIVE TIMEOUT" not in seen:
                seen += ser.read(256)
            if b"DRIVE TIMEOUT" in seen:
                return ser, p
            ser.close()
            replies[p] = seen.decode(errors="replace").strip()[-60:]
        raise RuntimeError(f"no rover ESP32 found; answers to M0,0: {replies or 'no serial ports'}")

    def drive(self, left, right):
        with self.lock:
            self.cmd, self.cmd_t = (int(left), int(right)), time.time()

    def _loop(self):
        while self.running:
            with self.lock:
                cmd, fresh = self.cmd, time.time() - self.cmd_t < self.hold_s
            if fresh:
                self._send(*cmd)
            time.sleep(0.1)

    def _send(self, left, right):
        self.ser.write(f"M{left},{right}\n".encode())
        self.ser.reset_input_buffer()  # throw away the ESP32's prints

    def close(self):
        self.running = False
        time.sleep(0.15)
        try:
            self._send(0, 0)
        finally:
            self.ser.close()


def wheels(throttle, steer, min_pwm, max_pwm):
    """throttle 0..1 and steer -1..1 (right +) -> left/right PWM."""
    left, right = throttle + steer, throttle - steer
    m = max(1.0, abs(left), abs(right))
    out = []
    for v in (left / m, right / m):
        out.append(0 if abs(v) < 0.02 else int(np.sign(v) * (min_pwm + (max_pwm - min_pwm) * abs(v))))
    return out


def load_k(cap):
    try:
        with open(CALIBRATION_FILE) as f:
            cal = json.load(f)
    except (OSError, ValueError):
        return None
    if "k_width" not in cal:
        return None  # from the old height-based follower
    if cal.get("cap") != cap:
        print(f"calibration.json was made at {cal.get('cap')}, now {cap}: calibrate again", flush=True)
        return None
    return cal["k_width"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="ESP32 serial port, or auto; omit for a dry run")
    p.add_argument("--stop-dist", type=parse_dist, default="40cm", help="stop at this distance or closer")
    p.add_argument("--full-speed-dist", type=parse_dist, default="150cm", help="full speed this far away")
    p.add_argument("--min-pwm", type=int, default=90, help="PWM where the wheels start turning")
    p.add_argument("--max-pwm", type=int, default=150, help="PWM at full speed (of 255)")
    p.add_argument("--steer-gain", type=float, default=0.8, help="0 = drive straight only")
    p.add_argument("--conf", type=float, default=0.5, help="YOLO person confidence")
    p.add_argument("--calibrate", type=parse_dist, default=None, metavar="DIST",
                   help="stand DIST from the camera, save the calibration and exit")
    p.add_argument("--source", default="usb", help="usb, /dev/videoN, an RTSP URL, or csi")
    p.add_argument("--cap", default="640x480", help="capture size WxH")
    p.add_argument("--rotate", default="none", choices=ROTATIONS)
    p.add_argument("--model", default="auto")
    p.add_argument("--view", type=int, default=8080, help="live view port, 0 = off")
    args = p.parse_args()

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(0))

    print("follow_rover:", " ".join(f"{n}={v}" for n, v in vars(args).items()), flush=True)
    k = load_k(args.cap)
    if k:
        print(f"calibrated: k = {k:.3f}, stop at {args.stop_dist * 100:.0f} cm", flush=True)
    else:
        print("NOT CALIBRATED - will not drive. Stand 1 m from the camera and run: ./pi.sh calibrate 1m",
              flush=True)

    cap_w, cap_h = (int(v) for v in args.cap.lower().split("x"))
    cam = open_camera(args.source, cap_w, cap_h, rotate=args.rotate)
    model_path, imgsz = choose_model(args.model, 320)
    model = YOLO(model_path, task="detect")
    print(f"camera {args.source}, model {os.path.basename(model_path)} input {imgsz}", flush=True)
    link = RoverLink(args.port) if args.port and args.calibrate is None else None
    print(f"rover link {link.port}" if link else "no rover link - dry run, motors not driven", flush=True)
    viewer = None
    if args.view:
        from webview import FrameServer
        viewer = FrameServer(args.view)
        print(f"live view: http://<pi>:{args.view}/", flush=True)

    def run():
        nonlocal k
        calib_dist = args.calibrate
        calib_widths, calib_start = [], time.time()
        following = False
        fps = infer_ms = 0.0
        last_print = 0.0
        done = -1

        while True:
            if cam.count == done:
                time.sleep(0.005)
                continue
            done = cam.count
            frame = cam.read()
            if frame is None:  # camera stalled
                if link:
                    link.drive(0, 0)
                time.sleep(0.05)
                continue
            now = time.time()
            h, w = frame.shape[:2]

            if calib_dist is None and os.path.exists(CALIBRATION_REQUEST):
                try:
                    with open(CALIBRATION_REQUEST) as f:
                        calib_dist = parse_dist(json.load(f).get("dist") or "1m")
                except ValueError as e:
                    print(f"calibration request ignored: {e}", flush=True)
                os.remove(CALIBRATION_REQUEST)
                if calib_dist:
                    calib_widths, calib_start = [], now
                    print(f"calibrating: stand {calib_dist * 100:.0f} cm from the camera, face it, hold still "
                          "- not driving", flush=True)

            # 1. detect people, take the biggest box = the nearest person
            t0 = time.time()
            res = model.predict(frame, classes=[PERSON_CLASS], conf=args.conf, imgsz=imgsz,
                                device="cpu", verbose=False)[0]
            infer_ms = 0.8 * infer_ms + 0.2 * (time.time() - t0) * 1000
            box = None
            if len(res.boxes):
                xyxy = res.boxes.xyxy.cpu().numpy()
                areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
                i = int(np.argmax(areas))
                box, conf = xyxy[i], float(res.boxes.conf[i])

            # 2. distance from the box width
            width = dist = None
            steer = throttle = 0.0
            if box is not None:
                x1, y1, x2, y2 = box
                width = (x2 - x1) / w
                if k:
                    dist = k / width

            # 3. calibrate, or decide go / stop
            if calib_dist is not None:
                following = False
                if width is not None:
                    calib_widths = (calib_widths + [width])[-CALIB_FRAMES:]
                if len(calib_widths) == CALIB_FRAMES and max(calib_widths) - min(calib_widths) < 0.05:
                    bw = float(np.median(calib_widths))
                    k = bw * calib_dist
                    with open(CALIBRATION_FILE, "w") as f:
                        json.dump({"k_width": round(k, 4), "box_width": round(bw, 3), "dist_m": calib_dist,
                                   "cap": args.cap, "saved": time.strftime("%Y-%m-%d %H:%M")}, f, indent=1)
                    print(f"calibrated: box {bw * 100:.0f}% wide at {calib_dist * 100:.0f} cm, k = {k:.3f} "
                          "- saved", flush=True)
                    if args.calibrate is not None:
                        return
                    calib_dist = None
                    print("calibration done - following again", flush=True)
                elif now - calib_start > 90:
                    print("calibration cancelled: no steady person box in 90 s", flush=True)
                    if args.calibrate is not None:
                        return
                    calib_dist = None
                state = f"CALIBRATING {len(calib_widths)}/{CALIB_FRAMES}"
            elif box is None:
                following = False
                state = "no person - STOP"
            elif not k:
                following = False
                state = "NOT CALIBRATED"
            elif width >= FILL or dist <= args.stop_dist:
                following = False
                state = "close - STOP"
            else:
                # 10 cm of slack before starting again, so it doesn't stutter at the stop distance
                following = following or dist > args.stop_dist + 0.10
                state = "FOLLOW" if following else "near - STOP"
                if following:
                    throttle = float(np.clip((dist - args.stop_dist) / (args.full_speed_dist - args.stop_dist),
                                             0.0, 1.0))
                    x_err = ((x1 + x2) / 2 - w / 2) / (w / 2)  # -1 left .. +1 right
                    if abs(x_err) > 0.1:
                        steer = float(np.clip(args.steer_gain * x_err, -1.0, 1.0))

            left, right = wheels(throttle, steer, args.min_pwm, args.max_pwm)
            if link:
                link.drive(left, right)
            fps = 0.9 * fps + 0.1 / max(time.time() - now, 1e-3)

            dist_txt = f"{dist * 100:.0f} cm" if dist is not None else "-"
            width_txt = f"{width * 100:.0f}%" if width is not None else "-"
            if now - last_print >= 1.0:
                last_print = now
                print(f"{fps:4.1f} fps  infer {infer_ms:4.0f} ms  person {'yes' if box is not None else 'no '}  "
                      f"width {width_txt:>4}  dist {dist_txt:>7}  {state}  L{left:+4d} R{right:+4d}"
                      f"{'' if link else '  dry run'}", flush=True)

            if viewer:
                colour = (0, 220, 0) if following else (0, 0, 255)
                if box is not None:
                    bx1, by1, bx2, by2 = [int(v) for v in box]
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), colour, 2)
                    cv2.putText(frame, f"{dist_txt}  w {width_txt}  {conf:.2f}", (bx1 + 4, min(h - 10, by2 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
                cv2.line(frame, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
                for j, line in enumerate((f"{fps:.1f} fps  {state}", f"L {left:+d}  R {right:+d}")):
                    cv2.putText(frame, line, (10, 24 + 24 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                viewer.publish(frame)

    # Loop in its own function: on Python 3.11 a Ctrl+C/SIGTERM in a tight loop can
    # skip a try/finally in the same function, and with it the motor stop.
    try:
        run()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if link:
            link.close()
        cam.release()


if __name__ == "__main__":
    main()
