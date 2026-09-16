import argparse
import os
import time

import cv2
import numpy as np
from ultralytics import YOLO

from camera import ROTATIONS, open_camera
from control import PERSON_CLASS, TRACK_CONF, TRACKER_CFG, TargetLock, choose_model


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="usb",
                   help="usb (webcam, found automatically), /dev/videoN, an RTSP URL, or csi")
    p.add_argument("--transport", default="tcp", choices=["tcp", "udp"])
    p.add_argument("--rotate", default="none", choices=ROTATIONS,
                   help="turn the picture upright for a camera mounted sideways (cw: head was on the left)")
    p.add_argument("--model", default="auto",
                   help="auto: the ONNX export matching the picture shape (3x faster on the Pi 3), "
                        "else yolo11n.pt; the ncnn package crashes on the Pi 3")
    p.add_argument("--imgsz", type=int, default=320, help="network input size for .pt models")
    p.add_argument("--acquire-conf", type=float, default=0.6,
                   help="confidence needed to start following someone")
    p.add_argument("--lost-timeout", type=float, default=10.0,
                   help="seconds to wait for a lost target before picking someone new (0 = forever)")
    p.add_argument("--reid-dist", type=float, default=0.3,
                   help="max clothing-colour distance (0-1) to recognise the target under a new ID")
    args = p.parse_args()

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise SystemExit("no desktop to draw on: run this on a desktop (or in VNC)")

    cam = open_camera(args.source, transport=args.transport, rotate=args.rotate)
    print(f"camera started: {args.source}", flush=True)
    model_path, imgsz = choose_model(args.model, args.imgsz)
    model = YOLO(model_path, task="detect")
    print(f"model loaded {os.path.basename(model_path)}, input {imgsz}   keys: q quit, s snapshot, r release lock",
          flush=True)
    lock = TargetLock(args.acquire_conf, args.lost_timeout, args.reid_dist)

    def run():
        det_fps = infer_ms = cam_fps = 0.0
        detections = last_detections = 0  # frames detected on, for the real detection rate
        last_count, last_fps_t = cam.count, time.time()
        last_print = 0.0
        done = 0  # camera frame count last detected on
        while True:
            # only new frames: re-detecting the same image wastes CPU and misleads the tracker
            frame = cam.read() if cam.count != done else None
            if frame is None:
                cv2.waitKey(5)
                continue
            done = cam.count
            detections += 1

            t0 = time.time()
            res = model.track(frame, persist=True, classes=[PERSON_CLASS], imgsz=imgsz,
                              conf=TRACK_CONF, tracker=TRACKER_CFG, verbose=False)[0]
            infer_ms = 0.8 * infer_ms + 0.2 * (time.time() - t0) * 1000

            boxes = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else np.empty((0, 4))
            confs = res.boxes.conf.cpu().tolist() if res.boxes is not None else []
            ids = (res.boxes.id.int().cpu().tolist()
                   if res.boxes is not None and res.boxes.id is not None else [])
            if len(ids) != len(boxes):
                boxes, ids, confs = np.empty((0, 4)), [], []  # tracker not warmed up yet

            now = time.time()
            target = lock.pick(frame, boxes, ids, confs, now)

            view = frame
            for box, c, tid in zip(boxes, confs, ids):
                x1, y1, x2, y2 = [int(v) for v in box]
                is_target = target is not None and tid == lock.tid
                colour = (0, 220, 0) if is_target else (255, 160, 0)
                cv2.rectangle(view, (x1, y1), (x2, y2), colour, 3 if is_target else 1)
                label = f"TARGET id {tid}  {c:.2f}" if is_target else f"id {tid}  {c:.2f}"
                cv2.putText(view, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

            if now - last_fps_t >= 1.0:
                cam_fps = (cam.count - last_count) / (now - last_fps_t)
                det_fps = (detections - last_detections) / (now - last_fps_t)
                last_count, last_detections, last_fps_t = cam.count, detections, now

            fh, fw = view.shape[:2]
            lines = [f"detect {det_fps:4.1f} fps   infer {infer_ms:5.1f} ms",
                     f"camera {cam_fps:4.1f} fps   {fw}x{fh} -> input {imgsz}",
                     f"persons {len(boxes)}   {lock.status(now)}"]
            cv2.rectangle(view, (0, 0), (400, 72), (0, 0, 0), -1)
            for i, line in enumerate(lines):
                cv2.putText(view, line, (8, 20 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            cv2.imshow("person detection", view)

            if now - last_print >= 1.0:
                last_print = now
                print(f"detect {det_fps:4.1f} fps  infer {infer_ms:5.1f} ms  camera {cam_fps:4.1f} fps  "
                      f"persons {len(boxes)}  {lock.status(now)}  re-found {lock.reid_count}x", flush=True)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("r"):
                lock.release()
                print("lock released", flush=True)
            if k == ord("s"):
                os.makedirs("snapshots", exist_ok=True)
                path = time.strftime("snapshots/detect-%Y%m%d-%H%M%S.jpg")
                cv2.imwrite(path, view)
                print(f"saved {path}", flush=True)
    # The loop runs in its own function: on Python 3.11 (the Pi's too) a Ctrl+C or
    # SIGTERM landing on a tight loop's jump back can skip a try/finally in the
    # same function, and with it the cleanup.
    try:
        run()
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
