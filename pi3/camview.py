"""Live view of the rover camera (USB webcam by default) in an OpenCV window.

Works on the laptop or on the Pi's desktop (VNC):
    python camview.py --source rtsp://192.168.1.x:554
    ./pi.sh view --source rtsp://...      (from the laptop, opens it on the Pi)

Keys, with the window focused:  q quit   s save snapshot   f flip 180

Measuring delay: run with --clock and point the camera at this window. The
picture then shows an older copy of the clock; the gap between the two
readings is the full camera-to-screen delay. Press s to keep a snapshot.
"""

import argparse
import os
import time

import cv2

from camera import ROTATIONS, open_camera


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="usb",
                   help="usb (webcam, found automatically), /dev/videoN, an RTSP URL, or csi")
    p.add_argument("--transport", default="tcp", choices=["tcp", "udp"])
    p.add_argument("--rotate", default="none", choices=ROTATIONS,
                   help="turn the picture upright for a camera mounted sideways (cw: head was on the left)")
    p.add_argument("--cap", default="640x480", help="capture size WxH for a webcam or Pi camera")
    p.add_argument("--flip", action="store_true", help="camera is mounted upside down, csi only")
    p.add_argument("--clock", action="store_true",
                   help="draw a large millisecond clock, to measure delay by filming the window")
    args = p.parse_args()

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise SystemExit("no desktop to draw on: run this on a desktop (or in VNC), or use ./pi.sh view")

    w, h = (int(v) for v in args.cap.lower().split("x"))
    cam = open_camera(args.source, w, h, args.flip, args.transport, args.rotate)
    print(f"camera started: {args.source}   keys: q quit, s snapshot, f flip", flush=True)

    def run():
        flip = False
        cam_fps = 0.0
        last_count, last_t = 0, time.time()
        last_warn = 0.0
        shown = 0  # camera frame count last drawn
        while True:
            # only redraw for a new frame; otherwise just keep the window responsive
            frame = cam.read() if cam.count != shown else None
            if frame is not None:
                shown = cam.count
            if frame is None:
                if cam.age() > 2.0 and time.time() - last_warn > 1.0:
                    print(f"WARNING no camera frame for {cam.age():.1f} s", flush=True)
                    last_warn = time.time()
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break
                continue
            if flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            now = time.time()
            if now - last_t >= 1.0:
                cam_fps = (cam.count - last_count) / (now - last_t)
                last_count, last_t = cam.count, now
                print(f"camera {cam_fps:4.1f} fps", flush=True)

            view = frame.copy()  # overlay on a copy so snapshots stay clean
            fh, fw = view.shape[:2]
            cv2.line(view, (fw // 2, 0), (fw // 2, fh), (80, 80, 80), 1)
            # bottom-left, so it doesn't cover the follower's status lines when watching its view
            cv2.putText(view, f"{fw}x{fh}  {cam_fps:4.1f} fps", (10, fh - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if args.clock:
                stamp = time.strftime("%S.", time.localtime(now)) + f"{int(now * 1000) % 1000:03d}"
                (tw, th), _ = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 2.5, 6)
                x, y = fw - tw - 20, fh - 20
                cv2.rectangle(view, (x - 10, y - th - 10), (x + tw + 10, y + 10), (0, 0, 0), -1)
                cv2.putText(view, stamp, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 6)
            cv2.imshow("camera", view)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("f"):
                flip = not flip
            if k == ord("s"):
                os.makedirs("snapshots", exist_ok=True)
                path = time.strftime("snapshots/snap-%Y%m%d-%H%M%S.jpg")
                cv2.imwrite(path, view if args.clock else frame)  # keep both clocks when measuring
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
