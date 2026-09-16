"""Camera sources for the rover, shared by follow_rover.py and camview.py.

    open_camera("usb")                     a USB webcam on the Pi (default); or "/dev/video0"
    open_camera("rtsp://10.42.0.50:554")   a network camera, e.g. the AMB82 Mini over Wi-Fi
    open_camera("csi")                     a Pi ribbon camera, via Picamera2
    open_camera("usb", rotate="cw")        a webcam mounted sideways: frames come out upright

Every source keeps only the newest frame in a background thread and never
hands back a stale one.
"""

import glob
import os
import re
import threading
import time

AMB82_URL = "rtsp://10.42.0.50:554"


ROTATIONS = ("none", "cw", "ccw", "180")


def open_camera(source, width=640, height=480, flip=False, transport="tcp", rotate="none"):
    """rotate turns every frame read: cw/ccw by 90 degrees, for a camera mounted sideways.

    Sideways, a webcam's wide horizontal view becomes the vertical one, so a
    person close to the rover still fits top to bottom. width/height are the
    capture size, before rotating.
    """
    if source == "csi":
        cam = PiCamGrabber(width, height, flip)
    elif source == "usb" or source.startswith("/dev/video") or source.isdigit():
        cam = WebcamGrabber(source, width, height)
    else:
        cam = StreamGrabber(source, transport)
    cam.set_rotation(rotate)
    return cam


class _LatestFrame:
    """The newest frame plus its arrival time, shared between threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.frame_t = 0.0
        self.count = 0  # frames received so far, for measuring camera fps
        self.running = True
        self.rotate = None  # function turning a frame upright, or None to hand frames out as captured

    def set_rotation(self, rotate):
        import cv2

        codes = {"none": None, "cw": cv2.ROTATE_90_CLOCKWISE,
                 "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}
        if rotate not in codes:
            raise ValueError(f"rotate must be one of {', '.join(ROTATIONS)}, not {rotate!r}")
        code = codes[rotate]
        self.rotate = None if code is None else (lambda frame: cv2.rotate(frame, code))

    def _put(self, frame):
        with self.lock:
            self.frame = frame
            self.frame_t = time.time()
            self.count += 1

    def read(self, max_age=0.5):
        """Newest frame, or None if there is none from the last max_age seconds.

        Never hands back a stale frame: acting on an old image is worse than not
        acting, since a stalled camera would otherwise freeze the scene the
        controller sees while the rover keeps moving.
        """
        with self.lock:
            if self.frame is None or time.time() - self.frame_t > max_age:
                return None
            if self.rotate is None:
                return self.frame.copy()
            return self.rotate(self.frame)  # a new array, so it doubles as the copy

    def age(self):
        """Seconds since the last frame arrived."""
        with self.lock:
            return time.time() - self.frame_t

    def _wait_first(self, timeout, message):
        deadline = time.time() + timeout
        while self.count == 0:
            if time.time() > deadline:
                self.release()
                raise RuntimeError(message)
            time.sleep(0.05)


class WebcamGrabber(_LatestFrame):
    """USB webcam through V4L2: compressed MJPEG, newest frame only, reopens if unplugged."""

    FIRST_FRAME_TIMEOUT = 15.0
    FPS = 15  # detection runs at ~4 fps; decoding more frames only takes CPU from YOLO
    SKIP_NAMES = ("bcm2835", "unicam", "rpivid", "pispbe", "codec", "isp")  # Pi internal nodes

    def __init__(self, device="usb", width=640, height=480):
        super().__init__()
        import cv2

        self.cv2 = cv2
        self.device, self.size = device, (width, height)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self._wait_first(
            self.FIRST_FRAME_TIMEOUT,
            f"no frames from a USB webcam ({device}) in {self.FIRST_FRAME_TIMEOUT:.0f} s. "
            "Is it plugged into the Pi? `ls /dev/video*` should list it.")

    def _candidates(self):
        if self.device != "usb":
            return [self.device]
        found = []
        for path in sorted(glob.glob("/dev/video*"), key=lambda p: int(re.sub(r"\D", "", p) or 0)):
            try:
                with open(f"/sys/class/video4linux/{os.path.basename(path)}/name") as f:
                    name = f.read().strip().lower()
            except OSError:
                name = ""
            if not any(skip in name for skip in self.SKIP_NAMES):
                found.append(path)
        return found

    def _open(self):
        cv2 = self.cv2
        for dev in self._candidates():
            cap = cv2.VideoCapture(int(dev) if dev.isdigit() else dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # compressed: light on USB 2.0
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_FPS, self.FPS)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # the newest frame, not a queue of old ones
            ok, frame = cap.read()
            if ok:
                return cap, dev, frame
            cap.release()  # e.g. the metadata node every webcam also creates
        return None, None, None

    def _loop(self):
        failures = 0
        while self.running:
            cap, dev, frame = self._open()
            if cap is None:
                failures += 1
                if failures == 1 or failures % 10 == 0:
                    print(f"webcam: no working camera for {self.device!r} (attempt {failures}), retrying",
                          flush=True)
                time.sleep(1.0)
                continue
            h, w = frame.shape[:2]
            print(f"webcam: {dev} {w}x{h} @ {cap.get(self.cv2.CAP_PROP_FPS):.0f} fps", flush=True)
            failures = 0
            self._put(frame)
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    print("webcam: read failed (unplugged?), reopening", flush=True)
                    break
                self._put(frame)
            cap.release()

    def release(self):
        self.running = False
        self.thread.join(3.0)  # let the thread release the device before exit


class StreamGrabber(_LatestFrame):
    """RTSP (or any FFmpeg URL) reader that reconnects when the stream drops."""

    FIRST_FRAME_TIMEOUT = 40.0  # the AMB82 can take several tries to accept a new session
    MAX_LATE = 0.5  # seconds behind the stream's normal delay before a frame is dropped

    def __init__(self, url, transport="tcp"):
        super().__init__()
        import cv2

        self.cv2 = cv2
        self.url = url
        # Low-latency FFmpeg options: no input buffering, decode frames as they arrive.
        # Read when the capture opens, so set before the first VideoCapture.
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{transport}|fflags;nobuffer|flags;low_delay|max_delay;0")
        # Joining mid-stream, the decoder reports every frame before the first keyframe
        # ("non-existing PPS", "no frame!"). Harmless; our own messages cover real failures.
        os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "8")  # AV_LOG_FATAL
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self._wait_first(
            self.FIRST_FRAME_TIMEOUT,
            f"no video from {url} after {self.FIRST_FRAME_TIMEOUT:.0f} s. Is the AMB82 powered "
            "and on the same network? Its Serial Monitor prints the stream URL.")

    def _loop(self):
        cv2 = self.cv2
        failures = 0
        while self.running:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000,  # connect + wait for a keyframe; slow on a Pi 3
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000,  # a stalled stream errors out instead of blocking
                # FFmpeg's frame-threaded decoding holds back one frame per extra thread
                # (11 frames on a 12-core laptop); one thread easily decodes 640x360.
                cv2.CAP_PROP_N_THREADS, 1,
            ])
            if not cap.isOpened():
                failures += 1
                if failures == 1 or failures % 10 == 0:
                    print(f"stream: cannot open {self.url} (attempt {failures}), retrying", flush=True)
                time.sleep(1.0)
                continue
            w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"stream: connected to {self.url} ({w}x{h})", flush=True)
            failures = 0
            base_lag = None  # smallest (arrival time - stream time) seen: the link's normal delay
            last_t = time.time()
            last_late_msg = 0.0
            while self.running:
                ok, frame = cap.read()
                now = time.time()
                if not ok:
                    print("stream: lost, reconnecting", flush=True)
                    break
                # After a stall FFmpeg delivers the frames it had buffered, seconds late
                # but looking new. Compare each frame's stream timestamp with the Pi's
                # clock and drop the ones that arrive well behind the normal delay.
                pts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                if pts > 0 and self.url.startswith("rtsp"):  # HTTP MJPEG has no real timestamps
                    lag = now - pts
                    # creep the baseline up slowly so camera/Pi clock drift isn't read as lateness
                    base_lag = lag if base_lag is None else min(base_lag + 0.01 * (now - last_t), lag)
                    last_t = now
                    if lag - base_lag > self.MAX_LATE:
                        if now - last_late_msg > 1.0:
                            print(f"stream: dropping late frames ({lag - base_lag:.1f} s behind)", flush=True)
                            last_late_msg = now
                        continue
                self._put(frame)
            cap.release()

    def release(self):
        self.running = False
        # Let the thread leave FFmpeg's read and release the capture: killing it
        # inside native code at interpreter exit aborts the process.
        self.thread.join(3.0)


class PiCamGrabber(_LatestFrame):
    """Pi ribbon camera via Picamera2."""

    FIRST_FRAME_TIMEOUT = 5.0

    def __init__(self, width, height, flip):
        super().__init__()
        from libcamera import Transform
        from picamera2 import Picamera2

        if not Picamera2.global_camera_info():
            raise RuntimeError(
                "no camera detected (rpicam-hello --list-cameras is empty). The Pi only "
                "probes cameras at boot: with the power off, reseat the ribbon at both "
                "ends, then boot.")
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},  # BGR byte order, as OpenCV wants
            transform=Transform(hflip=flip, vflip=flip),
            controls={"FrameRate": 30},
            buffer_count=2,
            queue=False,  # never hand back a frame that was already waiting
        )
        self.cam.configure(cfg)
        self.cam.start()
        threading.Thread(target=self._loop, daemon=True).start()
        self._wait_first(
            self.FIRST_FRAME_TIMEOUT,
            f"camera detected but no frames in {self.FIRST_FRAME_TIMEOUT:.0f} s: the sensor "
            "answers on its control lines but sends no image data. Check the ribbon at both "
            "ends and the small sensor connector on the camera board, or try another "
            "cable/camera.")

    def _loop(self):
        while self.running:
            try:
                # a timeout keeps this thread from blocking forever if the camera stalls
                frame = self.cam.capture_array("main", wait=1.0)
            except TimeoutError:
                continue
            self._put(frame)

    def release(self):
        self.running = False
        time.sleep(0.1)
        # Picamera2.stop() waits on its event loop, which never answers once the
        # camera frontend has timed out; don't let a dead camera hang shutdown.
        stopper = threading.Thread(target=self.cam.stop, daemon=True)
        stopper.start()
        stopper.join(2.0)
