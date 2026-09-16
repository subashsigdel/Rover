"""Control pieces for the human follower: PD loops, target lock, and the HUD."""

import importlib.util
import os
import re

import cv2
import numpy as np

PERSON_CLASS = 0
TRACKER_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rover_track.yaml")
TRACK_CONF = 0.1  # detector threshold for tracking; ByteTrack needs the weak detections too


def choose_model(model, imgsz):
    """Pick the detector and its input size.

    "auto" uses ONNX Runtime when it is installed: on the Pi 3 it ran yolo11n in
    177 ms against PyTorch's 567 ms, with the same detections. The export has a
    fixed, wide input matching the webcam. Without onnxruntime, fall back to
    yolo11n.pt.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    if model == "auto":
        if importlib.util.find_spec("onnxruntime") is None:
            model = "yolo11n.pt"
        else:
            model = "yolo11n_192x320.onnx"
    fixed = re.search(r"_(\d+)x(\d+)\.onnx$", model)
    if fixed:
        imgsz = [int(fixed.group(1)), int(fixed.group(2))]  # the export's input: predict must match
    if not os.path.isabs(model) and not os.path.exists(model):
        model = os.path.join(here, model)
    return model, imgsz


class PD:
    def __init__(self, kp, kd, deadband=0.0):
        self.kp, self.kd, self.deadband = kp, kd, deadband
        self.prev_err = 0.0
        self.prev_t = None

    def step(self, err, now):
        if abs(err) < self.deadband:
            err = 0.0
        d = 0.0
        if self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 1e-4:
                d = (err - self.prev_err) / dt
        self.prev_err, self.prev_t = err, now
        return self.kp * err + self.kd * d

    def reset(self):
        self.prev_err = 0.0
        self.prev_t = None


def appearance(frame, box):
    """Clothing colour signature: hue/saturation histograms of the upper and lower body.

    Made to survive lighting changes: brightness is left out, and the frame's
    overall colour cast is removed first (gray world). Tested on 5 people under
    darker, brighter, warm and cool light: the same person stayed within 0.26,
    different people were 0.30 apart or more - the close pair wore similar suits,
    which colour alone cannot tell apart.
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    fh, fw = frame.shape[:2]
    means = np.array(cv2.mean(frame)[:3])
    gains = (means.mean() / np.maximum(means, 1.0)).astype(np.float32)
    hists = []
    # the middle half of the width avoids background at the sides
    for top, bottom in ((0.15, 0.50), (0.55, 0.90)):
        cx1, cx2 = int(max(0, x1 + 0.25 * w)), int(min(fw, x2 - 0.25 * w))
        cy1, cy2 = int(max(0, y1 + top * h)), int(min(fh, y1 + bottom * h))
        if cx2 - cx1 < 4 or cy2 - cy1 < 4:
            return None
        crop = np.clip(frame[cy1:cy2, cx1:cx2].astype(np.float32) * gains, 0, 255).astype(np.uint8)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 4], [0, 180, 0, 256])
        hists.append(cv2.normalize(hist, None, norm_type=cv2.NORM_L1).flatten())
    return np.concatenate(hists).astype(np.float32)


def appearance_distance(a, b):
    """0 = identical colours, 1 = nothing in common (Bhattacharyya)."""
    if a is None or b is None:
        return 1.0
    return float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA))


class TargetLock:
    """Locks onto one person and stays with them until they are really gone.

    Acquire: the largest (nearest) person detected at acquire_conf or above.
    Follow:  that ByteTrack ID at whatever score, so partial occlusion keeps the lock.
    Lost:    nobody else is taken over. If the person comes back under a new
             tracker ID, their clothing colours pick them out again. People who
             were seen next to the target are known to be someone else and are
             never taken as the target coming back.
    Give up: only after lost_timeout seconds without them (0 = wait forever).
    """

    def __init__(self, acquire_conf=0.6, lost_timeout=10.0, reid_max_dist=0.3):
        self.acquire_conf = acquire_conf
        self.lost_timeout = lost_timeout
        self.reid_max_dist = reid_max_dist
        self.release()

    def pick(self, frame, boxes, ids, confs, now):
        """Return the locked target's box, or None while nobody is locked or they are lost."""
        if self.tid is not None:
            present = False
            for box, i, c in zip(boxes, ids, confs):
                if i == self.tid:
                    present, target = True, (box, c)
            if present:
                self.last_reid_dist = None
                self.others.update(i for i in ids if i != self.tid)
                self._seen(frame, *target, now)
                return target[0]

            # the target is missing: only a newcomer who looks like them counts
            best, best_d = None, self.reid_max_dist
            self.last_reid_dist = None  # closest match this frame, shown in the log for tuning
            for box, i, c in zip(boxes, ids, confs):
                if i in self.others or c < self.acquire_conf:
                    continue
                d = appearance_distance(self.signature, appearance(frame, box))
                if self.last_reid_dist is None or d < self.last_reid_dist:
                    self.last_reid_dist = d
                if d < best_d:
                    best, best_d = (box, i, c), d
            if best is not None:
                self.tid, self.reid_count = best[1], self.reid_count + 1
                self._seen(frame, best[0], best[2], now)
                return best[0]

            if self.lost_timeout <= 0 or now - self.last_seen < self.lost_timeout:
                return None  # keep waiting for this person; stop rather than follow anyone else
            self.release()

        confident = [(b, i, c) for b, i, c in zip(boxes, ids, confs) if c >= self.acquire_conf]
        if not confident:
            return None
        box, i, c = max(confident, key=lambda t: (t[0][2] - t[0][0]) * (t[0][3] - t[0][1]))
        self.release()
        self.tid = i
        self.others.update(j for j in ids if j != i)
        self._seen(frame, box, c, now)
        return box

    def _seen(self, frame, box, conf, now):
        self.last_seen = now
        if conf < self.acquire_conf:
            return  # a weak or half-hidden detection would smear the colour signature
        sig = appearance(frame, box)
        if sig is None:
            return
        # adapt slowly to lighting changes, without drifting to whoever walks past
        self.signature = sig if self.signature is None else 0.9 * self.signature + 0.1 * sig

    def status(self, now):
        if self.tid is None:
            return "NO TARGET"
        lost = now - self.last_seen
        if lost < 0.05:
            return f"LOCKED id {self.tid}"
        limit = "" if self.lost_timeout <= 0 else f"/{self.lost_timeout:.0f}"
        return f"SEARCHING id {self.tid} {lost:.1f}{limit} s"

    def release(self):
        self.tid = None
        self.signature = None
        self.others = set()
        self.last_seen = 0.0
        self.reid_count = 0
        self.last_reid_dist = None


def draw_hud(frame, box, label, too_close, steer, throttle, info):
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)

    if box is not None:
        x1, y1, x2, y2 = [int(v) for v in box]
        colour = (0, 165, 255) if too_close else (0, 220, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, label, (x1 + 4, max(20, y1 + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

    # drive command bars, centred at the base of the frame
    base_y, cx = h - 30, w // 2
    cv2.rectangle(frame, (cx - 150, base_y - 8), (cx + 150, base_y + 8), (50, 50, 50), 1)
    cv2.rectangle(frame, (cx, base_y - 8), (cx + int(steer * 150), base_y + 8), (0, 200, 255), -1)
    cv2.putText(frame, "steer", (max(4, cx - 150), base_y - 14),  # above the bar: fits portrait frames
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    bar_x = w - 40
    cv2.rectangle(frame, (bar_x - 8, h // 2 - 100), (bar_x + 8, h // 2 + 100), (50, 50, 50), 1)
    cv2.rectangle(frame, (bar_x - 8, h // 2), (bar_x + 8, h // 2 - int(throttle * 100)), (0, 220, 120), -1)
    cv2.putText(frame, "thr", (bar_x - 20, h // 2 + 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 120), 1)

    for i, line in enumerate(info):
        cv2.putText(frame, line, (10, 22 + 20 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
    return frame
