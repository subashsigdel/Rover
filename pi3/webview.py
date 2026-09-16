"""Live view of what the follower sees, served as MJPEG over HTTP.

    browser:  http://<pi-address>:8080/            (snapshot: /snapshot.jpg)
    cv2:      python camview.py --source http://<pi-address>:8080/

Frames are only JPEG-encoded while someone is watching: on a Pi 3 that costs
about 10 ms per frame, time better spent on detection.
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


def _encode(frame):
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes() if ok else None


class FrameServer:
    def __init__(self, port):
        self.cond = threading.Condition()
        self.frame = None       # newest annotated frame, kept for /snapshot.jpg
        self.jpeg = None        # newest encoded frame, only made while clients > 0
        self.jpeg_seq = 0
        self.clients = 0
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # one line per request would flood the follower's log

            def do_GET(self):
                if self.path.startswith("/snapshot"):
                    with server.cond:
                        frame = server.frame
                    jpeg = _encode(frame) if frame is not None else None
                    if jpeg is None:
                        self.send_error(503, "no frame yet")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with server.cond:
                    server.clients += 1
                last = -1
                try:
                    while True:
                        with server.cond:
                            server.cond.wait_for(lambda: server.jpeg_seq != last, timeout=5.0)
                            if server.jpeg_seq == last or server.jpeg is None:
                                continue
                            jpeg, last = server.jpeg, server.jpeg_seq
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # viewer closed
                finally:
                    with server.cond:
                        server.clients -= 1

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def publish(self, frame):
        with self.cond:
            self.frame = frame
            watching = self.clients > 0
        if not watching:
            return
        jpeg = _encode(frame)
        if jpeg is not None:
            with self.cond:
                self.jpeg, self.jpeg_seq = jpeg, self.jpeg_seq + 1
                self.cond.notify_all()
