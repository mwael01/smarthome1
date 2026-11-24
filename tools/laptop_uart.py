"""Serial camera input, laptop HTTP preview, and Wi-Fi hub feedback."""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import secrets
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

if __package__:
    from .usb_camera_preview import FrameParser, iter_serial_frames, open_camera_port
else:
    from usb_camera_preview import FrameParser, iter_serial_frames, open_camera_port


FRAME_MAX_AGE = 1.5
UNKNOWN_CONFIRMATIONS = 3
EVENT_COOLDOWN = {"recognized": 2.5, "unknown": 4.0}


class UartFaceClient:
    def __init__(self, port, baud, resolution, quality, hub, preview_bind, preview_port):
        self.port = port
        self.baud = baud
        self.resolution = resolution
        self.quality = quality
        self.preview_bind = preview_bind
        self.preview_port = preview_port
        self.stream_url = f"{port} ({baud} baud)"
        self.feedback_url = f"http://{hub}/api/laptop/event"
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._sequence = secrets.randbelow(0x7ffffffe)
        self._preview_version = secrets.randbelow(0x7ffffffe)
        self._verdict_lock = threading.Lock()
        self._last_observation = 0.0
        self._unknown_count = 0
        self._verdict = None

    def iter_mjpeg_frames(self, *, stop):
        with open_camera_port(self.port, uart=True, baud=self.baud) as port:
            for frame in iter_serial_frames(port, self.resolution, self.quality, stop, 5, FrameParser()):
                yield frame.jpeg

    def observe(self, verdict, frame_at):
        """Keep only the newest actionable result from a fresh camera frame."""
        result = verdict.get("result")
        now = time.monotonic()
        with self._verdict_lock:
            if result != "unknown" or now - self._last_observation > FRAME_MAX_AGE:
                self._unknown_count = 0
            self._last_observation = now
            self._verdict = None
            if now - frame_at > FRAME_MAX_AGE:
                self._unknown_count = 0
                return
            if result == "unknown":
                self._unknown_count += 1
                if self._unknown_count < UNKNOWN_CONFIRMATIONS:
                    return
            if result in EVENT_COOLDOWN:
                self._verdict = frame_at, verdict

    def _post(self, event, version, verdict):
        data = {"event": event, "online": int(version != 0), "port": self.preview_port,
                "version": version}
        if verdict is not None:
            self._sequence = self._sequence % 0x7ffffffe + 1
            label = verdict["name"].encode("utf-8")[:23].decode("utf-8", errors="ignore")
            data.update(seq=self._sequence, matchId=verdict["matchId"],
                        label=label, confidence=round(verdict["confidence"] * 100))
        request = urllib.request.Request(self.feedback_url, urllib.parse.urlencode(data).encode("ascii"))
        try:
            with self._opener.open(request, timeout=0.6) as response:
                response.read()
        except urllib.error.HTTPError as exc:
            exc.close()
            raise

    def run_feedback(self, stream_state, stop):
        next_heartbeat = 0.0
        last_version = -1
        last_event = {}
        link_failed = False
        while not stop.is_set():
            now = time.monotonic()
            online, generation = stream_state()
            version = (self._preview_version + generation) % 0x7ffffffe + 1 if online else 0
            with self._verdict_lock:
                sample = self._verdict
            verdict = None
            event = "heartbeat"
            if online and sample is not None:
                frame_at, candidate = sample
                result = candidate["result"]
                cooldown = EVENT_COOLDOWN[result]
                if now - frame_at <= FRAME_MAX_AGE and now - last_event.get(result, -float("inf")) >= cooldown:
                    verdict, event = candidate, result
                    last_event[result] = now
            if verdict is not None or now >= next_heartbeat or version != last_version:
                next_heartbeat, last_version = now + 2.0, version
                try:
                    self._post(event, version, verdict)
                    if link_failed:
                        print("[UART-CAM] Hub Wi-Fi connection restored", flush=True)
                    link_failed = False
                    if verdict is not None:
                        print(f"[UART-CAM] Sent {event} to hub", flush=True)
                except OSError as exc:
                    if not link_failed:
                        print(f"[UART-CAM] Hub unavailable ({exc}). Camera preview continues.", flush=True)
                    link_failed = True
            stop.wait(0.1)


def make_preview_server(bind, port, latest_frame):
    viewers = threading.BoundedSemaphore(4)
    boundary = b"frame"

    class PreviewHandler(BaseHTTPRequestHandler):
        def setup(self):
            self.request.settimeout(2.0)
            super().setup()

        def log_message(self, *args):
            pass

        def do_GET(self):
            path = urllib.parse.urlsplit(self.path).path
            if path not in ("/stream", "/preview.jpg"):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            if not viewers.acquire(blocking=False):
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "Preview viewer limit reached")
                return
            try:
                frame, sequence = latest_frame()
                if frame is None:
                    self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "Camera feed offline")
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-store")
                if path == "/preview.jpg":
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    return

                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                while frame is not None:
                    self.wfile.write(b"--" + boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(frame)).encode("ascii") + b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    frame, sequence = latest_frame(sequence)
                self.wfile.write(b"--" + boundary + b"--\r\n")
            except OSError:
                pass
            finally:
                viewers.release()

    return ThreadingHTTPServer((bind, port), PreviewHandler)
