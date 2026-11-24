"""Exercise the laptop UART bridge with a fake camera and a loopback hub."""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import unittest
from unittest.mock import patch
import urllib.error
import urllib.parse
import urllib.request
import zlib

import cv2
import numpy as np

from tools.camera_stream import iter_jpegs
from tools.laptop_face_bridge import LaptopFaceBridge
from tools.laptop_hub import LaptopHubClient
from tools.serial_camera import HEADER, MAGIC, SerialCamera


class TestUartBridge(unittest.TestCase):
    def test_direct_preview_and_feedback_recover_independently(self):
        _, image = cv2.imencode(".jpg", np.zeros((240, 320, 3), dtype=np.uint8))
        jpeg = image.tobytes()
        camera_online = threading.Event()
        camera_online.set()
        posts = deque()
        observations = deque()
        serial_commands = deque()

        class Camera:
            port = "TEST"
            sequence = 0
            in_waiting = 0

            def __enter__(self):
                if not camera_online.is_set():
                    raise OSError("Camera unplugged")
                return self

            def __exit__(self, *args):
                pass

            def write(self, data):
                serial_commands.append(data)
                return len(data)

            def read(self, _):
                time.sleep(0.03)
                if not camera_online.is_set():
                    raise OSError("Camera unplugged")
                Camera.sequence += 1
                seq = Camera.sequence
                header = HEADER.pack(MAGIC, len(jpeg), seq, seq * 30000, 320, 240, 0)[:-4]
                checksum = zlib.crc32(jpeg, zlib.crc32(header))
                return header + checksum.to_bytes(4, "little") + jpeg

        class Engine:
            faces_dir = "."
            known_names = ["Tester"]
            result = "no_face"

            def analyze_frame(self, image):
                self_result = self.result
                observations.append(self_result)
                return {"result": self_result, "name": "Tester", "confidence": 0.9,
                        "matchId": 0 if self_result == "recognized" else -1}

        class Hub(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                body = self.rfile.read(int(self.headers["Content-Length"])).decode("ascii")
                posts.append((self.path, urllib.parse.parse_qs(body)))
                try:
                    self.send_response(200)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                except (BrokenPipeError, ConnectionResetError):
                    pass  # A queued request can expire during the simulated outage.

        def wait_for(predicate):
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                if predicate():
                    return
                time.sleep(0.025)
            self.fail("Timed out waiting for integrated camera state")

        def posted(event, online="1"):
            return any(path == "/api/laptop/event"
                       and args["event"] == [event] and args["online"] == [online]
                       for path, args in tuple(posts))

        with ThreadingHTTPServer(("127.0.0.1", 0), Hub) as hub:
            engine = Engine()
            client = SerialCamera("TEST", 2_000_000, "qvga", 16, light=(48, 32))
            publisher = LaptopHubClient(f"127.0.0.1:{hub.server_port}", "127.0.0.1", 0)
            bridge = LaptopFaceBridge(client, engine, publisher, show_gui=False)
            hub_thread = threading.Thread(target=hub.serve_forever, daemon=True)
            bridge_thread = threading.Thread(target=bridge.run, daemon=True)
            with patch("tools.serial_camera.open_camera_port", return_value=Camera()):
                bridge_thread.start()
                try:
                    wait_for(lambda: bridge._stream_state()[0])
                    base = f"http://127.0.0.1:{publisher.preview_port}"
                    # The laptop serves video while the hub is still offline.
                    with urllib.request.urlopen(base + "/stream", timeout=2) as response:
                        frames = iter_jpegs(response)
                        self.assertEqual(next(frames), jpeg)
                        self.assertEqual(next(frames), jpeg)
                    wait_for(lambda: serial_commands.count(b"\nSTART qvga 16 48 32\n") >= 2)

                    hub_thread.start()
                    wait_for(lambda: posted("heartbeat"))
                    engine.result = "unknown"
                    wait_for(lambda: posted("unknown"))
                    self.assertGreaterEqual(observations.count("unknown"), 3)
                    engine.result = "recognized"
                    wait_for(lambda: posted("recognized"))
                    event = next(args for _, args in tuple(posts) if args["event"] == ["recognized"])
                    self.assertEqual(event["port"], [str(publisher.preview_port)])
                    self.assertEqual(event["matchId"], ["0"])

                    post_count = len(posts)
                    camera_online.clear()
                    wait_for(lambda: any(args["event"] == ["heartbeat"] and args["online"] == ["0"]
                                         for _, args in tuple(posts)[post_count:]))
                    with self.assertRaises(urllib.error.HTTPError) as offline:
                        urllib.request.urlopen(base + "/preview.jpg", timeout=2)
                    with offline.exception as response:
                        self.assertEqual(response.code, 503)

                    engine.result = "no_face"
                    camera_online.set()
                    wait_for(lambda: bridge._stream_state()[0])
                    wait_for(lambda: any(args["online"] == ["1"] and args["version"] != event["version"]
                                         for _, args in tuple(posts)))
                    with urllib.request.urlopen(base + "/preview.jpg", timeout=2) as response:
                        self.assertEqual(response.read(), jpeg)
                    self.assertEqual(sum(args["event"] == ["recognized"] for _, args in tuple(posts)), 1)
                finally:
                    bridge.stop()
                    bridge_thread.join(timeout=3)
                    if hub_thread.is_alive():
                        hub.shutdown()
                        hub_thread.join(timeout=3)
                self.assertFalse(bridge_thread.is_alive())
                self.assertIn(b"\nSTOP\n", serial_commands)


if __name__ == "__main__":
    unittest.main()
