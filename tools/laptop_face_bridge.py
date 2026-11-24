#!/usr/bin/env python3
"""Recognize faces from the serial camera and send verdicts to the Wi-Fi hub."""

import argparse
from dataclasses import dataclass
import multiprocessing
import os
import re
import signal
import statistics
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

if __package__:
    from .camera_stream import RESOLUTIONS
    from .laptop_hub import DEFAULT_PREVIEW_PORT, FRAME_MAX_AGE, LaptopHubClient, make_preview_server
    from .serial_camera import SerialCamera, UART_BAUD, add_light_argument, validate_light
else:
    from camera_stream import RESOLUTIONS
    from laptop_hub import DEFAULT_PREVIEW_PORT, FRAME_MAX_AGE, LaptopHubClient, make_preview_server
    from serial_camera import SerialCamera, UART_BAUD, add_light_argument, validate_light


OVERLAY_MAX_AGE = FRAME_MAX_AGE
DEFAULT_TOLERANCE = 0.6
MARGIN = 0.06
_IDENTITY_RE = re.compile(r"^(?P<name>.+)-\d+\.(?:jpe?g|png)$", re.IGNORECASE)
_SUPPORTED_EXTS = (".jpg", ".jpeg", ".png")


def decode_jpeg(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)


def _identity_from_filename(fname: str) -> str:
    match = _IDENTITY_RE.match(fname)
    if match:
        return match.group("name").strip().lower()
    stem = os.path.splitext(fname)[0].strip().lower()
    return re.sub(r"[\s_]+", "-", stem)


def _display_name(identity: str) -> str:
    return identity.replace("-", " ").replace("_", " ").title()


class FastFaceEngine:
    """Original face_recognition HOG detector + grouped median ResNet embedding matcher."""

    def __init__(self, faces_dir: str, distance_thresh: float = DEFAULT_TOLERANCE, faces_module=None):
        if faces_module is None:
            import face_recognition as faces_module

        self._faces = faces_module
        self.faces_dir = os.path.abspath(faces_dir)
        self.distance_thresh = distance_thresh
        os.makedirs(self.faces_dir, exist_ok=True)

        self.known: Dict[str, List[np.ndarray]] = {}
        self.known_names: List[str] = []
        self.lock = threading.Lock()
        self.load_known_faces()

    def load_known_faces(self):
        with self.lock:
            self.known.clear()
            self.known_names.clear()
            if not os.path.isdir(self.faces_dir):
                return
            total_photos = 0
            for fname in sorted(os.listdir(self.faces_dir)):
                if fname.lower().endswith(_SUPPORTED_EXTS):
                    path = os.path.join(self.faces_dir, fname)
                    try:
                        img = self._faces.load_image_file(path)
                        encs = self._faces.face_encodings(img)
                        if not encs:
                            print(f"[FACE-ENGINE] No face found in {fname}, skipping", file=sys.stderr)
                            continue
                        identity = _identity_from_filename(fname)
                        self.known.setdefault(identity, []).append(encs[0])
                        total_photos += 1
                    except Exception as e:
                        print(f"[FACE-ENGINE] Error loading {fname}: {e}", file=sys.stderr)
            self.known_names = list(self.known.keys())
            for identity in self.known_names:
                print(
                    f"[FACE-ENGINE] Loaded identity: \"{_display_name(identity)}\" "
                    f"({len(self.known[identity])} photo(s))"
                )
            print(
                f"[FACE-ENGINE] Total enrolled identities active: "
                f"{len(self.known_names)} ({total_photos} photo(s))"
            )

    def enroll(self, bgr_img: np.ndarray, box: List[int], name: str) -> bool:
        """Enroll face into memory and persist to known_faces/<identity>-<next_index>.jpg."""
        left, top, right, bottom = box
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        dlib_box = (top, right, bottom, left)
        encs = self._faces.face_encodings(rgb, [dlib_box])
        if not encs:
            return False

        with self.lock:
            clean_id = re.sub(r"[^a-zA-Z0-9_-]", "", name.strip().lower().replace(" ", "-"))
            if not clean_id:
                clean_id = f"user-{len(self.known_names) + 1}"

            existing_indices = []
            for fname in os.listdir(self.faces_dir):
                if not fname.lower().endswith(_SUPPORTED_EXTS):
                    continue
                match = _IDENTITY_RE.match(fname)
                if match and match.group("name").lower() == clean_id:
                    num_match = re.search(r"-(\d+)\.", fname)
                    if num_match:
                        existing_indices.append(int(num_match.group(1)))

            next_idx = max(existing_indices, default=0) + 1

            # Save cropped face photo with padding for future reference loading
            h, w = bgr_img.shape[:2]
            pad_x = int((right - left) * 0.25)
            pad_y = int((bottom - top) * 0.25)
            crop_x1 = max(0, left - pad_x)
            crop_y1 = max(0, top - pad_y)
            crop_x2 = min(w, right + pad_x)
            crop_y2 = min(h, bottom + pad_y)
            crop = bgr_img[crop_y1:crop_y2, crop_x1:crop_x2]

            fname = f"{clean_id}-{next_idx}.jpg"
            save_path = os.path.join(self.faces_dir, fname)
            if not cv2.imwrite(save_path, crop if crop.size > 0 else bgr_img):
                return False

            if clean_id not in self.known:
                self.known[clean_id] = []
                self.known_names.append(clean_id)
            self.known[clean_id].append(encs[0])

            print(f"[FACE-ENGINE] Successfully enrolled \"{_display_name(clean_id)}\" -> saved to {save_path}")
            return True

    def analyze_frame(self, bgr_img: np.ndarray) -> dict:
        t0 = time.perf_counter()
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]

        # Original face_demo.py detection sequence: HOG upsample=1 with upsample=0 fallback.
        locations = self._faces.face_locations(rgb, number_of_times_to_upsample=1)
        if not locations:
            locations = self._faces.face_locations(rgb, number_of_times_to_upsample=0)

        if not locations:
            infer_ms = int((time.perf_counter() - t0) * 1000.0)
            return {
                "result": "no_face",
                "faceDetected": False,
                "faceQuality": "NO_FACE",
                "box": [0, 0, 0, 0],
                "faces": [],
                "inferenceMs": infer_ms,
                "name": "---",
                "confidence": 0.0,
                "matchId": -1,
            }

        clamped_locations = []
        for top, right, bottom, left in locations:
            top = max(0, min(h - 1, int(top)))
            bottom = max(0, min(h - 1, int(bottom)))
            left = max(0, min(w - 1, int(left)))
            right = max(0, min(w - 1, int(right)))
            clamped_locations.append((top, right, bottom, left))

        encs = self._faces.face_encodings(rgb, clamped_locations)
        infer_ms = int((time.perf_counter() - t0) * 1000.0)
        if not encs:
            top, right, bottom, left = max(
                clamped_locations, key=lambda b: (b[2] - b[0]) * (b[1] - b[3])
            )
            box = [left, top, right, bottom]
            return {
                "result": "aligning",
                "faceDetected": True,
                "faceQuality": "BLURRY",
                "box": box,
                "faces": [{
                    "result": "aligning",
                    "box": box,
                    "name": "Aligning",
                    "confidence": 0.0,
                    "matchId": -1,
                }],
                "inferenceMs": infer_ms,
                "name": "Aligning",
                "confidence": 0.0,
                "matchId": -1,
            }

        faces = []
        with self.lock:
            for (top, right, bottom, left), encoding in zip(clamped_locations, encs):
                box = [left, top, right, bottom]
                if not self.known:
                    faces.append({
                        "result": "unknown",
                        "box": box,
                        "name": "Unknown",
                        "confidence": 0.0,
                        "matchId": -1,
                        "unregistered": True,
                    })
                    continue

                ranked = sorted(
                    (
                        float(statistics.median(self._faces.face_distance(refs, encoding))),
                        idx,
                        identity,
                    )
                    for idx, (identity, refs) in enumerate(self.known.items())
                )
                best_dist, best_idx, identity = ranked[0]
                sim = max(0.0, min(1.0, 1.0 - (best_dist / self.distance_thresh) * 0.5))
                clear_winner = len(ranked) == 1 or best_dist + MARGIN < ranked[1][0]
                if best_dist <= self.distance_thresh and clear_winner:
                    faces.append({
                        "result": "recognized",
                        "box": box,
                        "name": _display_name(identity),
                        "confidence": sim,
                        "matchId": best_idx,
                    })
                else:
                    faces.append({
                        "result": "unknown",
                        "box": box,
                        "name": "Unknown",
                        "confidence": sim,
                        "matchId": -1,
                    })

        primary = next((face for face in faces if face["result"] == "recognized"), faces[0])
        verdict = {
            "result": primary["result"],
            "faceDetected": True,
            "faceQuality": "READY",
            "box": primary["box"],
            "faces": faces,
            "inferenceMs": infer_ms,
            "name": primary["name"],
            "confidence": primary["confidence"],
            "matchId": primary["matchId"],
        }
        if primary.get("unregistered"):
            verdict["unregistered"] = True
        return verdict

    def analyze_jpeg(self, jpeg):
        image = decode_jpeg(jpeg)
        return self.analyze_frame(image) if image is not None else None

    def enroll_jpeg(self, jpeg, box, name):
        image = decode_jpeg(jpeg)
        return image is not None and self.enroll(image, box, name)


def run_face_worker(connection, faces_dir, threshold):
    # The preview process handles Ctrl+C and closes the pipe on exit.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    cv2.setNumThreads(1)
    with connection:
        try:
            engine = FastFaceEngine(faces_dir, distance_thresh=threshold)
        except Exception as exc:
            print(f"[FACE-ENGINE] Cannot load recognition model: {exc}", file=sys.stderr)
            return
        while True:
            try:
                method, args = connection.recv()
            except (EOFError, OSError):
                return
            try:
                response = True, getattr(engine, method)(*args)
            except Exception as exc:
                response = False, str(exc)
            try:
                connection.send(response)
            except (EOFError, OSError):
                return


class FaceWorker:
    """Run dlib outside the preview process so its calls cannot hold the preview's GIL."""

    def __init__(self, faces_dir, threshold):
        self.faces_dir = os.path.abspath(faces_dir)
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe()
        self._process = context.Process(target=run_face_worker,
                                        args=(child, self.faces_dir, threshold), daemon=True)
        try:
            self._process.start()
        except BaseException:
            self._connection.close()
            raise
        finally:
            child.close()

    def _request(self, method, *args):
        self._connection.send((method, args))
        ok, response = self._connection.recv()
        if not ok:
            raise RuntimeError(response)
        return response

    def analyze_jpeg(self, jpeg):
        return self._request("analyze_jpeg", jpeg)

    def enroll_jpeg(self, jpeg, box, name):
        return self._request("enroll_jpeg", jpeg, box, name)

    def load_known_faces(self):
        self._request("load_known_faces")

    def close(self):
        self._connection.close()
        self._process.join(timeout=0.25)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)


@dataclass(frozen=True)
class FaceDetection:
    jpeg: bytes
    frame_at: float
    verdict: dict


class LaptopFaceBridge:
    """Keep the newest serial frame available to the detector and dashboard."""

    def __init__(self, client, engine, hub, show_gui: bool = True):
        self.client = client
        self.engine = engine
        self.hub = hub
        self.show_gui = show_gui
        self.frames_count = 0
        self._analyzed_count = 0

        self._stop = threading.Event()
        self._latest_jpg: Optional[bytes] = None
        self._latest_frame_at = 0.0
        self._stream_generation = 0
        self._detection = None
        self._analysis_action = None
        self._jpg_lock = threading.Condition()

    def stop(self):
        self._stop.set()
        with self._jpg_lock:
            self._jpg_lock.notify_all()

    def _preview_frame(self, after=None):
        with self._jpg_lock:
            if after is not None:
                self._jpg_lock.wait_for(
                    lambda: self.frames_count != after or self._latest_jpg is None or self._stop.is_set(),
                    timeout=FRAME_MAX_AGE,
                )
            frame = self._latest_jpg if self.frames_count != after and self._recent_frame() else None
            return frame, self.frames_count

    def _recent_frame(self):
        return (self._latest_jpg is not None and not self._stop.is_set()
                and time.monotonic() - self._latest_frame_at <= FRAME_MAX_AGE)

    def _stream_state(self):
        with self._jpg_lock:
            return self._recent_frame(), self._stream_generation

    def _stream_reader_loop(self):
        while not self._stop.is_set():
            retry_delay = 1.0
            try:
                for jpg in self.client.iter_jpegs(stop=self._stop):
                    if self._stop.is_set():
                        break
                    with self._jpg_lock:
                        self._latest_jpg = jpg
                        self._latest_frame_at = time.monotonic()
                        self.frames_count += 1
                        self._jpg_lock.notify_all()
                    retry_delay = 0.2
            except OSError as exc:
                if not self._stop.is_set():
                    print(f"[LAPTOP-AI] Stream reconnecting ({exc})...", file=sys.stderr)
            finally:
                with self._jpg_lock:
                    self._latest_jpg = None
                    self._stream_generation += 1
                    self._detection = None
                    self.hub.observe({}, time.monotonic())
                    self._jpg_lock.notify_all()
            self._stop.wait(retry_delay)

    def _analysis_loop(self):
        analyzed_frame = 0
        failed = False
        while not self._stop.is_set():
            with self._jpg_lock:
                self._jpg_lock.wait_for(lambda: (
                    self._stop.is_set() or self._analysis_action is not None
                    or (self._latest_jpg is not None and self.frames_count != analyzed_frame)
                ))
                if self._stop.is_set():
                    return
                action = self._analysis_action
                self._analysis_action = None
                if action is None:
                    analyzed_frame = self.frames_count
                    if not self._recent_frame():
                        continue
                    jpeg = self._latest_jpg
                    frame_at = self._latest_frame_at
                    generation = self._stream_generation
            try:
                if action == ("reload",):
                    self.engine.load_known_faces()
                elif action is not None:
                    if not self.engine.enroll_jpeg(*action[1:]):
                        print("[ENROLL] Could not save a face from that frame.", flush=True)
                else:
                    verdict = self.engine.analyze_jpeg(jpeg)
                    if verdict is not None:
                        with self._jpg_lock:
                            self._analyzed_count += 1
                            if (self._stream_generation == generation and self._recent_frame()
                                    and time.monotonic() - frame_at <= FRAME_MAX_AGE):
                                self._detection = FaceDetection(jpeg, frame_at, verdict)
                                self.hub.observe(verdict, frame_at)
                failed = False
            except (EOFError, OSError, RuntimeError) as exc:
                if not failed and not self._stop.is_set():
                    print(f"[FACE-ENGINE] Recognition unavailable: {exc}", file=sys.stderr)
                failed = True
                self._stop.wait(0.25)

    def _queue_analysis_action(self, action):
        with self._jpg_lock:
            if self._analysis_action is None and not self._stop.is_set():
                self._analysis_action = action
                self._jpg_lock.notify_all()

    def _prompt_enrollment(self, detection):
        if sys.stdin is None or not sys.stdin.isatty():
            print("[ENROLL] Open the bridge in an interactive terminal to enter a name.", flush=True)
            return
        try:
            name = input("\n[ENROLL] Enter name for face in view: ").strip()
        except (EOFError, OSError):
            return
        self._queue_analysis_action(("enroll", detection.jpeg, detection.verdict["box"], name))

    def draw_hud(self, bgr_img: Optional[np.ndarray], verdict: dict, fps: float,
                 size: Tuple[int, int] = (960, 720), *, rx_fps=0.0, ai_fps=0.0) -> np.ndarray:
        width, height = size
        canvas = np.empty((height, width, 3), dtype=np.uint8)
        cv2.rectangle(canvas, (0, 0), (width - 1, height - 1), (24, 21, 18), cv2.FILLED)
        header, footer = min(60, height // 4), min(68, height // 4)
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = (241, 237, 230)
        muted = (174, 165, 156)
        teal = (190, 227, 89)
        live = bgr_img is not None

        def write(label, x, y, scale, color, available, thickness=1):
            (pixels, _), _ = cv2.getTextSize(label, font, scale, thickness)
            cv2.putText(canvas, label, (x, y), font, scale * min(1, available / max(1, pixels)),
                        color, thickness, cv2.LINE_AA)

        if header >= 28 and width >= 100:
            cv2.line(canvas, (0, header - 1), (width - 1, header - 1), (55, 48, 40))
            if width >= 285:
                cv2.circle(canvas, (width - 101, header // 2), 4, teal if live else muted, -1, cv2.LINE_AA)
                write("LIVE" if live else "WAITING", width - 89, header // 2 + 5,
                      0.47, teal if live else muted, 77)
            write("Face recognition", 18, header // 2 + 7, 0.67, text,
                  width - (145 if width >= 285 else 36), 2)

        if live:
            source_h, source_w = bgr_img.shape[:2]
            available_h = height - header - footer
            scale = min(width / source_w, available_h / source_h)
            video_w, video_h = max(1, round(source_w * scale)), max(1, round(source_h * scale))
            offset_x = (width - video_w) // 2
            offset_y = header + (available_h - video_h) // 2
            video = canvas[offset_y:offset_y + video_h, offset_x:offset_x + video_w]
            interpolation = cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
            cv2.resize(bgr_img, (video_w, video_h), dst=video, interpolation=interpolation)

            faces = verdict.get("faces")
            if not faces and verdict.get("faceDetected") and verdict.get("box") is not None:
                faces = [verdict]
            for face in faces or ():
                box = face.get("box")
                if box is None or len(box) != 4:
                    continue
                x1, y1, x2, y2 = (
                    round(box[0] * video_w / source_w), round(box[1] * video_h / source_h),
                    round(box[2] * video_w / source_w), round(box[3] * video_h / source_h),
                )
                x1, x2 = max(0, x1), min(video_w - 1, x2)
                y1, y2 = max(0, y1), min(video_h - 1, y2)
                if x2 > x1 and y2 > y1:
                    result = face.get("result")
                    if result == "recognized":
                        color = teal
                        tag = f"{face.get('name', 'Recognized')}  {face.get('confidence', 0):.0%}"
                    elif result == "unknown":
                        color, tag = (113, 128, 255), "Unknown face"
                    else:
                        color, tag = (104, 208, 250), "Face detected"
                    corner = min(20, (x2 - x1) // 3, (y2 - y1) // 3)
                    for x, dx in ((x1, 1), (x2, -1)):
                        for y, dy in ((y1, 1), (y2, -1)):
                            cv2.line(video, (x, y), (x + dx * corner, y), color, 2, cv2.LINE_AA)
                            cv2.line(video, (x, y), (x, y + dy * corner), color, 2, cv2.LINE_AA)

                    if video_w >= 80 and video_h >= 30:
                        (tag_w, _), _ = cv2.getTextSize(tag, font, 0.52, 1)
                        tag_w = min(tag_w + 16, video_w)
                        tag_x = min(x1, video_w - tag_w)
                        tag_y = max(0, y1 - 28) if y1 >= 28 else min(y2 + 5, video_h - 23)
                        cv2.rectangle(video, (tag_x, tag_y), (tag_x + tag_w - 1, tag_y + 22), color, -1)
                        write(tag, offset_x + tag_x + 8, offset_y + tag_y + 16, 0.52, (26, 25, 23), tag_w - 16)
        elif width >= 100:
            write("Waiting for camera", 18, height // 2, 0.62, muted, width - 36)

        if footer >= 25 and width >= 100:
            cv2.line(canvas, (0, height - footer), (width - 1, height - footer), (55, 48, 40))
            if live:
                stats = (f"{source_w}x{source_h}  |  RX {rx_fps:.1f} FPS"
                         f"  |  Preview {fps:.1f} FPS  |  AI {ai_fps:.1f} FPS")
            else:
                stats = self.client.description
            write(stats, 18, height - footer + 23, 0.52, text, width - 36)
            if footer >= 48 and width >= 210:
                write("E  Enroll     R  Reload faces     Q  Quit", 18, height - 15, 0.46, muted, width - 36)
        return canvas

    def _preview_loop(self):
        window = "Smart Home Face Recognition - Camera 1"
        size = (960, 720)
        cv2.setNumThreads(1)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
        cv2.resizeWindow(window, *size)
        cv2.imshow(window, self.draw_hud(None, {}, 0.0, size))

        decoded_frame = last_rx = last_ai = presented = 0
        sample_at = time.monotonic()
        rx_fps = preview_fps = ai_fps = 0.0
        image = painted_image = painted_detection = None
        painted_size = size
        enrollment_prompt = None

        while not self._stop.is_set():
            with self._jpg_lock:
                jpeg = self._latest_jpg if self._recent_frame() else None
                rx_count = self.frames_count
                ai_count = self._analyzed_count
                detection = self._detection
            now = time.monotonic()
            if jpeg is None:
                image = None
            elif rx_count != decoded_frame:
                decoded_frame = rx_count
                image = decode_jpeg(jpeg)

            if image is None or (detection is not None and now - detection.frame_at > OVERLAY_MAX_AGE):
                detection = None
            verdict = detection.verdict if detection is not None else {}
            try:
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    return
                _, _, width, height = cv2.getWindowImageRect(window)
                if width > 0 and height > 0:
                    size = width, height
            except cv2.error:
                pass  # Some HighGUI backends do not expose the drawable size.

            elapsed = now - sample_at
            rates_updated = elapsed >= 1.0
            if rates_updated:
                rx_fps = (rx_count - last_rx) / elapsed
                preview_fps = presented / elapsed
                ai_fps = (ai_count - last_ai) / elapsed
                sample_at, last_rx, last_ai, presented = now, rx_count, ai_count, 0

            new_video = image is not None and image is not painted_image
            if (image is not painted_image or detection is not painted_detection or size != painted_size
                    or (image is not None and rates_updated)):
                cv2.imshow(window, self.draw_hud(image, verdict, preview_fps, size,
                                                rx_fps=rx_fps, ai_fps=ai_fps))
                painted_image, painted_detection, painted_size = image, detection, size
                presented += int(new_video)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                return
            if key in (ord('e'), ord('E')):
                if detection is None or not verdict.get("faceDetected"):
                    print("[ENROLL] Look into the camera so a face can be detected.")
                elif enrollment_prompt is None or not enrollment_prompt.is_alive():
                    enrollment_prompt = threading.Thread(target=self._prompt_enrollment,
                                                         args=(detection,), daemon=True)
                    enrollment_prompt.start()
            elif key in (ord('r'), ord('R')):
                self._queue_analysis_action(("reload",))

            if not new_video:
                self._stop.wait(0.005)

    def run(self):
        print("\n" + "=" * 70)
        print(" [SMART HOME] Laptop Face Recognition AI Online")
        print(f" - Camera Stream: {self.client.description}")
        print(f" - Feedback API: {self.hub.feedback_url}")
        print(f" - Face directory: {self.engine.faces_dir}")
        if self.show_gui:
            print(" - Hotkeys: Press 'e' in preview window to enroll, 'q' to quit")
        print("=" * 70 + "\n")

        preview_server = None
        preview_worker = None
        workers = []
        try:
            preview_server = make_preview_server(self.hub.preview_bind, self.hub.preview_port,
                                                 self._preview_frame)
            self.hub.preview_port = preview_server.server_port
            print(f"[LAPTOP-HUB] Dashboard preview listening on TCP port {self.hub.preview_port}.", flush=True)
            preview_worker = threading.Thread(target=preview_server.serve_forever, daemon=True)
            workers = [
                preview_worker,
                threading.Thread(target=self.hub.run_feedback,
                                 args=(self._stream_state, self._stop), daemon=True),
                threading.Thread(target=self._stream_reader_loop, daemon=True),
                threading.Thread(target=self._analysis_loop, daemon=True),
            ]
            for worker in workers:
                worker.start()

            if self.show_gui:
                self._preview_loop()
            else:
                # A bounded wait lets Ctrl+C through on older Windows Python builds.
                while not self._stop.wait(0.25):
                    pass

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            if preview_server is not None:
                if preview_worker is not None and preview_worker.is_alive():
                    preview_server.shutdown()
                preview_server.server_close()
            for worker in workers:
                if worker.is_alive():
                    worker.join(timeout=1.0)
            if self.show_gui:
                cv2.destroyAllWindows()
            print("\n[LAPTOP-AI] Bridge stopped cleanly.")

def main():
    parser = argparse.ArgumentParser(description="High-Speed Laptop Face Recognition Bridge for ESP32-CAM")
    parser.add_argument("--uart-port", required=True, help="CH340 port, such as COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=UART_BAUD, help="Camera UART baud rate")
    parser.add_argument("--hub", default="192.168.4.1", help="Hub address for dashboard preview and feedback")
    parser.add_argument("--preview-bind", default="0.0.0.0", help="Laptop interface for the dashboard preview")
    parser.add_argument("--preview-port", type=int,
                        help=f"Fixed dashboard preview port (default: {DEFAULT_PREVIEW_PORT} or an available port)")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="qvga", help="Set the camera resolution before streaming")
    parser.add_argument("--jpeg-quality", type=int, default=16,
                        help="Set JPEG compression. Higher numbers send smaller frames")
    parser.add_argument("--faces-dir", default=os.path.join(os.path.dirname(__file__), "known_faces"),
                        help="Directory containing <Name>.jpg photos of authorized persons")
    parser.add_argument("--thresh", type=float, default=DEFAULT_TOLERANCE,
                        help=f"Euclidean face distance threshold (default: {DEFAULT_TOLERANCE}, lower=stricter)")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless background mode without OpenCV window")
    add_light_argument(parser)
    args = parser.parse_args()
    validate_light(parser, args.light)
    if not 10 <= args.jpeg_quality <= 63:
        parser.error("--jpeg-quality must be between 10 and 63")
    if args.preview_port is not None and not 1 <= args.preview_port <= 65535:
        parser.error("--preview-port must be 1–65535")
    if args.baud <= 0:
        parser.error("--baud must be positive")
    client = SerialCamera(args.uart_port, args.baud, args.resolution, args.jpeg_quality, args.light)
    hub = LaptopHubClient(args.hub, args.preview_bind, args.preview_port)
    engine = None
    try:
        engine = FaceWorker(args.faces_dir, args.thresh)
        LaptopFaceBridge(client, engine, hub, show_gui=not args.no_gui).run()
    except OSError as exc:
        parser.exit(1, f"Laptop face bridge failed: {exc}\n")
    finally:
        if engine is not None:
            engine.close()


if __name__ == "__main__":
    main()
