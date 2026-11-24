#!/usr/bin/env python3
"""Recognize faces from the serial camera and send verdicts to the Wi-Fi hub."""

import argparse
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import dlib
import face_recognition
import numpy as np

if __package__:
    from .camera_stream import RESOLUTIONS
    from .laptop_hub import DEFAULT_PREVIEW_PORT, FRAME_MAX_AGE, LaptopHubClient, make_preview_server
    from .serial_camera import SerialCamera, UART_BAUD, add_light_argument, validate_light
else:
    from camera_stream import RESOLUTIONS
    from laptop_hub import DEFAULT_PREVIEW_PORT, FRAME_MAX_AGE, LaptopHubClient, make_preview_server
    from serial_camera import SerialCamera, UART_BAUD, add_light_argument, validate_light


class FastFaceEngine:
    """High-speed multi-scale face detector and ResNet embedding matcher."""

    def __init__(self, faces_dir: str, distance_thresh: float = 0.58):
        self.faces_dir = os.path.abspath(faces_dir)
        self.distance_thresh = distance_thresh
        os.makedirs(self.faces_dir, exist_ok=True)

        self.known_encodings: List[np.ndarray] = []
        self.known_names: List[str] = []
        self.dlib_detector = dlib.get_frontal_face_detector()
        self.lock = threading.Lock()
        self.load_known_faces()

    def load_known_faces(self):
        with self.lock:
            self.known_encodings.clear()
            self.known_names.clear()
            if not os.path.isdir(self.faces_dir):
                return
            for fname in sorted(os.listdir(self.faces_dir)):
                if fname.lower().endswith((".jpg", ".jpeg", ".png")):
                    path = os.path.join(self.faces_dir, fname)
                    try:
                        img = face_recognition.load_image_file(path)
                        encs = face_recognition.face_encodings(img)
                        if encs:
                            self.known_encodings.append(encs[0])
                            name = os.path.splitext(fname)[0].replace("_", " ")
                            self.known_names.append(name)
                            print(f"[FACE-ENGINE] Loaded enrolled profile: \"{name}\" ({fname})")
                    except Exception as e:
                        print(f"[FACE-ENGINE] Error loading {fname}: {e}", file=sys.stderr)
            print(f"[FACE-ENGINE] Total enrolled faces active: {len(self.known_names)}")

    def enroll(self, bgr_img: np.ndarray, box: List[int], name: str) -> bool:
        """Enroll face into memory and persist to known_faces/<Name>.jpg."""
        left, top, right, bottom = box
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        dlib_box = (top, right, bottom, left)
        encs = face_recognition.face_encodings(rgb, [dlib_box])
        if not encs:
            return False

        with self.lock:
            clean_name = "".join(c for c in name if c.isalnum() or c in (" ", "-", "_")).strip()
            if not clean_name:
                clean_name = f"User_{len(self.known_names) + 1}"

            # Save full image or cropped face photo
            h, w = bgr_img.shape[:2]
            pad_x = int((right - left) * 0.25)
            pad_y = int((bottom - top) * 0.25)
            crop_x1 = max(0, left - pad_x)
            crop_y1 = max(0, top - pad_y)
            crop_x2 = min(w, right + pad_x)
            crop_y2 = min(h, bottom + pad_y)
            crop = bgr_img[crop_y1:crop_y2, crop_x1:crop_x2]

            fname = f"{clean_name.replace(' ', '_')}.jpg"
            save_path = os.path.join(self.faces_dir, fname)
            cv2.imwrite(save_path, crop if crop.size > 0 else bgr_img)

            if clean_name in self.known_names:
                idx = self.known_names.index(clean_name)
                self.known_encodings[idx] = encs[0]
            else:
                self.known_names.append(clean_name)
                self.known_encodings.append(encs[0])

            print(f"[FACE-ENGINE] Successfully enrolled \"{clean_name}\" -> saved to {save_path}")
            return True

    def analyze_frame(self, bgr_img: np.ndarray) -> dict:
        t0 = time.time()
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]

        # Multi-scale face detection:
        # 1. Primary: face_recognition HOG with upsample=1
        boxes = face_recognition.face_locations(rgb, number_of_times_to_upsample=1, model="hog")

        # 2. Secondary fallback: dlib frontal face detector
        if not boxes:
            dlib_rects = self.dlib_detector(rgb, 0)
            if dlib_rects:
                boxes = [(r.top(), r.right(), r.bottom(), r.left()) for r in dlib_rects]

        infer_ms = int((time.time() - t0) * 1000.0)

        if not boxes:
            return {
                "result": "no_face",
                "faceDetected": False,
                "faceQuality": "NO_FACE",
                "box": [0, 0, 0, 0],
                "inferenceMs": infer_ms,
                "name": "---",
                "confidence": 0.0,
                "matchId": -1,
            }

        # Select the largest face in view
        top, right, bottom, left = max(boxes, key=lambda b: (b[2] - b[0]) * (b[1] - b[3]))
        # Clamp to frame boundaries
        top = max(0, min(h - 1, top))
        bottom = max(0, min(h - 1, bottom))
        left = max(0, min(w - 1, left))
        right = max(0, min(w - 1, right))
        box = [int(left), int(top), int(right), int(bottom)]

        # Extract 128-d face embedding vector
        encs = face_recognition.face_encodings(rgb, [(top, right, bottom, left)])
        if not encs:
            return {
                "result": "aligning",
                "faceDetected": True,
                "faceQuality": "BLURRY",
                "box": box,
                "inferenceMs": infer_ms,
                "name": "Aligning",
                "confidence": 0.0,
                "matchId": -1,
            }

        with self.lock:
            if not self.known_encodings:
                return {
                    "result": "unknown",
                    "faceDetected": True,
                    "faceQuality": "READY",
                    "box": box,
                    "inferenceMs": infer_ms,
                    "name": "Unknown",
                    "confidence": 0.0,
                    "matchId": -1,
                    "unregistered": True,
                }

            dists = face_recognition.face_distance(self.known_encodings, encs[0])
            best_idx = int(np.argmin(dists))
            best_dist = float(dists[best_idx])
            sim = max(0.0, min(1.0, 1.0 - (best_dist / self.distance_thresh) * 0.5))

            if best_dist <= self.distance_thresh:
                return {
                    "result": "recognized",
                    "faceDetected": True,
                    "faceQuality": "READY",
                    "box": box,
                    "inferenceMs": infer_ms,
                    "name": self.known_names[best_idx],
                    "confidence": sim,
                    "matchId": best_idx,
                }
            else:
                return {
                    "result": "unknown",
                    "faceDetected": True,
                    "faceQuality": "READY",
                    "box": box,
                    "inferenceMs": infer_ms,
                    "name": "Unknown",
                    "confidence": sim,
                    "matchId": -1,
                }


class LaptopFaceBridge:
    """Keep the newest serial frame available to the detector and dashboard."""

    def __init__(self, client, engine: FastFaceEngine, hub, show_gui: bool = True):
        self.client = client
        self.engine = engine
        self.hub = hub
        self.show_gui = show_gui
        self.fps = 0.0
        self.frames_count = 0

        self._stop = threading.Event()
        self._latest_jpg: Optional[bytes] = None
        self._latest_frame_at = 0.0
        self._stream_generation = 0
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

    def _observe_result(self, verdict, frame_at, generation):
        with self._jpg_lock:
            if self._stream_generation != generation or not self._recent_frame():
                return
            self.hub.observe(verdict, frame_at)

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
                    self.hub.observe({}, time.monotonic())
                    self._jpg_lock.notify_all()
            self._stop.wait(retry_delay)

    def draw_hud(self, bgr_img: Optional[np.ndarray], verdict: dict, fps: float,
                 size: Tuple[int, int] = (960, 720)) -> np.ndarray:
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

            box = verdict.get("box")
            if verdict.get("faceDetected") and box is not None and len(box) == 4:
                x1, y1, x2, y2 = (
                    round(box[0] * video_w / source_w), round(box[1] * video_h / source_h),
                    round(box[2] * video_w / source_w), round(box[3] * video_h / source_h),
                )
                x1, x2 = max(0, x1), min(video_w - 1, x2)
                y1, y2 = max(0, y1), min(video_h - 1, y2)
                if x2 > x1 and y2 > y1:
                    result = verdict.get("result")
                    if result == "recognized":
                        color = teal
                        tag = f"{verdict.get('name', 'Recognized')}  {verdict.get('confidence', 0):.0%}"
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
                stats = (f"{source_w}x{source_h} source   |   Preview {fps:.1f} FPS"
                         f"   |   AI {verdict.get('inferenceMs', 0)} ms")
            else:
                stats = self.client.description
            write(stats, 18, height - footer + 23, 0.52, text, width - 36)
            if footer >= 48 and width >= 210:
                write("E  Enroll     R  Reload faces     Q  Quit", 18, height - 15, 0.46, muted, width - 36)
        return canvas

    def run(self):
        print("\n" + "=" * 70)
        print(f" [SMART HOME] Laptop Face Recognition AI Online")
        print(f" - Camera Stream: {self.client.description}")
        print(f" - Feedback API: {self.hub.feedback_url}")
        print(f" - Enrolled Faces: {len(self.engine.known_names)} loaded from {self.engine.faces_dir}")
        print(f" - Hotkeys: Press 'e' in preview window to enroll, 'q' to quit")
        print("=" * 70 + "\n")

        fps_timer = time.time()
        fps_frame_counter = 0
        window_name = "Smart Home Face Recognition - Camera 1"
        preview_size = (960, 720)
        painted_size = preview_size
        painted_frame = None
        current_bgr = None
        verdict = {}

        def paint_preview(frame, overlay):
            nonlocal preview_size, painted_size, painted_frame
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) == 0:
                    return False
                _, _, width, height = cv2.getWindowImageRect(window_name)
                if width > 0 and height > 0:
                    preview_size = width, height
            except cv2.error:
                # Some HighGUI backends do not expose the drawable size.
                pass
            if frame is not painted_frame or preview_size != painted_size:
                cv2.imshow(window_name, self.draw_hud(frame, overlay, self.fps, preview_size))
                painted_frame, painted_size = frame, preview_size
            return True

        if self.show_gui:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.resizeWindow(window_name, *preview_size)
            cv2.imshow(window_name, self.draw_hud(None, {}, 0.0, preview_size))

        last_frame_number = 0
        preview_server = None
        preview_worker = None
        workers = []
        try:
            preview_server = make_preview_server(self.hub.preview_bind, self.hub.preview_port,
                                                 self._preview_frame)
            self.hub.preview_port = preview_server.server_port
            print(f"[LAPTOP-HUB] Dashboard preview listening on TCP port {self.hub.preview_port}.", flush=True)
            preview_worker = threading.Thread(target=preview_server.serve_forever, daemon=True)
            workers.append(preview_worker)
            workers.append(threading.Thread(target=self.hub.run_feedback,
                                            args=(self._stream_state, self._stop), daemon=True))
            workers.append(threading.Thread(target=self._stream_reader_loop, daemon=True))
            for worker in workers:
                worker.start()

            while not self._stop.is_set():
                with self._jpg_lock:
                    jpg_bytes = self._latest_jpg
                    frame_number = self.frames_count
                    frame_at = self._latest_frame_at
                    generation = self._stream_generation

                camera_live = jpg_bytes is not None and time.monotonic() - frame_at <= 1.0
                if not camera_live or frame_number == last_frame_number:
                    if self.show_gui:
                        if not paint_preview(current_bgr if camera_live else None, verdict):
                            break
                        if (cv2.waitKey(1) & 0xFF) in (27, ord('q'), ord('Q')):
                            break
                    self._stop.wait(0.02)
                    continue

                last_frame_number = frame_number
                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                current_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if current_bgr is None:
                    continue

                # Run multi-scale AI face recognition
                verdict = self.engine.analyze_frame(current_bgr)

                now = time.time()
                fps_frame_counter += 1
                if now - fps_timer >= 1.0:
                    self.fps = fps_frame_counter / (now - fps_timer)
                    fps_frame_counter = 0
                    fps_timer = now

                self._observe_result(verdict, frame_at, generation)

                # Display live GUI
                if self.show_gui:
                    if not paint_preview(current_bgr, verdict):
                        break

                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q'), ord('Q')):
                        break
                    elif key in (ord('e'), ord('E')):
                        # Instant 1-Key Enrollment
                        if verdict.get("faceDetected") and verdict.get("box") != [0, 0, 0, 0]:
                            box = verdict.get("box")
                            enroll_name = input("\n[ENROLL] Enter name for face in view: ").strip()
                            if not enroll_name:
                                enroll_name = f"User_{len(self.engine.known_names) + 1}"
                            ok = self.engine.enroll(current_bgr, box, enroll_name)
                            if ok:
                                print(f"[ENROLL] Success! {enroll_name} is now authorized.\n")
                        else:
                            print("[ENROLL] Please look directly into the camera so a face is detected first.")
                    elif key in (ord('r'), ord('R')):
                        self.engine.load_known_faces()

                time.sleep(0.005)

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            if preview_server is not None:
                if preview_worker.is_alive():
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
    parser.add_argument("--thresh", type=float, default=0.58,
                        help="Euclidean face distance threshold (default: 0.58, lower=stricter)")
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
    engine = FastFaceEngine(args.faces_dir, distance_thresh=args.thresh)
    bridge = LaptopFaceBridge(client, engine, hub, show_gui=not args.no_gui)
    try:
        bridge.run()
    except OSError as exc:
        parser.exit(1, f"Laptop face bridge failed: {exc}\n")


if __name__ == "__main__":
    main()
