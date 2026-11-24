#!/usr/bin/env python3
"""
Laptop Face Recognition Bridge for Smart Home Camera 1 (cam_face_stream firmware).

Connects to the ESP32-CAM MJPEG stream (:81/stream), runs high-speed real-time face
detection and recognition on the laptop CPU/GPU, displays a live GUI HUD preview,
and sends recognition results back to the ESP32-CAM (POST /api/recognition/result).
Standalone mode displays the results locally. Hub mode can forward access events.

Features:
- Instant multi-scale face detection & MobileFaceNet/dlib embedding matching (<25 ms).
- Live OpenCV GUI window with green (Recognized), red (Unknown), and yellow (New face) reticles.
- 1-Key instant face enrollment (press 'e' in preview window to enroll on the spot).
- Persistent face storage in `tools/known_faces/` across restarts.
- Auto-sync with ESP32 Web Dashboard enrolled user lists.
"""

import argparse
import json
import os
import sys
import threading
import time
import urllib.request
from typing import Dict, List, Optional, Tuple

import cv2
import dlib
import face_recognition
import numpy as np

if __package__:
    from .camera_stream import IncompleteJpegPart, RESOLUTIONS, iter_jpegs, set_video_config
else:
    from camera_stream import IncompleteJpegPart, RESOLUTIONS, iter_jpegs, set_video_config


class EspCamStreamClient:
    """HTTP control & MJPEG stream client for the cam_face_stream ESP32-CAM node."""

    def __init__(self, cam_ip: Optional[str] = None, http_port: int = 80,
                 stream_port: int = 81, *, standalone: bool = False):
        cam_ip = cam_ip or ("192.168.11.1" if standalone else "192.168.4.101")
        self.cam_ip = cam_ip
        self.standalone = standalone or cam_ip == "192.168.11.1"
        self.base_url = f"http://{cam_ip}:{http_port}"
        self.stream_url = f"http://{cam_ip}:{stream_port}/stream"
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _post_json(self, path: str, payload: Optional[dict] = None, timeout: float = 1.5) -> dict:
        data = json.dumps(payload or {}).encode("utf-8")
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self._opener.open(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def get_status(self, timeout: float = 1.5) -> dict:
        with self._opener.open(f"{self.base_url}/api/status", timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def send_result(
        self,
        result: str,
        match_id: int = -1,
        name: str = "---",
        confidence: float = 0.0,
        face_detected: bool = False,
        face_quality: str = "NO_FACE",
        box: Optional[List[int]] = None,
        inference_ms: int = 0,
        laptop_model: str = "Laptop-AI",
        trigger_hub: bool = True,
        force: bool = False,
    ) -> dict:
        payload = {
            "result": result,
            "matchId": int(match_id),
            "name": name,
            "confidence": float(confidence),
            "faceDetected": bool(face_detected),
            "faceQuality": face_quality,
            "box": box or [0, 0, 0, 0],
            "inferenceMs": int(inference_ms),
            "laptopModel": laptop_model,
            "triggerHub": bool(trigger_hub and not self.standalone),
            "force": bool(force),
        }
        return self._post_json("/api/recognition/result", payload)

    def sync_enrolled(self, enrolled: List[Dict[str, object]]) -> dict:
        return self._post_json("/api/laptop/enrolled", {"enrolled": enrolled})

    def ack_enroll(self, ok: bool, face_id: int, name: str, msg: str = "") -> dict:
        return self._post_json(
            "/api/enroll/ack",
            {"ok": bool(ok), "id": int(face_id), "name": name, "msg": msg,
             "triggerHub": not self.standalone},
        )

    def ack_command(self, seq: int) -> dict:
        return self._post_json("/api/command/ack", {"seq": int(seq)})

    def iter_mjpeg_frames(self, timeout: float = 5.0):
        """Yield raw JPEG byte buffers from the ESP32-CAM multipart MJPEG stream."""
        req = urllib.request.Request(self.stream_url)
        with self._opener.open(req, timeout=timeout) as resp:
            yield from iter_jpegs(resp, skip_damaged=True)

    def configure_video(self, resolution=None, quality=None):
        set_video_config(self._opener, self.base_url, resolution, quality)


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
    """Manages stream reader, AI analyzer, live OpenCV window, and ESP-CAM feedback API."""

    def __init__(
        self,
        client: EspCamStreamClient,
        engine: FastFaceEngine,
        show_gui: bool = True,
        feedback_interval_s: float = 0.35,
        laptop_model: str = "Laptop-AI",
    ):
        self.client = client
        self.engine = engine
        self.show_gui = show_gui
        self.feedback_interval_s = feedback_interval_s
        self.laptop_model = laptop_model

        self.last_status: dict = {}
        self.last_feedback_time = 0.0
        self.last_result_sent = "NONE"
        self.fps = 0.0
        self.frames_count = 0

        self._stop = threading.Event()
        self._latest_jpg: Optional[bytes] = None
        self._latest_frame_at = 0.0
        self._jpg_lock = threading.Lock()

    def stop(self):
        self._stop.set()

    def _stream_reader_loop(self):
        while not self._stop.is_set():
            retry_delay = 1.0
            try:
                for jpg in self.client.iter_mjpeg_frames():
                    if self._stop.is_set():
                        break
                    with self._jpg_lock:
                        self._latest_jpg = jpg
                        self._latest_frame_at = time.monotonic()
                        self.frames_count += 1
                    retry_delay = 0.2
            except IncompleteJpegPart:
                retry_delay = 0.2
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"[LAPTOP-AI] Stream reconnecting ({exc})...", file=sys.stderr)
            finally:
                with self._jpg_lock:
                    self._latest_jpg = None
            self._stop.wait(retry_delay)

    def _status_sync_loop(self):
        """Periodically sync enrolled names and ESP-CAM commands in the background."""
        while not self._stop.is_set():
            try:
                status = self.client.get_status()
                self.last_status = status

                # Sync enrolled list with ESP32 Web UI
                enrolled_list = [
                    {"id": i, "name": name}
                    for i, name in enumerate(self.engine.known_names)
                ]
                self.client.sync_enrolled(enrolled_list)

                # Handle remote dashboard delete/clear commands
                if status.get("pendingAction") == "delete":
                    seq = int(status.get("pendingActionSeq", 0))
                    del_id = int(status.get("pendingDeleteId", -1))
                    if 0 <= del_id < len(self.engine.known_names):
                        del_name = self.engine.known_names[del_id]
                        fpath = os.path.join(self.engine.faces_dir, f"{del_name.replace(' ', '_')}.jpg")
                        if os.path.isfile(fpath):
                            os.remove(fpath)
                        self.engine.load_known_faces()
                    self.client.ack_command(seq)
                elif status.get("pendingAction") == "clear":
                    seq = int(status.get("pendingActionSeq", 0))
                    for fname in os.listdir(self.engine.faces_dir):
                        p = os.path.join(self.engine.faces_dir, fname)
                        if os.path.isfile(p):
                            os.remove(p)
                    self.engine.load_known_faces()
                    self.client.ack_command(seq)

            except Exception:
                pass
            time.sleep(1.5)

    def step_once(self) -> Optional[dict]:
        """Analyze the latest frame once and return the camera response.

        This synchronous entry point is useful for integration tests and for
        headless callers. The production loop below uses the same feedback API
        and keeps its reconnecting reader thread.
        """
        with self._jpg_lock:
            jpg_bytes = self._latest_jpg
        if jpg_bytes is None:
            return None

        # Keep the synchronous path equivalent to the background sync thread.
        # This also makes dashboard commands deterministic in integration tests.
        try:
            self.last_status = self.client.get_status()
        except Exception:
            pass

        if callable(self.engine) and not hasattr(self.engine, "analyze_frame"):
            verdict = self.engine(jpg_bytes, self.last_status)
        else:
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return None
            verdict = self.engine.analyze_frame(frame)

        if verdict.get("enrollCaptured") and self.last_status.get("enrollPending"):
            target_name = str(self.last_status.get("enrollTargetName") or "User")
            enrolled = self.last_status.get("enrolled") or []
            new_id = max((int(item.get("id", -1)) for item in enrolled), default=-1) + 1
            return self.client.ack_enroll(True, new_id, target_name, "Enrolled via Laptop AI")

        pending_action = self.last_status.get("pendingAction")
        if pending_action in ("delete", "clear"):
            self.client.ack_command(int(self.last_status.get("pendingActionSeq", 0)))
            self.last_status = self.client.get_status()

        result = verdict.get("result", "no_face")
        response = self.client.send_result(
            result=result,
            match_id=verdict.get("matchId", -1),
            name=verdict.get("name", "---"),
            confidence=verdict.get("confidence", 0.0),
            face_detected=verdict.get("faceDetected", False),
            face_quality=verdict.get("faceQuality", "NO_FACE"),
            box=verdict.get("box", [0, 0, 0, 0]),
            inference_ms=verdict.get("inferenceMs", 0),
            laptop_model=self.laptop_model,
        )
        self.last_feedback_time = time.time()
        self.last_result_sent = result
        return response

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
            stats = self.client.cam_ip
            if live:
                stats = (f"{source_w}x{source_h} source   |   Preview {fps:.1f} FPS"
                         f"   |   AI {verdict.get('inferenceMs', 0)} ms")
            write(stats, 18, height - footer + 23, 0.52, text, width - 36)
            if footer >= 48 and width >= 210:
                write("E  Enroll     R  Reload faces     Q  Quit", 18, height - 15, 0.46, muted, width - 36)
        return canvas

    def run(self):
        reader = threading.Thread(target=self._stream_reader_loop, daemon=True)
        reader.start()

        sync_thread = threading.Thread(target=self._status_sync_loop, daemon=True)
        sync_thread.start()

        print("\n" + "=" * 70)
        print(f" [SMART HOME] Laptop Face Recognition AI Online")
        print(f" - Camera Stream: {self.client.stream_url}")
        print(f" - Feedback API:  {self.client.base_url}/api/recognition/result")
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

        try:
            while not self._stop.is_set():
                with self._jpg_lock:
                    jpg_bytes = self._latest_jpg
                    frame_number = self.frames_count
                    frame_at = self._latest_frame_at

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

                # Dispatch result back to ESP32-CAM (throttled to avoid HTTP flooding)
                result_str = verdict.get("result", "no_face")
                should_send = (
                    (result_str in ("recognized", "unknown") and (now - self.last_feedback_time >= self.feedback_interval_s)) or
                    (result_str != self.last_result_sent and result_str != "no_face") or
                    (now - self.last_feedback_time >= 2.0)
                )

                if should_send:
                    try:
                        response = self.client.send_result(
                            result=result_str,
                            match_id=verdict.get("matchId", -1),
                            name=verdict.get("name", "---"),
                            confidence=verdict.get("confidence", 0.0),
                            face_detected=verdict.get("faceDetected", False),
                            face_quality=verdict.get("faceQuality", "NO_FACE"),
                            box=verdict.get("box", [0, 0, 0, 0]),
                            inference_ms=verdict.get("inferenceMs", 0),
                            laptop_model=self.laptop_model,
                        )
                        if response.get("hubDispatched"):
                            print(f"[AI-MATCH] Sent {response.get('hubAction')} to the hub")
                        elif result_str != self.last_result_sent and result_str in ("recognized", "unknown"):
                            print(f"[AI-MATCH] {result_str.upper()}: {verdict.get('name', 'Unknown')}")
                        self.last_result_sent = result_str
                    except (OSError, ValueError) as exc:
                        print(f"[LAPTOP-AI] Camera feedback failed: {exc}", file=sys.stderr)
                    finally:
                        self.last_feedback_time = now

                # Check if dashboard triggered an enrollment
                if self.last_status.get("enrollPending") and verdict.get("faceDetected"):
                    box = verdict.get("box", [0, 0, 0, 0])
                    target_name = str(self.last_status.get("enrollTargetName") or "User")
                    if self.engine.enroll(current_bgr, box, target_name):
                        new_id = len(self.engine.known_names) - 1
                        try:
                            self.client.ack_enroll(True, new_id, target_name, "Enrolled via Laptop AI")
                        except Exception:
                            pass

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
            if self.show_gui:
                cv2.destroyAllWindows()
            print("\n[LAPTOP-AI] Bridge stopped cleanly.")


def main():
    parser = argparse.ArgumentParser(description="High-Speed Laptop Face Recognition Bridge for ESP32-CAM")
    parser.add_argument("--standalone", action="store_true",
                        help="Use the camera hotspot at 192.168.11.1 and keep results local")
    parser.add_argument("--ip", help="Override the camera IP (hub mode defaults to 192.168.4.101)")
    parser.add_argument("--http-port", type=int, default=80, help="ESP32-CAM HTTP control port (default: 80)")
    parser.add_argument("--stream-port", type=int, default=81, help="ESP32-CAM MJPEG stream port (default: 81)")
    parser.add_argument("--resolution", choices=RESOLUTIONS, help="Set the camera resolution before streaming")
    parser.add_argument("--jpeg-quality", type=int, metavar="8–30",
                        help="Set JPEG compression. Higher numbers send smaller frames")
    parser.add_argument("--faces-dir", default=os.path.join(os.path.dirname(__file__), "known_faces"),
                        help="Directory containing <Name>.jpg photos of authorized persons")
    parser.add_argument("--thresh", type=float, default=0.58,
                        help="Euclidean face distance threshold (default: 0.58, lower=stricter)")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless background mode without OpenCV window")
    args = parser.parse_args()
    if args.jpeg_quality is not None and not 8 <= args.jpeg_quality <= 30:
        parser.error("--jpeg-quality must be between 8 and 30")

    client = EspCamStreamClient(args.ip, args.http_port, args.stream_port, standalone=args.standalone)
    try:
        client.configure_video(args.resolution, args.jpeg_quality)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Camera configuration failed: {exc}\n")
    engine = FastFaceEngine(args.faces_dir, distance_thresh=args.thresh)
    bridge = LaptopFaceBridge(client, engine, show_gui=not args.no_gui)

    bridge.run()


if __name__ == "__main__":
    main()
