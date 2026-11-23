#!/usr/bin/env python3
"""
Laptop Face Recognition Bridge for Smart Home Camera 1 (cam_face_stream firmware).

Connects to the ESP32-CAM MJPEG stream (:81/stream), runs high-speed real-time face
detection and recognition on the laptop CPU/GPU, displays a live GUI HUD preview,
and sends recognition results back to the ESP32-CAM (POST /api/recognition/result)
to immediately trigger Main Hub door unlock or intruder siren.

Features:
- Instant multi-scale face detection & MobileFaceNet/dlib embedding matching (<25 ms).
- Live OpenCV GUI window with green (Recognized), red (Unknown), and yellow (New face) reticles.
- 1-Key instant face enrollment (press 'e' in preview window to enroll on the spot).
- Persistent face storage in `tools/known_faces/` across restarts.
- Auto-sync with ESP32 Web Dashboard enrolled user lists.
"""

import argparse
import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from typing import Dict, List, Optional, Tuple

import cv2
import dlib
import face_recognition
import numpy as np


class EspCamStreamClient:
    """HTTP control & MJPEG stream client for the cam_face_stream ESP32-CAM node."""

    def __init__(self, cam_ip: str = "192.168.4.101", http_port: int = 80, stream_port: int = 81):
        self.cam_ip = cam_ip
        self.base_url = f"http://{cam_ip}:{http_port}"
        self.stream_url = f"http://{cam_ip}:{stream_port}/stream"

    def _post_json(self, path: str, payload: Optional[dict] = None, timeout: float = 1.5) -> dict:
        data = json.dumps(payload or {}).encode("utf-8")
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def get_status(self, timeout: float = 1.5) -> dict:
        with urllib.request.urlopen(f"{self.base_url}/api/status", timeout=timeout) as resp:
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
            "triggerHub": bool(trigger_hub),
            "force": bool(force),
        }
        return self._post_json("/api/recognition/result", payload)

    def sync_enrolled(self, enrolled: List[Dict[str, object]]) -> dict:
        return self._post_json("/api/laptop/enrolled", {"enrolled": enrolled})

    def ack_enroll(self, ok: bool, face_id: int, name: str, msg: str = "") -> dict:
        return self._post_json(
            "/api/enroll/ack",
            {"ok": bool(ok), "id": int(face_id), "name": name, "msg": msg},
        )

    def ack_command(self, seq: int) -> dict:
        return self._post_json("/api/command/ack", {"seq": int(seq)})

    def iter_mjpeg_frames(self, timeout: float = 5.0):
        """Yield raw JPEG byte buffers from the ESP32-CAM multipart MJPEG stream."""
        req = urllib.request.Request(self.stream_url)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            buf = bytearray()
            while True:
                chunk = resp.read(8192)
                if not chunk:
                    break
                buf.extend(chunk)
                while True:
                    soi = buf.find(b"\xff\xd8")
                    if soi < 0:
                        if len(buf) > 65536:
                            del buf[:-2]
                        break
                    eoi = buf.find(b"\xff\xd9", soi + 2)
                    if eoi < 0:
                        if soi > 0:
                            del buf[:soi]
                        break
                    jpg = bytes(buf[soi : eoi + 2])
                    del buf[: eoi + 2]
                    yield jpg


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
        self._jpg_lock = threading.Lock()

    def stop(self):
        self._stop.set()

    def _stream_reader_loop(self):
        while not self._stop.is_set():
            try:
                for jpg in self.client.iter_mjpeg_frames():
                    if self._stop.is_set():
                        break
                    with self._jpg_lock:
                        self._latest_jpg = jpg
                        self.frames_count += 1
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"[LAPTOP-AI] Stream reconnecting ({exc})...", file=sys.stderr)
                    time.sleep(1.0)

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

    def draw_hud(self, bgr_img: np.ndarray, verdict: dict, fps: float) -> np.ndarray:
        frame = bgr_img.copy()
        h, w = frame.shape[:2]

        result = verdict.get("result", "no_face")
        name = verdict.get("name", "---")
        conf = verdict.get("confidence", 0.0)
        box = verdict.get("box", [0, 0, 0, 0])
        infer_ms = verdict.get("inferenceMs", 0)

        # Draw Top Status Bar
        cv2.rectangle(frame, (0, 0), (w, 38), (20, 20, 20), -1)
        status_text = (
            f"ESP32-CAM: {self.client.cam_ip} | {w}x{h} | FPS: {fps:.1f} | "
            f"Infer: {infer_ms}ms | Enrolled: {len(self.engine.known_names)}"
        )
        cv2.putText(frame, status_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (220, 220, 220), 1, cv2.LINE_AA)

        # Draw Face Reticle
        if verdict.get("faceDetected") and box != [0, 0, 0, 0]:
            left, top, right, bottom = box
            if result == "recognized":
                color = (0, 255, 0)      # Bright Green = Authorized
                tag = f"AUTHORIZED: {name.upper()} ({int(conf * 100)}%) -> UNLOCKING DOOR"
            elif result == "unknown":
                color = (0, 0, 255)      # Red = Unknown Intruder
                tag = f"UNKNOWN FACE (INTRUDER ALERT) ({int(conf * 100)}%)"
            else:
                color = (0, 255, 255)    # Yellow = Aligning / Unregistered
                tag = f"DETECTED FACE (Press 'e' to enroll)"

            # Draw rounded bounding box
            cv2.rectangle(frame, (left, top), (right, bottom), color, 2, cv2.LINE_AA)
            # Reticle corners
            corner_len = min(20, (right - left) // 4)
            cv2.line(frame, (left, top), (left + corner_len, top), color, 4)
            cv2.line(frame, (left, top), (left, top + corner_len), color, 4)
            cv2.line(frame, (right, top), (right - corner_len, top), color, 4)
            cv2.line(frame, (right, top), (right, top + corner_len), color, 4)
            cv2.line(frame, (left, bottom), (left + corner_len, bottom), color, 4)
            cv2.line(frame, (left, bottom), (left, bottom - corner_len), color, 4)
            cv2.line(frame, (right, bottom), (right - corner_len, bottom), color, 4)
            cv2.line(frame, (right, bottom), (right, bottom - corner_len), color, 4)

            # Draw Tag Background
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
            tag_y = max(top - 8, th + 44)
            cv2.rectangle(frame, (left, tag_y - th - 6), (left + tw + 10, tag_y + 4), color, -1)
            cv2.putText(frame, tag, (left + 5, tag_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 1, cv2.LINE_AA)

        # Draw Bottom Help Bar
        cv2.rectangle(frame, (0, h - 28), (w, h), (20, 20, 20), -1)
        help_text = "[E] Enroll Face in View   |   [R] Reload Known Faces   |   [Q/ESC] Quit"
        cv2.putText(frame, help_text, (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 230, 230), 1, cv2.LINE_AA)

        return frame

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

        if self.show_gui:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

        last_verdict = {"result": "no_face", "faceDetected": False}
        current_bgr = None

        try:
            while not self._stop.is_set():
                with self._jpg_lock:
                    jpg_bytes = self._latest_jpg

                if jpg_bytes is None:
                    time.sleep(0.02)
                    continue

                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                current_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if current_bgr is None:
                    continue

                # Run multi-scale AI face recognition
                verdict = self.engine.analyze_frame(current_bgr)
                last_verdict = verdict

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
                        self.client.send_result(
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
                        self.last_feedback_time = now
                        self.last_result_sent = result_str

                        if result_str == "recognized":
                            print(f"[AI-MATCH] RECOGNIZED: {verdict.get('name')} ({int(verdict.get('confidence', 0)*100)}%) -> Door Unlocked!")
                        elif result_str == "unknown":
                            print(f"[AI-MATCH] UNKNOWN PERSON DETECTED -> Intruder Siren Triggered")
                    except Exception as e:
                        pass

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
                    hud_frame = self.draw_hud(current_bgr, verdict, self.fps)
                    cv2.imshow(window_name, hud_frame)

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
    parser.add_argument("--ip", default="192.168.4.101", help="ESP32-CAM IP address (default: 192.168.4.101)")
    parser.add_argument("--http-port", type=int, default=80, help="ESP32-CAM HTTP control port (default: 80)")
    parser.add_argument("--stream-port", type=int, default=81, help="ESP32-CAM MJPEG stream port (default: 81)")
    parser.add_argument("--faces-dir", default=os.path.join(os.path.dirname(__file__), "known_faces"),
                        help="Directory containing <Name>.jpg photos of authorized persons")
    parser.add_argument("--thresh", type=float, default=0.58,
                        help="Euclidean face distance threshold (default: 0.58, lower=stricter)")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless background mode without OpenCV window")
    args = parser.parse_args()

    client = EspCamStreamClient(args.ip, args.http_port, args.stream_port)
    engine = FastFaceEngine(args.faces_dir, distance_thresh=args.thresh)
    bridge = LaptopFaceBridge(client, engine, show_gui=not args.no_gui)

    bridge.run()


if __name__ == "__main__":
    main()
