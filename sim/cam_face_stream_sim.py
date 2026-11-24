#!/usr/bin/env python3
"""
Virtual ESP32-CAM Node 1 (Digital Twin of src/cam_face_stream/main.cpp)

Simulates the temporary offloaded-AI camera firmware:
- Clean raw MJPEG stream on port 8082 (/stream) with NO baked-in bounding boxes
- Web Management Dashboard & Laptop AI Feedback API on port 8081
- Dual-Path Hub notifications (UDP simulated ESP-NOW + HTTP POST /api/cam/event)
"""

import argparse
import io
import json
import os
import socket
import struct
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
from PIL import Image, ImageDraw

HUB_UDP_PORT = 9000
HUB_HTTP_URL = "http://127.0.0.1:8080/api/cam/event"

PACKET_MAGIC = 0x53484331  # "SHC1"
CAM_SOURCE_FACE = 1
CAM_EVENT_RECOGNIZED = 1
CAM_EVENT_UNKNOWN = 2
CAM_EVENT_HEARTBEAT = 3

RECOGNIZED_COOLDOWN_S = 2.5
UNKNOWN_COOLDOWN_S = 4.0
ENROLL_TIMEOUT_S = 12.0
LAPTOP_LINK_TIMEOUT_S = 5.0


def load_firmware_dashboard_html(stream_port: int) -> bytes:
    """Load the exact Web Dashboard HTML from src/cam_face_stream/main.cpp."""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cpp_path = os.path.join(repo_root, "src", "cam_face_stream", "main.cpp")
    with open(cpp_path, "r", encoding="utf-8") as f:
        src = f.read()
    start_marker = 'R"HTML('
    end_marker = ')HTML"'
    s_idx = src.find(start_marker)
    e_idx = src.find(end_marker, s_idx + len(start_marker))
    if s_idx < 0 or e_idx < 0:
        return b"<html><body>Dashboard missing</body></html>"
    html = src[s_idx + len(start_marker) : e_idx]
    if stream_port != 81:
        html = html.replace("':81/stream'", f"':{stream_port}/stream'")
        html = html.replace("STREAMING :81", f"STREAMING :{stream_port}")
    return html.encode("utf-8")


class StreamCamState:
    def __init__(self, stream_port: int = 8082):
        self.lock = threading.Lock()
        self.stream_port = stream_port
        self.camera_ready = True
        self.laptop_connected = False
        self.last_laptop_ms = 0.0
        self.laptop_model = "Laptop AI"
        self.face_detected = False
        self.motion_detected = False
        self.face_quality = "NO_FACE"
        self.face_box = [0, 0, 0, 0]
        self.frame_width = 640
        self.frame_height = 480
        self.enroll_pending = False
        self.enroll_start_ms = 0.0
        self.enroll_target_name = ""
        self.enroll_message = "Idle: Ready for Laptop enrollment"
        self.pending_action = "none"
        self.pending_action_seq = 0
        self.pending_delete_id = -1
        self.last_result = "NONE"
        self.last_match_name = "---"
        self.last_match_id = -1
        self.last_similarity = 0.0
        self.inference_ms = 0
        self.fps = 24.0
        self.frame_count = 0
        self.scene_luma = 0.45
        self.flash_duty_applied = 0
        self.feedback_count = 0
        self.enrolled = []
        self.cfg = {
            "armed": True,
            "unknownConfirm": 2,
            "flashDuty": 0,
            "flashAuto": True,
            "hmirror": False,
            "vflip": False,
            "frameRes": 2,
            "jpegQuality": 12,
        }
        self.unknown_streak = 0
        self.last_recognized_sent = 0.0
        self.last_unknown_sent = 0.0
        self.seq = 0
        self.latest_jpeg = b""
        self.new_frame_event = threading.Event()
        self.dashboard_html = load_firmware_dashboard_html(stream_port)

    def send_dual_path_event(self, event_type: int, match_id: int, conf: int, label: str) -> bool:
        self.seq += 1
        seq = self.seq
        udp_ok = False
        http_ok = False

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            raw_label = label.encode("utf-8")[:23].ljust(24, b"\x00")
            pkt = struct.pack(
                "<IBBhBI24s",
                PACKET_MAGIC,
                CAM_SOURCE_FACE,
                event_type,
                match_id,
                min(max(int(conf), 0), 100),
                seq,
                raw_label,
            )
            sock.sendto(pkt, ("127.0.0.1", HUB_UDP_PORT))
            sock.close()
            udp_ok = True
        except Exception as exc:
            print(f"[STREAM-CAM-SIM] UDP send error: {exc}")

        if event_type != CAM_EVENT_HEARTBEAT:
            try:
                payload = json.dumps(
                    {
                        "source": CAM_SOURCE_FACE,
                        "event": event_type,
                        "matchId": match_id,
                        "confidence": min(max(int(conf), 0), 100),
                        "seq": seq,
                        "label": label,
                    }
                ).encode("utf-8")
                req = urllib.request.Request(
                    HUB_HTTP_URL,
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=0.6) as resp:
                    http_ok = resp.status == 200
            except Exception as exc:
                print(f"[STREAM-CAM-SIM] HTTP send error: {exc}")

        return udp_ok or http_ok


def frame_generator_loop(cam: StreamCamState, stop_event: threading.Event):
    last_heartbeat = 0.0
    while not stop_event.is_set():
        now = time.time()
        with cam.lock:
            w, h = cam.frame_width, cam.frame_height
            cam.frame_count += 1
            fnum = cam.frame_count

        # Render a clean raw synthetic frame (NO baked bounding boxes or text over the face)
        img = Image.new("RGB", (w, h), color=(18, 28, 46))
        draw = ImageDraw.Draw(img)
        cx = w // 2 + int(8 * ((fnum % 20) - 10) / 10.0)
        cy = h // 2
        r = min(w, h) // 5
        # Clean synthetic face silhouette for testing Laptop analyzers
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(224, 184, 152), outline=(190, 145, 115))
        draw.ellipse([cx - r // 2, cy - r // 3, cx - r // 4, cy - r // 8], fill=(35, 35, 45))
        draw.ellipse([cx + r // 4, cy - r // 3, cx + r // 2, cy - r // 8], fill=(35, 35, 45))
        draw.arc([cx - r // 3, cy + r // 8, cx + r // 3, cy + r // 2], start=15, end=165, fill=(135, 70, 65), width=3)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        jpg = buf.getvalue()

        with cam.lock:
            cam.latest_jpeg = jpg
        cam.new_frame_event.set()

        if now - last_heartbeat >= 1.5:
            last_heartbeat = now
            cam.send_dual_path_event(CAM_EVENT_HEARTBEAT, 0, 100, "FACE-CAM")

        stop_event.wait(0.05)  # ~20 FPS clean hardware-like stream


def make_http_handler(cam: StreamCamState):
    class CamHttpHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_json(self, code: int, obj: dict):
            raw = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _read_body_and_params(self):
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            length = int(self.headers.get("Content-Length", "0") or 0)
            body_bytes = self.rfile.read(length) if length > 0 else b""
            body_json = {}
            if body_bytes:
                try:
                    body_json = json.loads(body_bytes.decode("utf-8"))
                except Exception:
                    body_json = {}
            return parsed.path, qs, body_json

        def _param(self, qs, body_json, key, default=None):
            if key in qs and len(qs[key]) > 0:
                return qs[key][0]
            if isinstance(body_json, dict) and key in body_json:
                return body_json[key]
            return default

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)

            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(cam.dashboard_html)))
                self.end_headers()
                self.wfile.write(cam.dashboard_html)
                return

            if path == "/preview.jpg":
                with cam.lock:
                    jpg = cam.latest_jpeg
                if not jpg:
                    self.send_error(503, "Preview not ready")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                return

            if path == "/stream":
                host = self.headers.get("Host", "127.0.0.1").split(":")[0]
                self.send_response(302)
                self.send_header("Location", f"http://{host}:{cam.stream_port}/stream")
                self.end_headers()
                return

            if path == "/api/status":
                now = time.time()
                with cam.lock:
                    if cam.last_laptop_ms > 0 and (now - cam.last_laptop_ms <= LAPTOP_LINK_TIMEOUT_S):
                        cam.laptop_connected = True
                        cam.laptop_age_ms = int((now - cam.last_laptop_ms) * 1000)
                    else:
                        if cam.laptop_connected:
                            cam.laptop_connected = False
                            cam.face_detected = False
                            cam.face_box = [0, 0, 0, 0]
                            cam.face_quality = "NO_FACE"
                            cam.last_result = "NONE"
                            cam.last_match_name = "---"
                            cam.last_match_id = -1
                            cam.last_similarity = 0.0
                    if cam.enroll_pending:
                        elapsed = now - cam.enroll_start_ms
                        if elapsed >= ENROLL_TIMEOUT_S:
                            cam.enroll_pending = False
                            rem = 0
                            cam.enroll_message = "Enroll timed out: wait for Laptop confirmation"
                        else:
                            rem = int((ENROLL_TIMEOUT_S - elapsed) * 1000)
                    else:
                        rem = 0

                    payload = {
                        "mode": "REMOTE_LAPTOP_AI",
                        "cameraReady": cam.camera_ready,
                        "modelReady": cam.laptop_connected,
                        "flashOk": True,
                        "laptopConnected": cam.laptop_connected,
                        "laptopAgeMs": int((now - cam.last_laptop_ms) * 1000) if cam.last_laptop_ms else 0,
                        "laptopModel": cam.laptop_model,
                        "feedbackCount": cam.feedback_count,
                        "hubConnected": True,
                        "fallbackAp": False,
                        "ip": "127.0.0.1",
                        "streamPort": cam.stream_port,
                        "frameWidth": cam.frame_width,
                        "frameHeight": cam.frame_height,
                        "faceDetected": cam.face_detected,
                        "motionDetected": cam.motion_detected,
                        "faceQuality": cam.face_quality,
                        "faceBox": list(cam.face_box),
                        "enrollPending": cam.enroll_pending,
                        "enrollRemainingMs": rem,
                        "enrollTargetName": cam.enroll_target_name,
                        "enrollMessage": cam.enroll_message,
                        "pendingAction": cam.pending_action,
                        "pendingActionSeq": cam.pending_action_seq,
                        "pendingDeleteId": cam.pending_delete_id,
                        "lastResult": cam.last_result,
                        "lastMatchName": cam.last_match_name,
                        "lastMatchId": cam.last_match_id,
                        "lastSimilarity": round(cam.last_similarity, 3),
                        "inferenceMs": cam.inference_ms,
                        "fps": round(cam.fps, 1),
                        "frameCount": cam.frame_count,
                        "sceneLuma": round(cam.scene_luma, 3),
                        "flashDutyApplied": cam.flash_duty_applied,
                        "enrolledCount": len(cam.enrolled),
                        "enrolled": list(cam.enrolled),
                        "cfg": dict(cam.cfg),
                    }
                self._send_json(200, payload)
                return

            if path == "/api/enroll/cancel":
                with cam.lock:
                    cam.enroll_pending = False
                    cam.enroll_message = "Enrollment cancelled"
                self._send_json(200, {"cancelled": True})
                return

            self.send_error(404)

        def do_POST(self):
            path, qs, body = self._read_body_and_params()
            now = time.time()

            if path in ("/api/recognition/result", "/api/feedback"):
                raw_res = str(self._param(qs, body, "result", "")).upper()
                ev_code = self._param(qs, body, "event", None)
                if not raw_res and ev_code is not None:
                    raw_res = "RECOGNIZED" if int(ev_code) == 1 else ("UNKNOWN" if int(ev_code) == 2 else "NO_FACE")
                if not raw_res:
                    raw_res = "NO_FACE"

                match_id = self._param(qs, body, "matchId", self._param(qs, body, "id", -1))
                match_id = int(match_id) if match_id is not None else -1
                name = str(self._param(qs, body, "name", self._param(qs, body, "label", "---")))

                raw_conf = float(self._param(qs, body, "confidence", self._param(qs, body, "sim", 0.0)))
                sim = min(max(raw_conf / 100.0 if raw_conf > 1.0 else raw_conf, 0.0), 1.0)
                conf_pct = int(round(sim * 100.0))

                is_rec = raw_res in ("RECOGNIZED", "MATCH", "ENROLLED")
                is_unk = raw_res in ("UNKNOWN", "INTRUDER")
                is_align = raw_res in ("ALIGNING", "VERIFYING", "DETECTED")

                face_det_raw = self._param(qs, body, "faceDetected", is_rec or is_unk or is_align)
                face_det = face_det_raw in (True, 1, "1", "true", "True", "yes")
                face_qual = str(self._param(qs, body, "faceQuality", "READY" if face_det else "NO_FACE")).upper()

                box_raw = self._param(qs, body, "box", None)
                box = None
                if isinstance(box_raw, list) and len(box_raw) == 4:
                    box = [int(v) for v in box_raw]
                elif isinstance(box_raw, str):
                    parts = [p.strip() for p in box_raw.strip("[]").split(",") if p.strip()]
                    if len(parts) == 4:
                        box = [int(p) for p in parts]

                infer_ms = max(0, int(self._param(qs, body, "inferenceMs", 0)))
                model_name = self._param(qs, body, "laptopModel", None)
                trigger_hub = self._param(qs, body, "triggerHub", True) in (True, 1, "1", "true", "True")
                force = self._param(qs, body, "force", False) in (True, 1, "1", "true", "True")

                hub_dispatched = False
                hub_action = "none"
                display_res = "NO FACE"
                send_event_args = None

                with cam.lock:
                    cam.last_laptop_ms = now
                    cam.laptop_connected = True
                    if model_name:
                        cam.laptop_model = str(model_name)[:23]

                    if is_rec:
                        cam.unknown_streak = 0
                        if match_id < 0:
                            match_id = 0
                        if name == "---":
                            name = f"ID-{match_id}"
                        display_res = "RECOGNIZED"
                        if trigger_hub and (force or (now - cam.last_recognized_sent >= RECOGNIZED_COOLDOWN_S)):
                            cam.last_recognized_sent = now
                            send_event_args = (CAM_EVENT_RECOGNIZED, match_id, conf_pct, name)
                            hub_action = "DOOR_OPEN"
                        elif trigger_hub:
                            hub_action = "COOLDOWN"
                    elif is_unk:
                        match_id = -1
                        if name == "---":
                            name = "Unknown"
                        cam.unknown_streak += 1
                        if cam.unknown_streak >= cam.cfg["unknownConfirm"] or force:
                            display_res = "UNKNOWN"
                            if trigger_hub and cam.cfg["armed"] and (force or (now - cam.last_unknown_sent >= UNKNOWN_COOLDOWN_S)):
                                cam.last_unknown_sent = now
                                send_event_args = (CAM_EVENT_UNKNOWN, -1, conf_pct, "UNKNOWN")
                                hub_action = "INTRUDER_ALARM"
                            elif trigger_hub:
                                hub_action = "COOLDOWN" if cam.cfg["armed"] else "DISARMED"
                        else:
                            display_res = "VERIFYING"
                            hub_action = "CONFIRM_STREAK"
                    elif is_align or face_det:
                        cam.unknown_streak = 0
                        display_res = "ALIGNING"
                    else:
                        cam.unknown_streak = 0
                        display_res = "NO FACE"

                    cam.face_detected = face_det
                    cam.face_quality = face_qual
                    if box is not None:
                        cam.face_box = box
                    elif not face_det:
                        cam.face_box = [0, 0, 0, 0]
                    cam.last_match_id = match_id
                    cam.last_similarity = sim
                    cam.inference_ms = infer_ms
                    cam.feedback_count += 1
                    cam.last_result = display_res
                    cam.last_match_name = name[:23]
                    streak_snap = cam.unknown_streak
                    armed_snap = cam.cfg["armed"]
                    uconf_snap = cam.cfg["unknownConfirm"]

                if send_event_args is not None:
                    hub_dispatched = cam.send_dual_path_event(*send_event_args)

                self._send_json(
                    200,
                    {
                        "ok": True,
                        "result": display_res,
                        "hubDispatched": hub_dispatched,
                        "hubAction": hub_action,
                        "armed": armed_snap,
                        "unknownStreak": streak_snap,
                        "unknownConfirm": uconf_snap,
                    },
                )
                return

            if path == "/api/laptop/heartbeat":
                model_name = self._param(qs, body, "laptopModel", None)
                with cam.lock:
                    cam.last_laptop_ms = now
                    cam.laptop_connected = True
                    if model_name:
                        cam.laptop_model = str(model_name)[:23]
                self._send_json(200, {"ok": True})
                return

            if path == "/api/laptop/enrolled":
                items = body.get("enrolled") if isinstance(body, dict) else None
                if not isinstance(items, list):
                    self._send_json(400, {"ok": False, "error": "invalid enrolled json array"})
                    return
                cleaned = [{"id": int(it["id"]), "name": str(it["name"])[:23]} for it in items if "id" in it]
                with cam.lock:
                    cam.last_laptop_ms = now
                    cam.laptop_connected = True
                    cam.enrolled = cleaned
                self._send_json(200, {"ok": True, "count": len(cleaned)})
                return

            if path == "/api/enroll":
                name = str(self._param(qs, body, "name", "User"))[:23] or "User"
                with cam.lock:
                    cam.enroll_pending = True
                    cam.enroll_start_ms = now
                    cam.enroll_target_name = name
                    cam.enroll_message = f'Waiting for Laptop to capture "{name}"...'
                self._send_json(200, {"queued": True})
                return

            if path == "/api/enroll/cancel":
                with cam.lock:
                    cam.enroll_pending = False
                    cam.enroll_message = "Enrollment cancelled"
                self._send_json(200, {"cancelled": True})
                return

            if path == "/api/enroll/ack":
                ok_raw = self._param(qs, body, "ok", True)
                ok = ok_raw in (True, 1, "1", "true", "True")
                new_id = int(self._param(qs, body, "id", 0))
                name = str(self._param(qs, body, "name", cam.enroll_target_name or "User"))[:23]
                msg = str(self._param(qs, body, "msg", "Enroll failed on Laptop"))
                trigger_hub = self._param(qs, body, "triggerHub", True) in (True, 1, "1", "true", "True")
                with cam.lock:
                    cam.last_laptop_ms = now
                    cam.laptop_connected = True
                    cam.enroll_pending = False
                    if ok and new_id >= 0:
                        if not any(e["id"] == new_id for e in cam.enrolled):
                            cam.enrolled.append({"id": new_id, "name": name})
                        cam.enroll_message = f'Saved "{name}" on Laptop as ID #{new_id}'
                        cam.last_match_id = new_id
                        cam.last_match_name = name
                        cam.last_similarity = 1.0
                        cam.last_result = "RECOGNIZED"
                    else:
                        cam.enroll_message = msg
                hub_sent = False
                if ok and new_id >= 0 and trigger_hub:
                    hub_sent = cam.send_dual_path_event(CAM_EVENT_RECOGNIZED, new_id, 100, name)
                self._send_json(200, {"ok": ok, "hubDispatched": hub_sent})
                return

            if path == "/api/delete":
                del_id = int(self._param(qs, body, "id", -1))
                if del_id < 0:
                    self._send_json(400, {"error": "invalid id"})
                    return
                with cam.lock:
                    cam.pending_action = "delete"
                    cam.pending_action_seq += 1
                    cam.pending_delete_id = del_id
                    cam.enrolled = [e for e in cam.enrolled if e["id"] != del_id]
                    cam.enroll_message = f"Requested Laptop to delete ID #{del_id}"
                self._send_json(200, {"queued": True})
                return

            if path == "/api/clear":
                with cam.lock:
                    cam.pending_action = "clear"
                    cam.pending_action_seq += 1
                    cam.pending_delete_id = -1
                    cam.enrolled.clear()
                    cam.enroll_message = "Requested Laptop to clear all faces"
                self._send_json(200, {"queued": True})
                return

            if path == "/api/command/ack":
                seq = int(self._param(qs, body, "seq", -1))
                with cam.lock:
                    cam.last_laptop_ms = now
                    cam.laptop_connected = True
                    if seq < 0 or seq == cam.pending_action_seq:
                        cam.pending_action = "none"
                        cam.pending_delete_id = -1
                self._send_json(200, {"ok": True})
                return

            if path == "/api/config":
                with cam.lock:
                    for k, cfg_k, cast in [
                        ("armed", "armed", lambda v: int(v) != 0 if isinstance(v, str) else bool(v)),
                        ("uconf", "unknownConfirm", lambda v: max(1, min(5, int(v)))),
                        ("flash", "flashDuty", lambda v: max(0, min(180, int(v)))),
                        ("flash_auto", "flashAuto", lambda v: int(v) != 0 if isinstance(v, str) else bool(v)),
                        ("hmir", "hmirror", lambda v: int(v) != 0 if isinstance(v, str) else bool(v)),
                        ("vflip", "vflip", lambda v: int(v) != 0 if isinstance(v, str) else bool(v)),
                        ("res", "frameRes", lambda v: max(0, min(2, int(v)))),
                        ("qual", "jpegQuality", lambda v: max(8, min(30, int(v)))),
                    ]:
                        v = self._param(qs, body, k, None)
                        if v is not None:
                            cam.cfg[cfg_k] = cast(v)
                    res_dims = [(320, 240), (480, 320), (640, 480)]
                    cam.frame_width, cam.frame_height = res_dims[cam.cfg["frameRes"]]
                self._send_json(200, {"ok": True})
                return

            if path == "/api/test":
                ok = int(self._param(qs, body, "ok", 0)) != 0
                sent = (
                    cam.send_dual_path_event(CAM_EVENT_RECOGNIZED, 0, 100, "TEST-USER")
                    if ok
                    else cam.send_dual_path_event(CAM_EVENT_UNKNOWN, -1, 0, "TEST-UNKNOWN")
                )
                self._send_json(200, {"sent": sent})
                return

            self.send_error(404)

    return CamHttpHandler


def make_stream_handler(cam: StreamCamState):
    class MjpegHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if not self.path.startswith("/stream"):
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            try:
                while True:
                    cam.new_frame_event.wait(timeout=0.5)
                    cam.new_frame_event.clear()
                    with cam.lock:
                        jpg = cam.latest_jpeg
                    if not jpg:
                        continue
                    hdr = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(hdr + jpg + b"\r\n")
                    self.wfile.flush()
            except Exception:
                pass

    return MjpegHandler


def main():
    parser = argparse.ArgumentParser(description="Virtual ESP32-CAM Stream + Laptop AI Digital Twin")
    parser.add_argument("--http-port", type=int, default=8081)
    parser.add_argument("--stream-port", type=int, default=8082)
    args = parser.parse_args()

    cam = StreamCamState(stream_port=args.stream_port)
    stop_event = threading.Event()
    gen_thread = threading.Thread(target=frame_generator_loop, args=(cam, stop_event), daemon=True)
    gen_thread.start()

    http_srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_http_handler(cam))
    stream_srv = ThreadingHTTPServer(("127.0.0.1", args.stream_port), make_stream_handler(cam))

    threading.Thread(target=stream_srv.serve_forever, daemon=True).start()
    print(f"[STREAM-CAM-SIM] Dashboard & Feedback API: http://127.0.0.1:{args.http_port}")
    print(f"[STREAM-CAM-SIM] Clean Raw MJPEG Stream: http://127.0.0.1:{args.stream_port}/stream")
    try:
        http_srv.serve_forever()
    except KeyboardInterrupt:
        stop_event.set()
        http_srv.shutdown()
        stream_srv.shutdown()


if __name__ == "__main__":
    main()
