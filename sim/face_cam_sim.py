#!/usr/bin/env python3
"""
Virtual ESP32-CAM Node 1: Face Detection & Live Stream (Digital Twin of src/cam_face/main.cpp)
Runs:
- Web Management Server on localhost:8081
- Real-Time MJPEG Streaming Server on localhost:8082 (/stream)
Simulates:
- OV2640 camera frame capture (240x240 RGB565 / JPEG)
- 2-stage face detection & 5-point facial landmarks reticle overlay
- MobileFaceNet S8 recognition & Flash database ("fr" partition)
- Dual-Path Event Delivery: UDP (simulated ESP-NOW) + HTTP POST to Hub (/api/cam/event)
"""

import sys
import os
import time
import json
import struct
import socket
import io
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from PIL import Image, ImageDraw, ImageFont

CAM_HTTP_PORT = 8081
CAM_STREAM_PORT = 8082
HUB_UDP_PORT = 9000
HUB_HTTP_URL = "http://127.0.0.1:8080/api/cam/event"

PACKET_MAGIC = 0x53484331 # "SHC1"
CAM_SOURCE_FACE = 1
CAM_EVENT_RECOGNIZED = 1
CAM_EVENT_UNKNOWN = 2
CAM_EVENT_HEARTBEAT = 3

class FaceCamState:
    def __init__(self):
        self.lock = threading.Lock()
        self.enrolled = [
            {"id": 0, "name": "Khaled"},
            {"id": 1, "name": "Admin"}
        ]
        self.mode = "NO_FACE" # "NO_FACE", "RECOGNIZED", "UNKNOWN", "ENROLLING"
        self.active_person = "none"
        self.active_id = 0
        self.active_similarity = 0.94
        
        self.fps = 20.0
        self.frame_count = 0
        self.latest_jpeg = b""
        self.new_frame_event = threading.Event()
        self.seq = 0
        self.udp_enabled = True # For fault-injection testing

    def get_enrolled_list(self):
        with self.lock:
            return list(self.enrolled)

    def enroll_face(self, name):
        with self.lock:
            new_id = len(self.enrolled)
            self.enrolled.append({"id": new_id, "name": name})
            self.mode = "RECOGNIZED"
            self.active_person = name
            self.active_id = new_id
            self.active_similarity = 0.99
            return new_id

    def delete_face(self, face_id):
        with self.lock:
            self.enrolled = [e for e in self.enrolled if e["id"] != face_id]

    def clear_faces(self):
        with self.lock:
            self.enrolled.clear()

    def set_detection_mode(self, mode, name="Khaled", face_id=0, sim=0.94):
        with self.lock:
            self.mode = mode
            self.active_person = name
            self.active_id = face_id
            self.active_similarity = sim

    def send_dual_path_event(self, event_type, match_id, conf, label):
        self.seq += 1
        # 1. Path 1: UDP (simulated ESP-NOW packet, 37 bytes)
        if self.udp_enabled:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                raw_label = label.encode("utf-8")[:23].ljust(24, b"\x00")
                packet = struct.pack("<IBBhBI24s", PACKET_MAGIC, CAM_SOURCE_FACE, event_type, match_id, conf, self.seq, raw_label)
                sock.sendto(packet, ("127.0.0.1", HUB_UDP_PORT))
                sock.close()
            except Exception as e:
                print(f"[CAM-SIM] UDP ESP-NOW send error: {e}")

        # 2. Path 2: HTTP POST fallback to Hub
        if event_type != CAM_EVENT_HEARTBEAT:
            try:
                import urllib.request
                payload = json.dumps({
                    "source": CAM_SOURCE_FACE,
                    "event": event_type,
                    "matchId": match_id,
                    "confidence": conf,
                    "label": label
                }).encode("utf-8")
                req = urllib.request.Request(HUB_HTTP_URL, data=payload, headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(req, timeout=0.5) as resp:
                    pass
            except Exception as e:
                print(f"[CAM-SIM] HTTP fallback send error: {e}")

g_cam = FaceCamState()

# ---------------------------------------------------------------------------
# Synthetic OV2640 Frame Generator (Pillow)
# ---------------------------------------------------------------------------
def frame_generator_thread():
    last_event_time = 0.0
    last_heartbeat_time = 0.0

    while True:
        t0 = time.time()
        with g_cam.lock:
            mode = g_cam.mode
            name = g_cam.active_person
            face_id = g_cam.active_id
            sim = g_cam.active_similarity
            g_cam.frame_count += 1
            frame_num = g_cam.frame_count

        # Create 240x240 dark camera frame
        img = Image.new("RGB", (240, 240), color=(14, 20, 32))
        draw = ImageDraw.Draw(img)

        # Draw grid & crosshair
        draw.line([(120, 0), (120, 240)], fill=(28, 40, 60), width=1)
        draw.line([(0, 120), (240, 120)], fill=(28, 40, 60), width=1)

        # Draw biometric face detection simulation
        if mode == "RECOGNIZED":
            color = (16, 185, 129) # Green
            box = [45, 35, 195, 185]
            draw.rectangle(box, outline=color, width=2)
            # 5-point landmarks (eyes, nose, mouth corners)
            draw.rectangle([80, 85, 86, 91], fill=color)   # Left eye
            draw.rectangle([154, 85, 160, 91], fill=color) # Right eye
            draw.rectangle([117, 115, 123, 121], fill=color) # Nose
            draw.rectangle([88, 148, 94, 154], fill=color)  # Mouth left
            draw.rectangle([146, 148, 152, 154], fill=color) # Mouth right
            draw.text((50, 190), f"{name} ({int(sim*100)}%)", fill=color)
        elif mode == "UNKNOWN":
            color = (239, 68, 68) # Red
            box = [45, 35, 195, 185]
            draw.rectangle(box, outline=color, width=2)
            draw.rectangle([80, 85, 86, 91], fill=color)
            draw.rectangle([154, 85, 160, 91], fill=color)
            draw.rectangle([117, 115, 123, 121], fill=color)
            draw.rectangle([88, 148, 94, 154], fill=color)
            draw.rectangle([146, 148, 152, 154], fill=color)
            draw.text((50, 190), "UNKNOWN INTRUDER", fill=color)
        elif mode == "ENROLLING":
            color = (6, 182, 212) # Cyan
            box = [45, 35, 195, 185]
            draw.rectangle(box, outline=color, width=2)
            draw.text((50, 190), "ENROLLING FACE...", fill=color)
        else:
            draw.text((75, 110), "NO FACE DETECTED", fill=(148, 163, 184))

        # HUD Top Banner
        time_str = time.strftime("%H:%M:%S")
        draw.text((8, 6), f"FPS: {g_cam.fps:.1f} | HW-297 | {time_str}", fill=(241, 245, 249))

        # Encode to JPEG
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=75)
        jpeg_bytes = buf.getvalue()

        with g_cam.lock:
            g_cam.latest_jpeg = jpeg_bytes

        g_cam.new_frame_event.set()
        g_cam.new_frame_event.clear()

        # Dispatch periodic dual-path events & heartbeats
        now = time.time()
        if now - last_heartbeat_time >= 5.0:
            last_heartbeat_time = now
            g_cam.send_dual_path_event(CAM_EVENT_HEARTBEAT, 0, 100, "FACE-CAM")

        if now - last_event_time >= 3.0:
            if mode == "RECOGNIZED":
                last_event_time = now
                g_cam.send_dual_path_event(CAM_EVENT_RECOGNIZED, face_id, int(sim * 100), name)
            elif mode == "UNKNOWN":
                last_event_time = now
                g_cam.send_dual_path_event(CAM_EVENT_UNKNOWN, -1, 0, "UNKNOWN")

        # Maintain ~20 FPS (50 ms frame interval)
        elapsed = time.time() - t0
        time.sleep(max(0.005, 0.050 - elapsed))

# ---------------------------------------------------------------------------
# MJPEG Streaming Server on Port 8082
# ---------------------------------------------------------------------------
class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/stream" or self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--frame")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()

            while True:
                try:
                    with g_cam.lock:
                        frame = g_cam.latest_jpeg
                    if frame:
                        header = f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode("utf-8")
                        self.wfile.write(header)
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.050)
                except Exception:
                    break
        else:
            self.send_response(404)
            self.end_headers()

def run_stream_server():
    server = HTTPServer(("0.0.0.0", CAM_STREAM_PORT), MJPEGStreamHandler)
    print(f"[CAM-SIM] MJPEG Live Stream Server listening on http://localhost:{CAM_STREAM_PORT}/stream")
    server.serve_forever()

# ---------------------------------------------------------------------------
# Web Management Server on Port 8081
# ---------------------------------------------------------------------------
FACE_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Smart Camera 1 — Face Recognition (Simulator)</title>
<style>
  body{font-family:system-ui,sans-serif;background:#090d16;color:#f1f5f9;padding:16px}
  .card{background:#131c2e;border:1px solid #23324d;border-radius:12px;padding:16px;max-width:540px;margin:0 auto}
  .view{background:#000;aspect-ratio:1/1;border-radius:8px;overflow:hidden;position:relative}
  img{width:100%;height:100%;object-fit:cover;display:block}
  .badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:.75rem;font-weight:700}
  .b-ok{background:#065f46;color:#a7f3d0}.b-bad{background:#991b1b;color:#fecaca}
  button{background:#3b82f6;color:#fff;border:none;padding:6px 12px;border-radius:6px;font-weight:600;cursor:pointer;margin:4px}
  button.danger{background:#ef4444}
</style>
</head>
<body>
<div class="card">
  <h2>Smart Camera 1: Face Recognition &amp; Stream</h2>
  <div class="view" style="margin:12px 0">
    <img src="http://localhost:8082/stream" alt="Live Stream">
  </div>
  <div>
    <strong>Simulate Camera View:</strong>
    <button onclick="setSim('RECOGNIZED', 'Khaled', 0)">Recognize Khaled</button>
    <button onclick="setSim('RECOGNIZED', 'Admin', 1)">Recognize Admin</button>
    <button class="danger" onclick="setSim('UNKNOWN', 'Unknown', -1)">Simulate Intruder</button>
    <button onclick="setSim('NO_FACE', 'none', -1)">Clear View</button>
  </div>
  <div style="margin-top:14px">
    <input id="enrollName" type="text" placeholder="Name to enroll" style="padding:6px;border-radius:4px;border:1px solid #334155;background:#0b1120;color:#fff">
    <button onclick="enroll()">Enroll Face</button>
  </div>
</div>
<script>
async function setSim(mode, name, id){
  await fetch('/api/simulate?mode=' + mode + '&name=' + name + '&id=' + id, {method:'POST'});
}
async function enroll(){
  const n = document.getElementById('enrollName').value.trim();
  if(!n) return;
  await fetch('/api/enroll?name=' + encodeURIComponent(n), {method:'POST'});
  alert('Enrolled ' + n + ' in Flash DB');
}
</script>
</body>
</html>"""

class CamHttpHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(FACE_PAGE.encode("utf-8"))
            return

        elif path == "/preview.jpg":
            with g_cam.lock:
                jpg = g_cam.latest_jpeg
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            self.wfile.write(jpg)
            return

        elif path == "/stream":
            self.send_response(302)
            self.send_header("Location", f"http://localhost:{CAM_STREAM_PORT}/stream")
            self.end_headers()
            return

        elif path == "/api/status":
            with g_cam.lock:
                data = {
                    "cameraReady": True,
                    "modelReady": True,
                    "faceDetected": g_cam.mode != "NO_FACE",
                    "lastResult": g_cam.mode,
                    "lastMatchName": g_cam.active_person,
                    "lastSimilarity": g_cam.active_similarity,
                    "fps": g_cam.fps,
                    "frameCount": g_cam.frame_count,
                    "enrolledCount": len(g_cam.enrolled),
                    "enrolled": g_cam.enrolled
                }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == "/api/simulate":
            mode = qs.get("mode", ["RECOGNIZED"])[0]
            name = qs.get("name", ["Khaled"])[0]
            face_id = int(qs.get("id", [0])[0])
            g_cam.set_detection_mode(mode, name, face_id)
            if mode == "RECOGNIZED":
                g_cam.send_dual_path_event(CAM_EVENT_RECOGNIZED, face_id, 96, name)
            elif mode == "UNKNOWN":
                g_cam.send_dual_path_event(CAM_EVENT_UNKNOWN, -1, 0, "UNKNOWN")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"simulated"}')
            return

        elif path == "/api/enroll":
            name = qs.get("name", ["NewUser"])[0]
            new_id = g_cam.enroll_face(name)
            g_cam.send_dual_path_event(CAM_EVENT_RECOGNIZED, new_id, 99, name)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"enrolledId": new_id, "name": name}).encode("utf-8"))
            return

        elif path == "/api/delete":
            face_id = int(qs.get("id", [-1])[0])
            g_cam.delete_face(face_id)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"deleted"}')
            return

        elif path == "/api/clear":
            g_cam.clear_faces()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"cleared"}')
            return

        self.send_response(404)
        self.end_headers()

def run_face_cam():
    t_gen = threading.Thread(target=frame_generator_thread, daemon=True)
    t_gen.start()

    t_stream = threading.Thread(target=run_stream_server, daemon=True)
    t_stream.start()

    server = HTTPServer(("0.0.0.0", CAM_HTTP_PORT), CamHttpHandler)
    print(f"[CAM-SIM] Virtual Face Cam Web Management Server listening at http://localhost:{CAM_HTTP_PORT}")
    server.serve_forever()

if __name__ == "__main__":
    run_face_cam()
