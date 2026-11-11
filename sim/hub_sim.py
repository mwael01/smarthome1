#!/usr/bin/env python3
"""
Virtual Main ESP32 Hub (Digital Twin of src/main.cpp)
Runs on localhost:8080.
Simulates:
- SoftAP Web Admin Dashboard & REST APIs
- Dual-Path Receiver: UDP (simulated ESP-NOW on port 9000) + HTTP POST (/api/cam/event)
- Servos 1 & 2 (Car Gate) & Servo 3 (Face Door) clamped to 160° max with 2°/20ms ramping
- MFRC522 RFID card reader (Card 52411902 -> Gate Open 160°; Unknown -> Intruder Alarm)
- Environmental & Proximity Sensors (Flame, Smoke, PIR, Ultrasonic)
- Proxy streaming for Camera 1 (/api/cam/1/preview.jpg)
"""

import sys
import os
import time
import json
import struct
import socket
import threading
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ---------------------------------------------------------------------------
# Constants matching src/main.cpp
# ---------------------------------------------------------------------------
HUB_HTTP_PORT = 8080
HUB_UDP_PORT = 9000
ACCEPTED_RFID_UID = "52411902"

SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 160
SERVO_STEP_DEG = 2
SERVO_STEP_INTERVAL = 0.020  # 20 ms

GATE_HOLD_TIME = 5.0   # seconds
DOOR_HOLD_TIME = 3.0   # seconds
ALARM_HOLD_TIME = 4.0  # seconds
CAM_ONLINE_TIMEOUT = 15.0 # seconds

PACKET_MAGIC = 0x53484331 # "SHC1"
CAM_SOURCE_FACE = 1
CAM_EVENT_RECOGNIZED = 1
CAM_EVENT_UNKNOWN = 2
CAM_EVENT_HEARTBEAT = 3

# ---------------------------------------------------------------------------
# Hub State
# ---------------------------------------------------------------------------
class HubState:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()
        
        # Servos
        self.servo1_angle = 0
        self.servo2_angle = 0
        self.door3_angle = 0
        self.target_servo1 = 0
        self.target_servo2 = 0
        self.target_door3 = 0
        self.gate_open = False
        self.door_open = False
        self.gate_until = 0.0
        self.door_until = 0.0
        
        # Alarms
        self.siren_active = False
        self.siren_muted = False
        self.siren_hz = 700
        self.cam_alarm_until = 0.0
        self.siren_test_until = 0.0
        
        # Sensors
        self.flame_adc = 3800
        self.smoke_adc = 400
        self.pir = False
        self.dist_cm = 45.0
        self.fire_threshold = 500
        self.smoke_threshold = 2000
        self.ultra_threshold = 25.0
        
        # RFID
        self.last_rfid_uid = ""
        self.last_rfid_time = 0.0
        
        # Face Cam Telemetry
        self.face_cam_seen = False
        self.face_cam_last_seen = 0.0
        self.face_cam_last_event_time = 0.0
        self.face_cam_last_event = 0
        self.face_cam_last_id = -1
        self.face_cam_last_conf = 0
        self.face_cam_last_label = "none"
        self.face_cam_seq = 0
        
        # Event Log
        self.events = []
        self.add_event("info", "Smart Home Hub started (Simulator). Web Admin ready.")

    def add_event(self, ev_type, msg):
        sec = int(time.time() - self.start_time)
        h = sec // 3600
        m = (sec % 3600) // 60
        s = sec % 60
        time_str = f"{h:02d}:{m:02d}:{s:02d}"
        self.events.append({"time": time_str, "type": ev_type, "msg": msg})
        if len(self.events) > 16:
            self.events.pop(0)
        print(f"[{time_str}] [{ev_type.upper()}] {msg}")

    def update_actuators(self):
        with self.lock:
            now = time.time()
            
            # 1. Gate holding timer
            if self.gate_open and now >= self.gate_until:
                self.gate_open = False
                self.target_servo1 = 0
                self.target_servo2 = 0
                self.add_event("info", "Car gate closing (Servos 1 & 2)")

            # 2. Door holding timer
            if self.door_open and now >= self.door_until:
                self.door_open = False
                self.target_door3 = 0
                self.add_event("info", "Face door locked (Servo 3)")

            # 3. Siren timers
            alarm_active = False
            if self.cam_alarm_until > 0:
                if now < self.cam_alarm_until:
                    alarm_active = True
                else:
                    self.cam_alarm_until = 0.0

            if self.siren_test_until > 0:
                if now < self.siren_test_until:
                    alarm_active = True
                else:
                    self.siren_test_until = 0.0

            # Environmental sensor triggers
            if self.flame_adc <= self.fire_threshold:
                alarm_active = True
            if self.smoke_adc >= self.smoke_threshold:
                alarm_active = True
            if 0 < self.dist_cm <= self.ultra_threshold:
                alarm_active = True

            self.siren_active = alarm_active

            # 4. Step servos smoothly towards target (clamped strictly to [0, 160])
            def step(curr, target):
                target = max(0, min(target, 160)) # Safety clamping
                if curr < target:
                    return min(curr + SERVO_STEP_DEG, target)
                elif curr > target:
                    return max(curr - SERVO_STEP_DEG, target)
                return curr

            self.servo1_angle = step(self.servo1_angle, self.target_servo1)
            self.servo2_angle = step(self.servo2_angle, self.target_servo2)
            self.door3_angle = step(self.door3_angle, self.target_door3)

    def handle_cam_packet(self, source, event, match_id, conf, label):
        with self.lock:
            now = time.time()
            if source != CAM_SOURCE_FACE:
                return # Plate camera disabled

            self.face_cam_seen = True
            self.face_cam_last_seen = now

            if event == CAM_EVENT_HEARTBEAT:
                return

            self.face_cam_last_event_time = now
            self.face_cam_last_event = event
            self.face_cam_last_id = match_id
            self.face_cam_last_conf = conf
            self.face_cam_last_label = label

            if event == CAM_EVENT_RECOGNIZED:
                self.cam_alarm_until = 0.0 # Cancel intruder alarm
                self.door_open = True
                self.door_until = now + DOOR_HOLD_TIME
                self.target_door3 = 160 # Open to 160°
                self.add_event("success", f'Face recognized: "{label}" ({conf}%) -> Door unlocked (160°)')
            elif event == CAM_EVENT_UNKNOWN:
                self.cam_alarm_until = now + ALARM_HOLD_TIME
                self.add_event("danger", f'INTRUDER ALERT: Unrecognized face detected! Siren active ({ALARM_HOLD_TIME}s)')

    def handle_rfid_scan(self, uid):
        with self.lock:
            now = time.time()
            self.last_rfid_uid = uid.upper()
            self.last_rfid_time = now

            if self.last_rfid_uid == ACCEPTED_RFID_UID:
                self.gate_open = True
                self.gate_until = now + GATE_HOLD_TIME
                self.target_servo1 = 160 # Open to 160°
                self.target_servo2 = 160 # Open to 160°
                self.add_event("success", f'RFID Authorized [{self.last_rfid_uid}] -> Car Gate opened (Servos 1&2 to 160°)')
                return True
            else:
                self.cam_alarm_until = now + ALARM_HOLD_TIME
                self.add_event("danger", f'RFID Unknown [{self.last_rfid_uid}] -> Intruder alarm started')
                return False

g_hub = HubState()

# ---------------------------------------------------------------------------
# Background Servo / Actuator Simulation Loop
# ---------------------------------------------------------------------------
def hub_actuator_thread():
    while True:
        g_hub.update_actuators()
        time.sleep(SERVO_STEP_INTERVAL)

# ---------------------------------------------------------------------------
# UDP Receiver Thread (Simulates ESP-NOW packet reception)
# ---------------------------------------------------------------------------
def hub_udp_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", HUB_UDP_PORT))
    print(f"[HUB-SIM] Simulated ESP-NOW UDP receiver listening on port {HUB_UDP_PORT}")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            # Struct format: <I B B h B I 24s (total 37 bytes)
            if len(data) == 37:
                magic, source, event, match_id, conf, seq, raw_label = struct.unpack("<IBBhBI24s", data)
                if magic == PACKET_MAGIC:
                    label = raw_label.split(b"\x00")[0].decode("utf-8", errors="ignore")
                    g_hub.handle_cam_packet(source, event, match_id, conf, label)
        except Exception as e:
            time.sleep(0.01)

# ---------------------------------------------------------------------------
# HTTP Server (Matching src/main.cpp routes)
# ---------------------------------------------------------------------------
HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Home Security & Access Hub (Simulator)</title>
<style>
  :root{
    --bg:#0b1120;--surface:#1e293b;--surface2:#334155;--border:#475569;
    --text:#f8fafc;--muted:#94a3b8;--primary:#3b82f6;--primary-hover:#2563eb;
    --success:#10b981;--warning:#f59e0b;--danger:#ef4444;--danger-bg:#7f1d1d;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--text);padding:14px;line-height:1.45}
  .wrap{max-width:1120px;margin:0 auto}
  header{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:12px 16px;margin-bottom:14px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px}
  .h-title h1{font-size:1.25rem;font-weight:700}
  .grid-2col{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;margin-bottom:14px}
  .card{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:14px}
  .card-title{font-size:.9rem;font-weight:700;color:var(--muted);text-transform:uppercase;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center}
  .badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:.72rem;font-weight:700;letter-spacing:.02em}
  .b-ok{background:#065f46;color:#a7f3d0}
  .b-bad{background:#991b1b;color:#fecaca}
  .b-warn{background:#92400e;color:#fef3c7}
  .b-off{background:#334155;color:#94a3b8}
  .cam-box{background:#020617;border-radius:8px;overflow:hidden;position:relative;aspect-ratio:4/3;margin-bottom:10px}
  .cam-img{width:100%;height:100%;object-fit:cover;display:block}
  .cam-meta .row{display:flex;justify-content:space-between;padding:4px 0;font-size:.85rem;border-bottom:1px solid rgba(255,255,255,0.05)}
  .cam-actions{display:flex;gap:6px;margin-top:10px;flex-wrap:wrap}
  button{background:var(--primary);color:#fff;border:none;padding:6px 12px;border-radius:6px;font-size:.82rem;font-weight:600;cursor:pointer;transition:background .15s}
  button:hover{background:var(--primary-hover)}
  button.secondary{background:var(--surface2)}
  button.danger{background:var(--danger)}
  .log-line{padding:3px 0;font-family:monospace;font-size:.8rem;border-bottom:1px solid rgba(255,255,255,0.05)}
  .c-danger{color:#f87171}.c-warning{color:#fbbf24}.c-success{color:#34d399}.c-info{color:#38bdf8}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div class="h-title">
      <h1>🏠 Smart Home Hub &amp; Digital Twin (160° Servos &amp; Face Cam)</h1>
    </div>
    <div style="display:flex;gap:8px;align-items:center">
      <span id="badgeAlarm" class="badge b-ok">SYSTEM NORMAL</span>
      <button class="danger" onclick="triggerAction('siren_test')">Test Siren</button>
    </div>
  </header>

  <div class="grid-2col">
    <!-- Camera 1: Face Recognition -->
    <div class="card">
      <div class="card-title">
        <span>Camera 1: Face Recognition (HW-297)</span>
        <span id="cam1Status" class="badge b-off">CONNECTING...</span>
      </div>
      <div class="cam-box">
        <img id="cam1Img" class="cam-img" alt="Camera 1 Stream">
      </div>
      <div class="cam-meta">
        <div class="row"><span>Identified Person</span><span id="cam1Who">---</span></div>
        <div class="row"><span>Link / Last Seen</span><span id="cam1Link">---</span></div>
      </div>
      <div class="cam-actions">
        <button onclick="triggerAction('door_open')">🔓 Unlock Door (Servo 3 to 160°)</button>
        <button class="secondary" onclick="triggerAction('door_close')">🔒 Lock Door</button>
        <a class="badge b-ok" href="http://localhost:8081/" target="_blank" style="text-decoration:none;padding:6px 10px">Open Dedicated Face UI ↗</a>
      </div>
    </div>

    <!-- RFID Car Gate Access Control -->
    <div class="card">
      <div class="card-title">
        <span>🚗 RFID Car Gate Access (MFRC522)</span>
        <span id="rfidStatus" class="badge b-ok">ACTIVE</span>
      </div>
      <div style="padding:16px 12px;text-align:center;background:#0d1527;border-radius:8px;margin-bottom:12px;border:1px dashed #23324d">
        <div style="font-size:1.8rem;margin-bottom:4px">💳</div>
        <div style="font-size:.82rem;color:var(--muted)">Present RFID card to open Car Gate (Servos 1&amp;2 to 160°)</div>
        <div style="margin-top:8px;font-family:monospace;font-size:1.15rem;font-weight:700;color:#06b6d4" id="rfidUidText">NO CARD SCANNED</div>
      </div>
      <div class="cam-meta">
        <div class="row"><span>Accepted Card UID</span><span id="rfidAccepted" style="font-family:monospace">52411902</span></div>
        <div class="row"><span>Last Scan</span><span id="rfidLastSeen">---</span></div>
      </div>
      <div class="cam-actions">
        <button onclick="scanRfid('52411902')">💳 Scan Accepted Card (52411902)</button>
        <button class="danger" onclick="scanRfid('B4C3D2E1')">❌ Scan Unknown Card</button>
        <button class="secondary" onclick="triggerAction('gate_open')">🚗 Open Gate (160°)</button>
        <button class="secondary" onclick="triggerAction('gate_close')">⛔ Close Gate</button>
      </div>
    </div>
  </div>

  <!-- Hardware State & Event Log -->
  <div class="grid-2col">
    <div class="card">
      <div class="card-title">⚙️ Actuators &amp; Safety Angles</div>
      <div class="cam-meta">
        <div class="row"><span>Car Gate (Servos 1 &amp; 2)</span><span id="gateState">CLOSED (0° / 0°)</span></div>
        <div class="row"><span>Face Door (Servo 3)</span><span id="doorState">CLOSED (0°)</span></div>
        <div class="row"><span>Siren Tone (GPIO25)</span><span id="sirenHz">OFF</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">📋 Rolling System Event Log</div>
      <div id="logBox" style="max-height:160px;overflow-y:auto"></div>
    </div>
  </div>
</div>

<script>
function streamCam(){
  const img = document.getElementById('cam1Img');
  img.src = "/api/cam/1/preview.jpg?t=" + Date.now();
  img.onload = () => setTimeout(streamCam, 350);
  img.onerror = () => setTimeout(streamCam, 1500);
}

async function refreshStatus(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();

    document.getElementById('badgeAlarm').textContent = d.alarm ? (d.camAlarm ? "INTRUDER ALERT" : "ALARM ACTIVE") : "SYSTEM NORMAL";
    document.getElementById('badgeAlarm').className = "badge " + (d.alarm ? "b-bad" : "b-ok");

    document.getElementById('gateState').textContent = (d.gateOpen ? "OPEN " : "CLOSED ") + "(" + d.servo1 + "°/" + d.servo2 + "°)";
    document.getElementById('doorState').textContent = (d.doorOpen ? "OPEN " : "CLOSED ") + "(" + d.door3 + "°)";
    document.getElementById('sirenHz').textContent = d.sirenActive ? (d.sirenHz + " Hz") : "OFF";

    document.getElementById('cam1Status').textContent = d.faceCam.online ? "ONLINE" : "OFFLINE";
    document.getElementById('cam1Status').className = "badge " + (d.faceCam.online ? "b-ok" : "b-bad");
    document.getElementById('cam1Who').textContent = d.faceCam.label + " (" + d.faceCam.conf + "%)";
    document.getElementById('cam1Link').textContent = d.faceCam.online ? ("Online (" + d.faceCam.ageSec + "s ago)") : "Offline";

    if(d.rfid){
      document.getElementById('rfidUidText').textContent = d.rfid.lastUid ? d.rfid.lastUid : "NO CARD SCANNED";
      document.getElementById('rfidAccepted').textContent = d.rfid.acceptedUid;
      document.getElementById('rfidLastSeen').textContent = d.rfid.lastScanSec < 9999 ? (d.rfid.lastScanSec + "s ago") : "---";
      const isAuth = (d.rfid.lastUid === d.rfid.acceptedUid);
      document.getElementById('rfidStatus').textContent = isAuth ? "AUTHORIZED" : (d.rfid.lastUid ? "DENIED" : "ACTIVE");
      document.getElementById('rfidStatus').className = "badge " + (isAuth ? "b-ok" : (d.rfid.lastUid ? "b-bad" : "b-ok"));
    }

    const box = document.getElementById('logBox');
    box.innerHTML = '';
    d.events.slice().reverse().forEach(ev=>{
      const line = document.createElement('div');
      line.className = 'log-line';
      line.innerHTML = '<span style="color:#64748b;margin-right:6px">[' + ev.time + ']</span><span class="c-' + ev.type + '">' + ev.msg + '</span>';
      box.appendChild(line);
    });
  }catch(e){}
}

async function triggerAction(act){
  await fetch('/api/control?action=' + act, {method:'POST'});
  refreshStatus();
}

async function scanRfid(uid){
  await fetch('/api/rfid/scan?uid=' + uid, {method:'POST'});
  refreshStatus();
}

streamCam();
setInterval(refreshStatus, 800);
refreshStatus();
</script>
</body>
</html>"""

class HubRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Silence default access logs

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
            return

        elif path == "/api/status":
            now = time.time()
            with g_hub.lock:
                face_online = g_hub.face_cam_seen and (now - g_hub.face_cam_last_seen <= CAM_ONLINE_TIMEOUT)
                data = {
                    "uptimeSec": int(now - g_hub.start_time),
                    "uptimeStr": f"{int(now - g_hub.start_time)}s",
                    "freeHeap": 220000,
                    "flameAdc": g_hub.flame_adc,
                    "fire": g_hub.flame_adc <= g_hub.fire_threshold,
                    "smokeAdc": g_hub.smoke_adc,
                    "smoke": g_hub.smoke_adc >= g_hub.smoke_threshold,
                    "pir": g_hub.pir,
                    "ultra": 0 < g_hub.dist_cm <= g_hub.ultra_threshold,
                    "distCm": g_hub.dist_cm,
                    "alarm": g_hub.siren_active,
                    "camAlarm": g_hub.cam_alarm_until > now,
                    "sirenActive": g_hub.siren_active,
                    "sirenMuted": g_hub.siren_muted,
                    "sirenHz": g_hub.siren_hz if g_hub.siren_active else 0,
                    "gateOpen": g_hub.gate_open,
                    "servo1": g_hub.servo1_angle,
                    "servo2": g_hub.servo2_angle,
                    "doorOpen": g_hub.door_open,
                    "door3": g_hub.door3_angle,
                    "rxDropped": 0,
                    "wifi": {"apSsid": "SmartHome-Net", "apIp": "192.168.4.1", "apStations": 1},
                    "faceCam": {
                        "online": face_online,
                        "event": "RECOGNIZED" if g_hub.face_cam_last_event == CAM_EVENT_RECOGNIZED else ("UNKNOWN" if g_hub.face_cam_last_event == CAM_EVENT_UNKNOWN else "NONE"),
                        "id": g_hub.face_cam_last_id,
                        "conf": g_hub.face_cam_last_conf,
                        "label": g_hub.face_cam_last_label,
                        "ageSec": int(now - g_hub.face_cam_last_event_time) if g_hub.face_cam_seen else 9999
                    },
                    "plateCam": {"online": False, "event": "DISABLED"},
                    "rfid": {
                        "lastUid": g_hub.last_rfid_uid,
                        "acceptedUid": ACCEPTED_RFID_UID,
                        "lastScanSec": int(now - g_hub.last_rfid_time) if g_hub.last_rfid_time > 0 else 9999
                    },
                    "events": g_hub.events
                }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode("utf-8"))
            return

        elif path == "/api/cam/1/preview.jpg":
            # Fast offline check matching firmware BUG-1 fix
            now = time.time()
            with g_hub.lock:
                face_online = g_hub.face_cam_seen and (now - g_hub.face_cam_last_seen <= CAM_ONLINE_TIMEOUT)
            if not face_online:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"Face camera offline (no telemetry)")
                return

            # Proxy to virtual face camera
            try:
                req = urllib.request.Request("http://127.0.0.1:8081/preview.jpg", headers={"User-Agent": "HubProxy"})
                with urllib.request.urlopen(req, timeout=1.0) as resp:
                    jpg_data = resp.read()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg_data)))
                self.send_header("Cache-Control", "no-cache, no-store")
                self.end_headers()
                self.wfile.write(jpg_data)
                return
            except Exception:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"Face Cam unreachable")
                return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == "/api/control":
            act = qs.get("action", [""])[0]
            with g_hub.lock:
                now = time.time()
                if act == "gate_open":
                    g_hub.gate_open = True
                    g_hub.gate_until = now + GATE_HOLD_TIME
                    g_hub.target_servo1 = 160
                    g_hub.target_servo2 = 160
                    g_hub.add_event("info", "Manual control: Car gate opened (160°)")
                elif act == "gate_close":
                    g_hub.gate_open = False
                    g_hub.target_servo1 = 0
                    g_hub.target_servo2 = 0
                    g_hub.add_event("info", "Manual control: Car gate closed (0°)")
                elif act == "door_open":
                    g_hub.door_open = True
                    g_hub.door_until = now + DOOR_HOLD_TIME
                    g_hub.target_door3 = 160
                    g_hub.add_event("info", "Manual control: Face door unlocked (160°)")
                elif act == "door_close":
                    g_hub.door_open = False
                    g_hub.target_door3 = 0
                    g_hub.add_event("info", "Manual control: Face door locked (0°)")
                elif act == "siren_test":
                    g_hub.siren_test_until = now + 3.0
                    g_hub.siren_active = True
                    g_hub.add_event("warning", "Manual control: Siren test activated (3s)")
                elif act == "siren_silence":
                    g_hub.siren_test_until = 0.0
                    g_hub.cam_alarm_until = 0.0
                    g_hub.siren_active = False
                    g_hub.add_event("info", "Manual control: Alarm silenced")
            
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
            return

        elif path == "/api/cam/event":
            # Dual-path HTTP event receiver
            content_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_len).decode("utf-8")
            try:
                ev = json.loads(body)
                source = ev.get("source", 1)
                event_type = ev.get("event", 1)
                match_id = ev.get("matchId", -1)
                conf = ev.get("confidence", 100)
                label = ev.get("label", "none")
                g_hub.handle_cam_packet(source, event_type, match_id, conf, label)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"ok","path":"http"}')
                return
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'{"error":"invalid_json"}')
                return

        elif path == "/api/rfid/scan":
            uid = qs.get("uid", ["52411902"])[0]
            ok = g_hub.handle_rfid_scan(uid)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"scanned": uid, "authorized": ok}).encode("utf-8"))
            return

        elif path == "/api/sensor/inject":
            with g_hub.lock:
                if "flame" in qs: g_hub.flame_adc = int(qs["flame"][0])
                if "smoke" in qs: g_hub.smoke_adc = int(qs["smoke"][0])
                if "pir" in qs: g_hub.pir = (qs["pir"][0] in ("1", "true"))
                if "dist" in qs: g_hub.dist_cm = float(qs["dist"][0])
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"sensor_updated"}')
            return

        self.send_response(404)
        self.end_headers()

def run_hub():
    t_act = threading.Thread(target=hub_actuator_thread, daemon=True)
    t_act.start()

    t_udp = threading.Thread(target=hub_udp_thread, daemon=True)
    t_udp.start()

    server = HTTPServer(("0.0.0.0", HUB_HTTP_PORT), HubRequestHandler)
    print(f"[HUB-SIM] Virtual Hub Web Server listening at http://localhost:{HUB_HTTP_PORT}")
    server.serve_forever()

if __name__ == "__main__":
    run_hub()
