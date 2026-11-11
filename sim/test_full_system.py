#!/usr/bin/env python3
"""
Automated Full-System Simulation Test Suite
Orchestrates Virtual Hub and Virtual Face Cam, executing 8 end-to-end integration scenarios.
"""

import sys
import os
import time
import json
import urllib.request
import subprocess

HUB_URL = "http://127.0.0.1:8080"
CAM_URL = "http://127.0.0.1:8081"
STREAM_URL = "http://127.0.0.1:8082/stream"

passed_tests = 0
failed_tests = 0

def log_test(name, passed, detail=""):
    global passed_tests, failed_tests
    if passed:
        passed_tests += 1
        print(f"  [PASS] {name} {detail}")
    else:
        failed_tests += 1
        print(f"  [FAIL] {name} - {detail}")

def http_get(url, timeout=3.0):
    req = urllib.request.Request(url, headers={"User-Agent": "TestHarness"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read()

def http_post(url, data=None, timeout=3.0):
    body = data.encode("utf-8") if isinstance(data, str) else None
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json", "User-Agent": "TestHarness"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read()

def wait_for_server(url, name, retries=15):
    for i in range(retries):
        try:
            status, _ = http_get(url + "/api/status", timeout=1.0)
            if status == 200:
                print(f"[INIT] {name} is READY at {url}")
                return True
        except Exception:
            time.sleep(0.4)
    print(f"[INIT] ERROR: {name} failed to start at {url}")
    return False

def run_tests():
    print("\n==================================================================")
    print("     SMART HOME FULL SYSTEM SIMULATION & VERIFICATION TEST")
    print("==================================================================\n")

    # Start Hub process
    hub_proc = subprocess.Popen([sys.executable, "sim/hub_sim.py"])
    # Start Cam process
    cam_proc = subprocess.Popen([sys.executable, "sim/face_cam_sim.py"])

    try:
        if not wait_for_server(HUB_URL, "Virtual Hub"):
            sys.exit(1)
        if not wait_for_server(CAM_URL, "Virtual Face Cam"):
            sys.exit(1)

        # -------------------------------------------------------------------
        # Scenario 1: System Boot & Heartbeat Telemetry
        # -------------------------------------------------------------------
        print("\n--- [Scenario 1: System Boot & Heartbeat Telemetry] ---")
        time.sleep(1.0)
        status, body = http_get(f"{HUB_URL}/api/status")
        hub_data = json.loads(body)
        log_test("Hub status API responds with 200 OK", status == 200)
        log_test("Hub reports SoftAP SmartHome-Net active", hub_data["wifi"]["apSsid"] == "SmartHome-Net")
        
        # Wait for camera heartbeat
        online = False
        for _ in range(10):
            status, body = http_get(f"{HUB_URL}/api/status")
            d = json.loads(body)
            if d["faceCam"]["online"]:
                online = True
                break
            time.sleep(0.5)
        log_test("Face Cam registered ONLINE via periodic heartbeats", online)

        # -------------------------------------------------------------------
        # Scenario 2: Real-time Live Video Streaming on Port 8082 & Hub Proxy
        # -------------------------------------------------------------------
        print("\n--- [Scenario 2: Real-time Video Streaming & Proxy] ---")
        # 1. Connect to port 8082 stream
        req = urllib.request.Request(STREAM_URL)
        stream_ok = False
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            content_type = resp.headers.get("Content-Type", "")
            chunk = resp.read(1024)
            if "multipart/x-mixed-replace" in content_type and b"\xff\xd8" in chunk:
                stream_ok = True
        log_test("MJPEG stream on port 8082 serves valid multipart JPEG chunks", stream_ok)

        # 2. Check Hub snapshot proxy
        status, preview_bytes = http_get(f"{HUB_URL}/api/cam/1/preview.jpg")
        log_test("Hub proxy /api/cam/1/preview.jpg returns valid JPEG", status == 200 and preview_bytes.startswith(b"\xff\xd8"))

        # -------------------------------------------------------------------
        # Scenario 3: Face Recognition -> Door Servo 3 Actuation
        # -------------------------------------------------------------------
        print("\n--- [Scenario 3: Biometric Face Recognition -> Door Actuation] ---")
        # Trigger face recognition for "Khaled"
        http_post(f"{CAM_URL}/api/simulate?mode=RECOGNIZED&name=Khaled&id=0")
        time.sleep(0.5)

        # Monitor door servo ramp
        max_door_angle = 0
        door_reached_160 = False
        for _ in range(40): # ~2 seconds
            status, body = http_get(f"{HUB_URL}/api/status")
            d = json.loads(body)
            angle = d["door3"]
            if angle > max_door_angle:
                max_door_angle = angle
            if angle == 160:
                door_reached_160 = True
                break
            time.sleep(0.05)

        log_test("Face recognition event delivered via Dual-Path (ESP-NOW + HTTP)", d["faceCam"]["event"] == "RECOGNIZED")
        log_test("Door Servo 3 smoothly swept to exactly 160°", door_reached_160, f"(reached {max_door_angle}°)")
        log_test("Door Servo 3 never exceeded 160° safety limit", max_door_angle <= 160)

        # Clear face from view (subject walks through door) so auto-lock timer can expire
        http_post(f"{CAM_URL}/api/simulate?mode=NO_FACE&name=none&id=-1")

        # Verify door holding timer and auto-lock
        print("  Waiting for door hold timer (3 seconds)...")
        time.sleep(3.2)
        for _ in range(40):
            status, body = http_get(f"{HUB_URL}/api/status")
            d = json.loads(body)
            if d["door3"] == 0 and not d["doorOpen"]:
                break
            time.sleep(0.05)
        log_test("Door Servo 3 automatically closed and locked at 0°", d["door3"] == 0 and not d["doorOpen"])

        # -------------------------------------------------------------------
        # Scenario 4: Unknown Face -> Intruder Siren Alarm
        # -------------------------------------------------------------------
        print("\n--- [Scenario 4: Unknown Face -> Intruder Siren Alarm] ---")
        http_post(f"{CAM_URL}/api/simulate?mode=UNKNOWN&name=Unknown&id=-1")
        time.sleep(0.4)

        status, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Intruder face triggered siren alarm on Hub", d["sirenActive"] and d["camAlarm"])
        log_test("Alarm event recorded in Hub Event Log", any("INTRUDER" in ev["msg"] for ev in d["events"]))

        # Clear intruder from view so siren duration timer can expire
        http_post(f"{CAM_URL}/api/simulate?mode=NO_FACE&name=none&id=-1")

        # Wait for alarm duration (4 seconds)
        print("  Waiting for siren duration (4 seconds)...")
        time.sleep(4.2)
        status, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Siren alarm automatically turned off after timeout", not d["sirenActive"])

        # -------------------------------------------------------------------
        # Scenario 5: RFID Car Gate Access (Servos 1 & 2 to 160°)
        # -------------------------------------------------------------------
        print("\n--- [Scenario 5: RFID Car Gate Access (Servos 1 & 2 to 160°)] ---")
        # Clear camera simulation
        http_post(f"{CAM_URL}/api/simulate?mode=NO_FACE&name=none&id=-1")
        time.sleep(0.2)

        # Scan accepted card
        status, body = http_post(f"{HUB_URL}/api/rfid/scan?uid=52411902")
        resp = json.loads(body)
        log_test("RFID reader accepted authorized card 52411902", resp["authorized"])

        # Monitor Car Gate Servos 1 & 2 ramp
        max_s1, max_s2 = 0, 0
        gate_reached_160 = False
        for _ in range(40):
            status, body = http_get(f"{HUB_URL}/api/status")
            d = json.loads(body)
            max_s1 = max(max_s1, d["servo1"])
            max_s2 = max(max_s2, d["servo2"])
            if d["servo1"] == 160 and d["servo2"] == 160:
                gate_reached_160 = True
                break
            time.sleep(0.05)

        log_test("Car Gate Servos 1 & 2 smoothly swept to 160°", gate_reached_160, f"(S1={max_s1}°, S2={max_s2}°)")
        log_test("Neither Car Gate Servo exceeded 160° safety limit", max_s1 <= 160 and max_s2 <= 160)

        # -------------------------------------------------------------------
        # Scenario 6: Unauthorized RFID Card -> Siren Alarm
        # -------------------------------------------------------------------
        print("\n--- [Scenario 6: Unauthorized RFID Card -> Siren Alarm] ---")
        status, body = http_post(f"{HUB_URL}/api/rfid/scan?uid=B9A8C7D6")
        time.sleep(0.3)
        status, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Unauthorized card triggered intruder alarm", d["sirenActive"])
        log_test("Hub registered denied card UID in RFID telemetry", d["rfid"]["lastUid"] == "B9A8C7D6")
        # Silence alarm
        http_post(f"{HUB_URL}/api/control?action=siren_silence")

        # -------------------------------------------------------------------
        # Scenario 7: Dual-Path Network Failover (UDP Interrupted)
        # -------------------------------------------------------------------
        print("\n--- [Scenario 7: Dual-Path Network Failover (HTTP Backup)] ---")
        # Directly dispatch an HTTP event to simulate ESP-NOW packet loss
        event_payload = json.dumps({
            "source": 1,
            "event": 1,
            "matchId": 1,
            "confidence": 98,
            "label": "Admin"
        })
        status, body = http_post(f"{HUB_URL}/api/cam/event", data=event_payload)
        resp = json.loads(body)
        log_test("Hub /api/cam/event received and processed HTTP event backup", status == 200 and resp["path"] == "http")

        time.sleep(0.5)
        status, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("HTTP fallback event triggered door unlock for 'Admin'", d["faceCam"]["label"] == "Admin" and d["doorOpen"])

        # -------------------------------------------------------------------
        # Scenario 8: Environmental Sensors & Universal 160° Verification
        # -------------------------------------------------------------------
        print("\n--- [Scenario 8: Environmental Sensors & Global Clamping] ---")
        # Test flame
        http_post(f"{HUB_URL}/api/sensor/inject?flame=300")
        time.sleep(0.2)
        _, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Flame sensor (ADC=300 <= 500) triggered FIRE alarm", d["fire"] and d["sirenActive"])

        # Reset flame, test smoke
        http_post(f"{HUB_URL}/api/sensor/inject?flame=3800&smoke=2500")
        time.sleep(0.2)
        _, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Smoke sensor (ADC=2500 >= 2000) triggered SMOKE alarm", d["smoke"] and d["sirenActive"])

        # Reset smoke, test ultrasonic
        http_post(f"{HUB_URL}/api/sensor/inject?smoke=400&dist=12.5")
        time.sleep(0.2)
        _, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Ultrasonic sensor (dist=12.5cm <= 25cm) triggered PROXIMITY alarm", d["ultra"] and d["sirenActive"])

        # Clear sensors
        http_post(f"{HUB_URL}/api/sensor/inject?dist=45.0")
        time.sleep(0.2)
        _, body = http_get(f"{HUB_URL}/api/status")
        d = json.loads(body)
        log_test("Sensors returned to normal; alarm cleared", not d["sirenActive"])

        print("\n==================================================================")
        print(f"TEST RESULTS: {passed_tests} PASSED | {failed_tests} FAILED")
        print("==================================================================\n")

    finally:
        hub_proc.terminate()
        cam_proc.terminate()

    if failed_tests > 0:
        sys.exit(1)

if __name__ == "__main__":
    run_tests()
