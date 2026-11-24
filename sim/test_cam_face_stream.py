#!/usr/bin/env python3
"""
End-to-End Integration Test for Temporary Stream + Laptop Face Recognition Pipeline.

Verifies:
- sim/hub_sim.py (Virtual Main Hub)
- sim/cam_face_stream_sim.py (Digital Twin of src/cam_face_stream/main.cpp)
- tools/laptop_face_bridge.py (Laptop MJPEG Stream Client & Feedback Bridge)
"""

import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from contextlib import closing

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

from laptop_face_bridge import EspCamStreamClient, LaptopFaceBridge

HUB_URL = "http://127.0.0.1:8080"
CAM_URL = "http://127.0.0.1:8081"

passed_tests = 0
failed_tests = 0


def log_test(name, ok, detail=""):
    global passed_tests, failed_tests
    if ok:
        passed_tests += 1
        print(f"  [PASS] {name} {detail}")
    else:
        failed_tests += 1
        print(f"  [FAIL] {name} - {detail}")


def http_get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.status, resp.read()


def http_post(url, data=None, timeout=3.0):
    body = data.encode("utf-8") if isinstance(data, str) else None
    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read()


def wait_for_server(url, name, retries=20):
    for _ in range(retries):
        try:
            st, _ = http_get(f"{url}/api/status", timeout=1.0)
            if st == 200:
                print(f"[INIT] {name} ready at {url}")
                return True
        except Exception:
            time.sleep(0.3)
    print(f"[INIT] ERROR: {name} failed to start at {url}")
    return False


def run_tests():
    print("\n==================================================================")
    print("   STREAM + REMOTE LAPTOP FACE RECOGNITION INTEGRATION SUITE")
    print("==================================================================\n")

    hub_proc = subprocess.Popen([sys.executable, os.path.join(REPO_ROOT, "sim", "hub_sim.py")])
    cam_proc = subprocess.Popen(
        [sys.executable, os.path.join(REPO_ROOT, "sim", "cam_face_stream_sim.py"), "--http-port", "8081", "--stream-port", "8082"]
    )

    try:
        if not wait_for_server(HUB_URL, "Virtual Hub"):
            sys.exit(1)
        if not wait_for_server(CAM_URL, "Virtual Stream Cam"):
            sys.exit(1)

        client = EspCamStreamClient("127.0.0.1", http_port=8081, stream_port=8082)

        # -------------------------------------------------------------------
        # Scenario 1: Dashboard HTML, Clean Stream, & Hub Proxy
        # -------------------------------------------------------------------
        print("\n--- [Scenario 1: Dashboard & Clean Raw MJPEG Stream] ---")
        with urllib.request.urlopen(f"{CAM_URL}/", timeout=3) as preview_response:
            log_test(
                "Camera root opens the raw MJPEG stream",
                preview_response.geturl().endswith(":8082/stream")
                and preview_response.headers.get_content_type() == "multipart/x-mixed-replace",
            )

        # Read real MJPEG frames through EspCamStreamClient
        got_frames = []
        for idx, jpg in enumerate(client.iter_mjpeg_frames()):
            got_frames.append(jpg)
            if idx >= 2:
                break
        log_test(
            "Laptop client reads valid JPEG frames from :8082/stream",
            len(got_frames) == 3 and all(f.startswith(b"\xff\xd8") and f.endswith(b"\xff\xd9") for f in got_frames),
            f"(bytes={len(got_frames[0])})",
        )

        st, preview = http_get(f"{HUB_URL}/api/cam/1/preview.jpg")
        log_test("Hub /api/cam/1/preview.jpg proxies valid JPEG", st == 200 and preview.startswith(b"\xff\xd8"))

        # -------------------------------------------------------------------
        # Scenario 2: Enrolled Sync & Pre-Recognition Alignment Feedback
        # -------------------------------------------------------------------
        print("\n--- [Scenario 2: Laptop Directory Sync & Alignment Feedback] ---")
        sync_resp = client.sync_enrolled([{"id": 0, "name": "Khaled"}, {"id": 1, "name": "Admin"}])
        log_test("Laptop synced enrolled faces to ESP dashboard", sync_resp.get("ok") and sync_resp.get("count") == 2)

        align_resp = client.send_result(
            result="aligning",
            face_detected=True,
            face_quality="TOO_FAR",
            box=[220, 120, 420, 340],
            inference_ms=28,
            laptop_model="InsightFace",
        )
        cam_st = client.get_status()
        log_test(
            "Alignment feedback updates HUD overlay without triggering Hub",
            align_resp.get("ok")
            and not align_resp.get("hubDispatched")
            and cam_st["laptopConnected"]
            and cam_st["laptopModel"] == "InsightFace"
            and cam_st["faceBox"] == [220, 120, 420, 340],
        )

        # -------------------------------------------------------------------
        # Scenario 3: Laptop RECOGNIZED -> ESP API -> Hub Door Open & Auto-Lock
        # -------------------------------------------------------------------
        print("\n--- [Scenario 3: Laptop RECOGNIZED -> ESP -> Hub Door Open] ---")
        current_verdict = {
            "result": "recognized",
            "matchId": 0,
            "name": "Khaled",
            "confidence": 0.96,
            "faceDetected": True,
            "faceQuality": "READY",
            "box": [180, 90, 460, 370],
            "inferenceMs": 35,
            "laptopModel": "InsightFace",
        }

        bridge = LaptopFaceBridge(client, lambda _jpg, _st: dict(current_verdict), laptop_model="InsightFace")
        bridge.enrolled = [{"id": 0, "name": "Khaled"}, {"id": 1, "name": "Admin"}]
        with bridge._jpg_lock:
            bridge._latest_jpg = got_frames[0]

        rec_resp = bridge.step_once()
        log_test(
            "ESP /api/recognition/result dispatched DOOR_OPEN to Hub",
            rec_resp is not None and rec_resp.get("hubDispatched") and rec_resp.get("hubAction") == "DOOR_OPEN",
        )

        door_reached_160 = False
        max_angle = 0
        hdata = {}
        for _ in range(45):
            _, hbody = http_get(f"{HUB_URL}/api/status")
            hdata = json.loads(hbody)
            max_angle = max(max_angle, hdata["door3"])
            if hdata["door3"] == 160 and hdata["doorOpen"]:
                door_reached_160 = True
                break
            time.sleep(0.05)
        log_test(
            "Hub opened Door Servo 3 to 160° on Laptop recognition verdict",
            door_reached_160 and hdata["faceCam"]["event"] == "RECOGNIZED" and hdata["faceCam"]["label"] == "Khaled",
            f"(max_angle={max_angle}°, label={hdata['faceCam']['label']})",
        )

        print("  Waiting 3s for Hub door auto-lock...")
        time.sleep(3.0)
        door_locked = False
        for _ in range(45):
            _, hbody = http_get(f"{HUB_URL}/api/status")
            hdata = json.loads(hbody)
            if hdata["door3"] == 0 and not hdata["doorOpen"]:
                door_locked = True
                break
            time.sleep(0.05)
        log_test("Hub auto-locked Door Servo 3 back to 0°", door_locked, f"(door3={hdata['door3']}°)")

        # -------------------------------------------------------------------
        # Scenario 4: Unknown Streak Confirmation -> Hub Intruder Siren Alarm
        # -------------------------------------------------------------------
        print("\n--- [Scenario 4: Unknown Streak Confirmation -> Hub Siren] ---")
        current_verdict = {
            "result": "unknown",
            "matchId": -1,
            "name": "Unknown",
            "confidence": 0.28,
            "faceDetected": True,
            "faceQuality": "READY",
            "box": [190, 95, 450, 365],
            "inferenceMs": 39,
            "laptopModel": "InsightFace",
        }

        u1 = bridge.step_once()
        log_test(
            "1st UNKNOWN stays in VERIFYING confirmation streak (no premature alarm)",
            u1["result"] == "VERIFYING" and not u1["hubDispatched"] and u1["unknownStreak"] == 1,
        )

        u2 = bridge.step_once()
        log_test(
            "2nd UNKNOWN confirms intruder and dispatches INTRUDER_ALARM to Hub",
            u2["result"] == "UNKNOWN" and u2["hubDispatched"] and u2["hubAction"] == "INTRUDER_ALARM",
        )

        time.sleep(0.3)
        _, hbody = http_get(f"{HUB_URL}/api/status")
        hdata = json.loads(hbody)
        log_test("Hub activated Intruder Siren Alarm", hdata["alarm"] and hdata["sirenActive"] and hdata["faceCam"]["event"] == "UNKNOWN")
        time.sleep(4.2)
        _, hbody = http_get(f"{HUB_URL}/api/status")
        hdata = json.loads(hbody)
        log_test("Hub siren cleared automatically after timeout", not hdata["alarm"] and not hdata["sirenActive"])

        # -------------------------------------------------------------------
        # Scenario 5: Dashboard Enrollment -> Laptop Capture -> Ack -> Hub
        # -------------------------------------------------------------------
        print("\n--- [Scenario 5: Dashboard Enroll -> Laptop Capture & Ack] ---")
        st, _ = http_post(f"{CAM_URL}/api/enroll?name=Nadia")
        cam_st = client.get_status()
        log_test(
            "Dashboard /api/enroll queued Laptop enrollment for Nadia",
            st == 200 and cam_st["enrollPending"] and cam_st["enrollTargetName"] == "Nadia",
        )

        current_verdict = {"enrollCaptured": True}
        time.sleep(2.6)  # Clear recognized cooldown before enroll ack
        ack_resp = bridge.step_once()
        cam_st = client.get_status()
        log_test(
            "Laptop completed enrollment via /api/enroll/ack and updated directory",
            ack_resp.get("ok")
            and ack_resp.get("hubDispatched")
            and not cam_st["enrollPending"]
            and any(e["name"] == "Nadia" for e in cam_st["enrolled"]),
        )

        # Wait out door cycle from enrollment
        time.sleep(3.5)

        # -------------------------------------------------------------------
        # Scenario 6: Dashboard Delete Sync & Resolution Change
        # -------------------------------------------------------------------
        print("\n--- [Scenario 6: Dashboard Delete Command Sync & Resolution Config] ---")
        st, _ = http_post(f"{CAM_URL}/api/delete?id=1")
        current_verdict = {"result": "no_face", "faceDetected": False}
        bridge.step_once()
        cam_st = client.get_status()
        log_test(
            "Dashboard delete synced to Laptop and acknowledged back cleanly",
            st == 200 and cam_st["pendingAction"] == "none" and not any(e["id"] == 1 for e in cam_st["enrolled"]),
        )

        client.configure_video("hvga", 15)
        cam_st = client.get_status()
        log_test(
            "Config resolution HVGA updated frameWidth=480, frameHeight=320",
            cam_st["frameWidth"] == 480 and cam_st["frameHeight"] == 320 and cam_st["cfg"]["jpegQuality"] == 15,
        )

        client.configure_video("qvga", 18)
        cam_st = client.get_status()
        log_test("Laptop can select the lower-bandwidth QVGA preset",
                 cam_st["frameWidth"] == 320 and cam_st["frameHeight"] == 240
                 and cam_st["cfg"]["jpegQuality"] == 18)

        hub_proc.terminate()
        hub_proc.wait()
        standalone = EspCamStreamClient("127.0.0.1", 8081, 8082, standalone=True)
        log_test("Standalone mode defaults to the hotspot IP",
                 EspCamStreamClient(standalone=True).cam_ip == "192.168.11.1")
        with closing(standalone.iter_mjpeg_frames()) as frames:
            jpeg = next(frames)
        result = standalone.send_result("recognized", match_id=0, name="Local Match",
                                        face_detected=True, box=[10, 20, 80, 90], force=True)
        status = standalone.get_status()
        log_test("Standalone stream and local recognition work with the hub stopped",
                 jpeg.startswith(b"\xff\xd8") and result["ok"] and not result["hubDispatched"]
                 and result["hubAction"] == "none"
                 and status["laptopConnected"] and status["lastMatchName"] == "Local Match"
                 and status["faceBox"] == [10, 20, 80, 90])
        enrollment = standalone.ack_enroll(True, 5, "Standalone Profile")
        log_test("Standalone enrollment updates the camera without a hub event",
                 enrollment["ok"] and not enrollment["hubDispatched"]
                 and any(e["id"] == 5 for e in standalone.get_status()["enrolled"]))

    finally:
        cam_proc.terminate()
        hub_proc.terminate()
        cam_proc.wait()
        hub_proc.wait()

    print("\n==================================================================")
    print(f"TEST RESULTS: {passed_tests} PASSED | {failed_tests} FAILED")
    print("==================================================================\n")
    sys.exit(0 if failed_tests == 0 else 1)


if __name__ == "__main__":
    run_tests()
