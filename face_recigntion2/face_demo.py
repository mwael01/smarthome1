"""Face-recognition demo and ESP32-CAM stream bridge for the Smart Home Security System.

Usage modes:

    python face_demo.py                          # connect to ESP32-CAM stream (default: 192.168.4.101)
    python face_demo.py --esp 192.168.4.101      # connect to custom ESP32-CAM IP
    python face_demo.py --camera                 # local webcam -> live recognition
    python face_demo.py --image path/to/pic.jpg  # recognize from a picture file
    python face_demo.py --list                   # show enrolled identities

Enrollment is "drop photos in a folder": put your photos in `known_faces/`
named `<identity>-<number>.<ext>`, e.g. `khaled-naser-1.jpeg`, `khaled-niazy-1.jpeg`.
Every photo sharing the same `<identity>` prefix (everything before the last
`-<number>`) is grouped into one enrolled identity. More photos per identity =
more robust matching. You can also press 'e' while streaming to enroll on the spot!

When streaming from the ESP32-CAM:
- Recognized enrolled face -> sends `POST /api/recognition/result` -> unlocks Face Door Servo (GPIO26) on Main Hub.
- Unknown intruder face     -> sends `POST /api/recognition/result` -> triggers 4-second Intruder Siren on Main Hub.
"""

from __future__ import annotations

import argparse
import http.client
import json
import re
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

# Keep the direct script entry point usable from the face demo directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from camera_stream import IncompleteJpegPart, iter_jpegs

try:
    import cv2
    import dlib
    import face_recognition
    import numpy as np
except ImportError as exc:  # friendly message for a not-yet-set-up environment
    print(
        f"[setup] Missing dependency: {exc.name!r}. "
        "Run the project setup (or `pip install -r requirements.txt`) first.",
        file=sys.stderr,
    )
    sys.exit(1)

KNOWN_FACES_DIR = Path(__file__).resolve().parent / "known_faces"
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
CAMERA_HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))
# <identity>-<number>.<jpg|jpeg|png>  — greedy so a hyphenated name still works
_IDENTITY_RE = re.compile(r"^(?P<name>.+)-\d+\.(?:jpe?g|png)$", re.IGNORECASE)
_SUPPORTED = ("*.jpg", "*.jpeg", "*.png")
# Rejection margin: the top match must beat the runner-up by this much, else the
# face is reported Unknown (a near-tie means "ambiguous, don't guess").
MARGIN = 0.06


def load_known_faces() -> dict[str, list]:
    """Return {identity: [embedding, ...]} from photos in known_faces/."""
    if not KNOWN_FACES_DIR.is_dir():
        KNOWN_FACES_DIR.mkdir(parents=True, exist_ok=True)
        return {}

    known: dict[str, list] = {}
    for ext in _SUPPORTED:
        for path in sorted(KNOWN_FACES_DIR.glob(ext)):
            match = _IDENTITY_RE.match(path.name)
            if not match:
                print(f"[warn] skipping {path.name} (expected <name>-<n>.jpg/png)")
                continue
            try:
                image = face_recognition.load_image_file(str(path))
                encodings = face_recognition.face_encodings(image)
                if not encodings:
                    print(f"[warn] no face found in {path.name}, skipping")
                    continue
                known.setdefault(match.group("name").lower(), []).append(encodings[0])
            except Exception as e:
                print(f"[warn] could not process {path.name}: {e}")

    return known


def recognize(frame_bgr, known: dict[str, list], tolerance: float) -> list[tuple]:
    """Return [(top, right, bottom, left, label, distance), ...] for each face.

    A face is labeled with its nearest enrolled identity only when (a) that
    identity's median distance is within `tolerance`, and (b) it is at least
    `MARGIN` closer than the runner-up. Otherwise it is "Unknown" — a near-tie
    between two identities is treated as ambiguous, never guessed.
    """
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    locations = face_recognition.face_locations(frame_rgb, number_of_times_to_upsample=1)
    if not locations:
        detector = dlib.get_frontal_face_detector()
        rects = detector(frame_rgb, 0)
        locations = [(r.top(), r.right(), r.bottom(), r.left()) for r in rects]

    if not locations:
        return []

    encodings = face_recognition.face_encodings(frame_rgb, locations)

    results: list[tuple] = []
    for (top, right, bottom, left), encoding in zip(locations, encodings):
        # (median distance, identity) ranked nearest-first
        ranked = sorted(
            (
                statistics.median(face_recognition.face_distance(refs, encoding)),
                identity,
            )
            for identity, refs in known.items()
        )
        name, distance = "Unknown", None
        if ranked:
            best, identity = ranked[0]
            distance = float(best)
            clear_winner = len(ranked) == 1 or best + MARGIN < ranked[1][0]
            if distance <= tolerance and clear_winner:
                name = identity
        results.append((top, right, bottom, left, name, distance))

    return results


def annotate(frame_bgr, results: list[tuple]) -> None:
    """Draw a box + label around each recognized face."""
    for top, right, bottom, left, name, distance in results:
        is_known = (name != "Unknown")
        color = (0, 220, 0) if is_known else (0, 0, 230)
        cv2.rectangle(frame_bgr, (left, top), (right, bottom), color, 2)

        display_name = name.replace("-", " ").title() if is_known else "Unknown"
        if is_known and distance is not None:
            conf_pct = int(max(0.0, min(1.0, 1.0 - (distance / 0.6) * 0.5)) * 100)
            label = f"{display_name} ({conf_pct}%) [UNLOCK]"
        else:
            label = "Unknown Intruder"

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        tag_y = max(top - 8, th + 36)
        cv2.rectangle(frame_bgr, (left, tag_y - th - 5), (left + tw + 8, tag_y + 3), color, -1)
        cv2.putText(frame_bgr, label, (left + 4, tag_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0) if is_known else (255, 255, 255), 1, cv2.LINE_AA)


def _save_annotated(frame_bgr, stem: str) -> Path | None:
    """Write the annotated frame to output/<stem>-<UTC timestamp>.jpg."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f")
    out = OUTPUT_DIR / f"{stem}-{stamp}.jpg"
    if not cv2.imwrite(str(out), frame_bgr):
        print(f"[error] could not write {out}", file=sys.stderr)
        return None
    print(f"[info] annotated frame saved to {out}")
    return out


def show_result(frame_bgr, results: list[tuple]) -> None:
    if not results:
        print("[info] no face detected in this frame.")
    for _, _, _, _, name, distance in results:
        display_name = name.replace("-", " ").title() if name != "Unknown" else "Unknown"
        if name == "Unknown":
            print(f"  result: Unknown")
        else:
            print(f"  result: {display_name} (distance {distance:.2f})")

    _save_annotated(frame_bgr, stem="result")
    cv2.imshow("Face Recognition", frame_bgr)
    print("[info] press any key to close the window")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_image(path: str, known: dict[str, list], tolerance: float) -> None:
    frame = cv2.imread(path)
    if frame is None:
        print(f"[error] could not read image: {path}", file=sys.stderr)
        sys.exit(1)
    results = recognize(frame, known, tolerance)
    show_result(frame, results)


def run_camera(known: dict[str, list], tolerance: float) -> None:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[error] could not open webcam (index 0).", file=sys.stderr)
        sys.exit(1)

    print("[camera] live recognition — Q to quit, SPACE to save a capture.")
    cv2.namedWindow("Face Recognition", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Face Recognition", 800, 600)
    detect_every = 2           # recognize every Nth frame; boxes persist in between
    results: list[tuple] = []  # last detection result, redrawn on every frame
    frame_i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[error] failed to read a frame from the camera.", file=sys.stderr)
            break

        if frame_i % detect_every == 0:
            results = recognize(frame, known, tolerance)
        annotate(frame, results)  # draw latest boxes + labels on every frame

        cv2.imshow("Face Recognition", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):  # SPACE = save the current annotated frame
            _save_annotated(frame, stem="snap")
        frame_i += 1

    cap.release()
    cv2.destroyAllWindows()


class EspStreamReader:
    """Non-blocking background thread that constantly reads the latest JPEG from the ESP32 MJPEG stream."""

    def __init__(self, stream_url: str):
        self.stream_url = stream_url
        self.latest_frame: np.ndarray | None = None
        self.latest_frame_at = 0.0
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.connected = False
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while not self.stopped.is_set():
            retry_delay = 1.0
            try:
                with CAMERA_HTTP.open(self.stream_url, timeout=4.0) as response:
                    for jpeg in iter_jpegs(response, skip_damaged=True):
                        if self.stopped.is_set():
                            break
                        try:
                            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        except cv2.error:
                            continue
                        if frame is not None:
                            with self.lock:
                                self.latest_frame = frame
                                self.latest_frame_at = time.monotonic()
                                self.connected = True
                            retry_delay = 0.2
            except IncompleteJpegPart:
                retry_delay = 0.2
            except (OSError, ValueError, http.client.HTTPException) as exc:
                if not self.stopped.is_set():
                    print(f"[esp-stream] Reconnecting ({exc})", file=sys.stderr)
            finally:
                with self.lock:
                    self.latest_frame = None
                    self.connected = False
            self.stopped.wait(retry_delay)

    def get_frame(self) -> np.ndarray | None:
        with self.lock:
            if time.monotonic() - self.latest_frame_at <= 1.5:
                return self.latest_frame
            return None

    def stop(self):
        self.stopped.set()


def send_esp_feedback(
    base_url: str,
    result: str,
    name: str,
    distance: float | None,
    box: list[int] | None = None,
    inference_ms: int = 0,
) -> None:
    """Send recognition result to ESP32-CAM (POST /api/recognition/result) to trigger Hub door unlock or alarm."""
    conf = 0.0
    if distance is not None:
        conf = max(0.0, min(1.0, 1.0 - (distance / 0.6) * 0.5))

    clean_name = name.replace("-", " ").title() if name != "Unknown" else "Unknown"
    payload = {
        "result": "recognized" if result == "recognized" else ("unknown" if result == "unknown" else "no_face"),
        "name": clean_name,
        "confidence": float(conf),
        "faceDetected": result in ("recognized", "unknown"),
        "faceQuality": "READY" if result in ("recognized", "unknown") else "NO_FACE",
        "box": box or [0, 0, 0, 0],
        "inferenceMs": int(inference_ms),
        "laptopModel": "FaceDemo-dlib",
        "triggerHub": True,
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{base_url}/api/recognition/result",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with CAMERA_HTTP.open(req, timeout=1.2) as resp:
            pass
    except Exception:
        pass


def enroll_new_identity(frame_bgr: np.ndarray, box: tuple | list, name: str) -> bool:
    """Crop face and save as known_faces/<identity>-<next_index>.jpg."""
    clean_id = re.sub(r"[^a-zA-Z0-9_-]", "", name.strip().lower().replace(" ", "-"))
    if not clean_id:
        clean_id = "user"

    # Find highest existing index for this identity
    existing_indices = []
    for ext in _SUPPORTED:
        for path in KNOWN_FACES_DIR.glob(ext):
            m = _IDENTITY_RE.match(path.name)
            if m and m.group("name").lower() == clean_id:
                # Extract number
                num_match = re.search(r"-(\d+)\.", path.name)
                if num_match:
                    existing_indices.append(int(num_match.group(1)))

    next_idx = max(existing_indices, default=0) + 1
    out_file = KNOWN_FACES_DIR / f"{clean_id}-{next_idx}.jpg"

    top, right, bottom, left = box
    h, w = frame_bgr.shape[:2]
    pad_x = int((right - left) * 0.25)
    pad_y = int((bottom - top) * 0.25)
    crop = frame_bgr[max(0, top - pad_y):min(h, bottom + pad_y),
                     max(0, left - pad_x):min(w, right + pad_x)]

    save_img = crop if crop.size > 0 else frame_bgr
    if cv2.imwrite(str(out_file), save_img):
        print(f"[enroll] Saved reference photo to {out_file}")
        return True
    return False


def run_esp(
    cam_ip: str,
    http_port: int,
    stream_port: int,
    known: dict[str, list],
    tolerance: float,
) -> None:
    base_url = f"http://{cam_ip}:{http_port}"
    stream_url = f"http://{cam_ip}:{stream_port}/stream"

    print("\n" + "=" * 70)
    print(" [SMART HOME] ESP32-CAM Face Recognition Bridge Online")
    print(f" - Video Stream:   {stream_url}")
    print(f" - Feedback API:   {base_url}/api/recognition/result")
    print(f" - Enrolled Faces: {len(known)} identities ({sum(len(v) for v in known.values())} photos)")
    for ident, encs in known.items():
        print(f"   * {ident.replace('-', ' ').title()}: {len(encs)} photo(s)")
    print(f" - Controls:       SPACE = save snapshot, E = enroll face, Q = quit")
    print("=" * 70 + "\n")

    # Initial sync with ESP32-CAM enrolled list
    try:
        enrolled_sync = [
            {"id": i, "name": name.replace("-", " ").title()}
            for i, name in enumerate(known.keys())
        ]
        sync_data = json.dumps({"enrolled": enrolled_sync}).encode("utf-8")
        sync_req = urllib.request.Request(
            f"{base_url}/api/laptop/enrolled",
            data=sync_data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with CAMERA_HTTP.open(sync_req, timeout=1.5) as resp:
            pass
    except Exception:
        pass

    reader = EspStreamReader(stream_url)
    window_name = "Smart Home Security — ESP32 Face Recognition"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)

    results: list[tuple] = []
    detect_every = 2
    frame_i = 0
    fps_frames = 0
    last_frame = None
    infer_ms = 0
    fps_timer = time.time()
    fps = 0.0
    last_feedback_time = 0.0
    last_verdict_sent = "NONE"

    try:
        while True:
            frame = reader.get_frame()
            if frame is None:
                last_frame = None
                results = []
                # Show waiting screen
                wait_screen = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(wait_screen, "Connecting to ESP32-CAM...", (80, 220),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(wait_screen, f"Target: {stream_url}", (80, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                network = "SmartHome-FaceCam" if cam_ip == "192.168.11.1" else "SmartHome-Net"
                cv2.putText(wait_screen, f"Connect the laptop to {network}", (80, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (150, 150, 150), 1)
                cv2.imshow(window_name, wait_screen)
                if (cv2.waitKey(30) & 0xFF) in (ord("q"), 27):
                    break
                continue

            now = time.time()
            if now - fps_timer >= 1.0:
                fps = fps_frames / (now - fps_timer)
                fps_frames = 0
                fps_timer = now

            fresh_frame = frame is not last_frame
            analyzed_frame = fresh_frame and (
                last_frame is None or frame.shape != last_frame.shape or frame_i % detect_every == 0
            )
            if analyzed_frame:
                t0 = time.perf_counter()
                results = recognize(frame, known, tolerance)
                infer_ms = int((time.perf_counter() - t0) * 1000.0)
            if fresh_frame:
                last_frame = frame
                frame_i += 1
                fps_frames += 1

            # Draw recognition boxes and labels
            display_frame = frame.copy()
            annotate(display_frame, results)

            # Draw HUD Overlays
            h, w = display_frame.shape[:2]
            cv2.rectangle(display_frame, (0, 0), (w, 36), (20, 20, 20), -1)
            hud_info = (f"ESP32-CAM: {cam_ip} | {w}x{h} | {fps:.1f} FPS | "
                        f"Infer: {infer_ms}ms | Enrolled: {len(known)}")
            cv2.putText(display_frame, hud_info, (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (230, 230, 230), 1, cv2.LINE_AA)

            cv2.rectangle(display_frame, (0, h - 26), (w, h), (20, 20, 20), -1)
            cv2.putText(display_frame, "[SPACE] Save Photo  |  [E] Enroll Face  |  [Q] Quit",
                        (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 230), 1, cv2.LINE_AA)

            # Determine decision and transmit to ESP32-CAM
            if analyzed_frame and results:
                # Prioritize any authorized match
                matched = next((r for r in results if r[4] != "Unknown"), None)
                if matched:
                    top, right, bottom, left, name, distance = matched
                    box = [int(left), int(top), int(right), int(bottom)]
                    if now - last_feedback_time >= 0.35 or last_verdict_sent != "recognized":
                        send_esp_feedback(base_url, "recognized", name, distance, box, infer_ms)
                        last_feedback_time = now
                        last_verdict_sent = "recognized"
                        clean_disp = name.replace("-", " ").title()
                        print(f"[ACCESS GRANTED] Recognized: {clean_disp} (distance {distance:.2f}) -> Hub Door Servo 3 Opened!")
                else:
                    # All detected faces are Unknown
                    top, right, bottom, left, _, _ = results[0]
                    box = [int(left), int(top), int(right), int(bottom)]
                    if now - last_feedback_time >= 0.40 or last_verdict_sent != "unknown":
                        send_esp_feedback(base_url, "unknown", "Unknown", None, box, infer_ms)
                        last_feedback_time = now
                        last_verdict_sent = "unknown"
                        print("[ACCESS DENIED] Unknown Person Detected -> Hub Intruder Siren Armed")
            elif analyzed_frame:
                # No face in view - send idle keepalive every 2.5s
                if now - last_feedback_time >= 2.5:
                    send_esp_feedback(base_url, "no_face", "---", None, None, 0)
                    last_feedback_time = now
                    last_verdict_sent = "no_face"

            cv2.imshow(window_name, display_frame)

            key = cv2.waitKey(1 if fresh_frame else 10) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                _save_annotated(display_frame, stem="esp-snap")
            elif key in (ord("e"), ord("E")):
                if results:
                    top, right, bottom, left, cur_name, _ = results[0]
                    box = (top, right, bottom, left)
                    print("\n--- Instant Face Enrollment ---")
                    new_name = input("Enter identity name (e.g. pola-lazem): ").strip()
                    if new_name:
                        if enroll_new_identity(frame, box, new_name):
                            known = load_known_faces()
                            print(f"[enroll] Enrolled! {len(known)} active identities in database.\n")
                else:
                    print("[enroll] Look directly into the camera so a face is detected first.")

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cv2.destroyAllWindows()
        print("\n[esp-stream] Stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Smart Home face-recognition demo & ESP32-CAM stream bridge")
    parser.add_argument("--esp", nargs="?", const="192.168.4.101", default=None,
                        help="stream from ESP32-CAM IP (default: 192.168.4.101); live recognition + Hub door unlock")
    parser.add_argument("--camera", action="store_true",
                        help="open local webcam; live recognition, SPACE to save, Q to quit")
    parser.add_argument("--image", help="path to a picture file to recognize")
    parser.add_argument("--list", action="store_true", help="list enrolled identities")
    parser.add_argument("--stream-port", type=int, default=81,
                        help="ESP32-CAM MJPEG stream port (default: 81)")
    parser.add_argument("--http-port", type=int, default=80,
                        help="ESP32-CAM HTTP control port (default: 80)")
    parser.add_argument("--tolerance", type=float, default=0.6,
                        help="match distance threshold (default 0.6, lower = stricter)")
    args = parser.parse_args()

    known = load_known_faces()

    if args.list:
        if not known:
            print(f"[info] no enrolled faces in {KNOWN_FACES_DIR}/. Add photos like 'khaled-naser-1.jpeg'.")
            return
        print(f"\nEnrolled Identities ({len(known)} total):")
        for identity, encodings in known.items():
            print(f"  * {identity.replace('-', ' ').title()}: {len(encodings)} photo(s)")
        return

    if not known:
        print(f"[warn] no enrolled faces in {KNOWN_FACES_DIR}/ — recognition will report everything as Unknown.")
        print("[hint] add photos named '<name>-<n>.jpg' (e.g. khaled-naser-1.jpeg).")

    if args.image:
        run_image(args.image, known, args.tolerance)
    elif args.camera:
        run_camera(known, args.tolerance)
    else:
        # Default mode: connect to ESP32-CAM stream
        cam_ip = args.esp if args.esp else "192.168.4.101"
        run_esp(cam_ip, args.http_port, args.stream_port, known, args.tolerance)


if __name__ == "__main__":
    main()
