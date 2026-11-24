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
import multiprocessing
import queue
import re
import statistics
import sys
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

# Keep the direct script entry point usable from the face demo directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from camera_stream import IncompleteJpegPart, RESOLUTIONS, iter_jpegs, set_video_config

try:
    import cv2
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
PREVIEW_SIZE = (960, 720)
FRAME_MAX_AGE = 1.5
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
        locations = face_recognition.face_locations(frame_rgb, number_of_times_to_upsample=0)

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
            label = f"{display_name} ({conf_pct}%)"
        else:
            label = "Unknown face"

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        tag_x = max(0, min(left, frame_bgr.shape[1] - tw - 8))
        tag_y = max(top - 8, th + 6)
        cv2.rectangle(frame_bgr, (tag_x, tag_y - th - 5), (tag_x + tw + 8, tag_y + 3), color, -1)
        cv2.putText(frame_bgr, label, (tag_x + 4, tag_y - 2),
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


@dataclass(eq=False)
class CameraFrame:
    image: np.ndarray
    received_at: float
    connection: int


class EspStreamReader:
    """Non-blocking background thread that constantly reads the latest JPEG from the ESP32 MJPEG stream."""

    def __init__(self, stream_url: str):
        self.stream_url = stream_url
        self.latest_frame: CameraFrame | None = None
        self.frame_count = 0
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        connection = 0
        while not self.stopped.is_set():
            retry_delay = 1.0
            try:
                with CAMERA_HTTP.open(self.stream_url, timeout=4.0) as response:
                    connection += 1
                    for jpeg in iter_jpegs(response, skip_damaged=True):
                        if self.stopped.is_set():
                            break
                        try:
                            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        except cv2.error:
                            continue
                        if frame is not None:
                            with self.lock:
                                self.latest_frame = CameraFrame(frame, time.monotonic(), connection)
                                self.frame_count += 1
                            retry_delay = 0.2
            except IncompleteJpegPart:
                retry_delay = 0.2
            except (OSError, ValueError, http.client.HTTPException) as exc:
                if not self.stopped.is_set():
                    print(f"[esp-stream] Reconnecting ({exc})", file=sys.stderr)
            finally:
                with self.lock:
                    self.latest_frame = None
            self.stopped.wait(retry_delay)

    def snapshot(self) -> tuple[CameraFrame | None, int]:
        with self.lock:
            frame = self.latest_frame
            if frame is not None and time.monotonic() - frame.received_at > FRAME_MAX_AGE:
                frame = None
            return frame, self.frame_count

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


def sync_enrolled(base_url, known):
    enrolled = [{"id": idx, "name": name.replace("-", " ").title()} for idx, name in enumerate(known)]
    request = urllib.request.Request(
        f"{base_url}/api/laptop/enrolled",
        data=json.dumps({"enrolled": enrolled}).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with CAMERA_HTTP.open(request, timeout=1.5):
            pass
    except OSError:
        pass


def analyze_stream_frames(requests, replies, known, tolerance):
    """Run dlib outside the GUI process, even when its routines hold the GIL."""
    cv2.setNumThreads(1)
    while True:
        request = requests.get()
        if request is None:
            return
        image, updated_known = request
        if updated_known is not None:
            known = updated_known
        started = time.perf_counter()
        results = recognize(image, known, tolerance)
        replies.put((results, round((time.perf_counter() - started) * 1000)))


def draw_preview(size, sample, results, received_fps, preview_fps, inference_ms):
    width, height = size
    canvas = np.full((height, width, 3), (27, 23, 20), dtype=np.uint8)
    header = min(55, height // 4)
    footer = min(60, height // 4)
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_color = (233, 228, 224)
    muted = (185, 177, 169)
    cv2.line(canvas, (0, header - 1), (width, header - 1), (58, 50, 43))
    cv2.line(canvas, (0, height - footer), (width, height - footer), (58, 50, 43))
    if width >= 180 and header >= 24:
        status_color = (148, 224, 84) if sample is not None else (64, 179, 249)
        cv2.circle(canvas, (22, header // 2), 4, status_color, -1, cv2.LINE_AA)
        title = "Camera preview" if sample is not None else ("Waiting for camera" if width >= 240 else "Waiting")
        cv2.putText(canvas, title, (36, header // 2 + 6), font, 0.6, text_color, 1, cv2.LINE_AA)
    if sample is not None:
        image = sample.image
        source_h, source_w = image.shape[:2]
        scale = min(width / source_w, (height - header - footer) / source_h)
        video_w, video_h = max(1, round(source_w * scale)), max(1, round(source_h * scale))
        left = (width - video_w) // 2
        top = header + (height - header - footer - video_h) // 2
        video = canvas[top:top + video_h, left:left + video_w]
        interpolation = cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
        cv2.resize(image, (video_w, video_h), dst=video, interpolation=interpolation)
        scaled_results = [
            (round(y1 * video_h / source_h), round(x2 * video_w / source_w),
             round(y2 * video_h / source_h), round(x1 * video_w / source_w), name, distance)
            for y1, x2, y2, x1, name, distance in results
        ]
        annotate(video, scaled_results)
        info = f"{source_w}x{source_h}  |  RX {received_fps:.1f} fps  |  Preview {preview_fps:.1f} fps"
        if width >= 530 and inference_ms is not None:
            info += f"  |  AI {inference_ms} ms"
        if footer >= 24 and width >= 180:
            (text_width, _), _ = cv2.getTextSize(info, font, 0.51, 1)
            cv2.putText(canvas, info, (16, height - footer + 19), font,
                        0.51 * min(1, (width - 32) / text_width), text_color, 1, cv2.LINE_AA)
    if footer >= 48 and width >= 320:
        cv2.putText(canvas, "Space  Save frame     E  Enroll     Q  Quit",
                    (16, height - 13), font, 0.44, muted, 1, cv2.LINE_AA)
    return canvas


def queue_feedback(pool, base_url, results, inference_ms, previous):
    """Send a fresh verdict without making a busy HTTP request block the preview."""
    future, last_verdict, last_sent = previous
    if future is not None and not future.done():
        return previous
    now = time.monotonic()
    if results:
        match = next((face for face in results if face[4] != "Unknown"), results[0])
        top, right, bottom, left, name, distance = match
        verdict = "unknown" if name == "Unknown" else "recognized"
        interval = 0.4 if verdict == "unknown" else 0.35
        if verdict != last_verdict or now - last_sent >= interval:
            future = pool.submit(send_esp_feedback, base_url, verdict, name,
                                 distance if verdict == "recognized" else None,
                                 [int(left), int(top), int(right), int(bottom)], inference_ms)
            return future, verdict, now
    elif now - last_sent >= 2.5:
        future = pool.submit(send_esp_feedback, base_url, "no_face", "---", None)
        return future, "no_face", now
    return previous


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

    reader = EspStreamReader(stream_url)
    window = "Smart Home Security - ESP32 Face Recognition"
    cv2.setNumThreads(1)
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    cv2.resizeWindow(window, *PREVIEW_SIZE)
    cv2.imshow(window, draw_preview(PREVIEW_SIZE, None, [], 0, 0, None))

    context = multiprocessing.get_context("spawn")
    requests, replies = context.Queue(maxsize=1), context.Queue(maxsize=1)
    worker = context.Process(target=analyze_stream_frames, args=(requests, replies, known, tolerance))
    feedback_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera-feedback")
    feedback = (feedback_pool.submit(sync_enrolled, base_url, known), "none", 0.0)
    worker_known = known
    pending = None
    last_analyzed = None
    analysis = None
    last_render = None
    size = PREVIEW_SIZE
    measured_at = time.monotonic()
    received_fps = preview_fps = 0.0
    received_before = displayed = displayed_before = 0

    try:
        worker.start()
        while True:
            sample, received = reader.snapshot()
            now = time.monotonic()
            if now - measured_at >= 1.5:
                elapsed = now - measured_at
                received_fps = (received - received_before) / elapsed
                preview_fps = (displayed - displayed_before) / elapsed
                received_before, displayed_before = received, displayed
                measured_at = now

            if analysis is not None:
                source = analysis[0]
                if (sample is None or source.connection != sample.connection
                        or source.image.shape != sample.image.shape
                        or now - source.received_at > FRAME_MAX_AGE):
                    analysis = None

            if pending is not None:
                try:
                    results, inference_ms = replies.get_nowait()
                except queue.Empty:
                    pass
                else:
                    source, submitted_known = pending
                    pending = None
                    if (sample is not None and source.connection == sample.connection
                            and source.image.shape == sample.image.shape and submitted_known is known
                            and now - source.received_at <= FRAME_MAX_AGE):
                        analysis = source, results, inference_ms
                        feedback = queue_feedback(feedback_pool, base_url, results, inference_ms, feedback)

            if not worker.is_alive():
                print("[esp-stream] Face analysis worker exited", file=sys.stderr)
                break
            if pending is None and sample is not None and sample is not last_analyzed:
                # One frame in flight. On completion take the newest capture.
                requests.put_nowait((sample.image, known if known is not worker_known else None))
                pending = sample, known
                worker_known, last_analyzed = known, sample

            try:
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    break
                _, _, width, height = cv2.getWindowImageRect(window)
                if width > 0 and height > 0:
                    size = width, height
            except cv2.error:
                # Some HighGUI backends cannot report the drawable area.
                pass
            render_state = size, sample, analysis, received_fps, preview_fps
            if render_state != last_render:
                results = analysis[1] if analysis is not None else []
                inference_ms = analysis[2] if analysis is not None else None
                display = draw_preview(size, sample, results, received_fps, preview_fps, inference_ms)
                cv2.imshow(window, display)
                if sample is not None and (last_render is None or last_render[1] is not sample):
                    displayed += 1
                last_render = render_state

            key = cv2.waitKey(8) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" ") and sample is not None:
                _save_annotated(display, stem="esp-snap")
            if key in (ord("e"), ord("E")):
                if analysis is not None and analysis[1]:
                    source, results, _ = analysis
                    top, right, bottom, left, _, _ = results[0]
                    new_name = input("Enter identity name: ").strip()
                    if new_name and enroll_new_identity(source.image, (top, right, bottom, left), new_name):
                        known = load_known_faces()
                        analysis = None
                else:
                    print("[enroll] Look directly into the camera so a face is detected first.")

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        feedback_pool.shutdown(wait=False, cancel_futures=True)
        if worker.pid is not None:
            try:
                requests.put_nowait(None)
            except queue.Full:
                pass
            worker.join(timeout=0.5)
            if worker.is_alive():
                worker.terminate()
                worker.join()
        requests.cancel_join_thread()
        requests.close()
        replies.close()
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
    parser.add_argument("--resolution", choices=RESOLUTIONS,
                        help="Set the ESP stream resolution before opening the preview")
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
        try:
            set_video_config(CAMERA_HTTP, f"http://{cam_ip}:{args.http_port}", args.resolution)
        except (OSError, ValueError) as exc:
            parser.exit(1, f"Camera configuration failed: {exc}\n")
        run_esp(cam_ip, args.http_port, args.stream_port, known, args.tolerance)


if __name__ == "__main__":
    main()
