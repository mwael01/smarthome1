#!/usr/bin/env python3
"""Preview and benchmark the standalone ESP32-S3 camera over USB."""

import argparse
from collections import deque
import threading
import time

if __package__:
    from .camera_stream import RESOLUTIONS, RESOLUTION_DIMENSIONS
    from .serial_camera import (
        FrameParser, HEADER, UART_BAUD, UINT32_MASK, add_light_argument,
        iter_serial_frames, open_camera_port, validate_light,
    )
else:
    from camera_stream import RESOLUTIONS, RESOLUTION_DIMENSIONS
    from serial_camera import (
        FrameParser, HEADER, UART_BAUD, UINT32_MASK, add_light_argument,
        iter_serial_frames, open_camera_port, validate_light,
    )

try:
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Install pyserial with the Python interpreter used to run this tool.") from exc


class StreamStats:
    def __init__(self):
        self.frames = 0
        self.byte_count = 0
        self.missing = 0
        self.longest_gap = 0.0
        self.longest_capture_gap = 0.0
        self.first_at = self.last_at = 0.0
        self.last_capture_us = self.last_sequence = 0

    def record(self, frame, now):
        if self.frames:
            self.byte_count += len(frame.jpeg) + HEADER.size
            self.longest_gap = max(self.longest_gap, now - self.last_at)
            step = (frame.sequence - self.last_sequence) & UINT32_MASK
            if 0 < step < (1 << 31):
                self.missing += step - 1
                gap_us = (frame.capture_us - self.last_capture_us) & UINT32_MASK
                self.longest_capture_gap = max(self.longest_capture_gap, gap_us / 1e6)
        else:
            self.first_at = now
        self.frames += 1
        self.last_at = now
        self.last_sequence = frame.sequence
        self.last_capture_us = frame.capture_us

    def report(self, damaged):
        elapsed = self.last_at - self.first_at
        fps = (self.frames - 1) / elapsed if elapsed else 0.0
        rate = self.byte_count / elapsed / 1024 if elapsed else 0.0
        print(
            f"{self.frames} frames | {fps:.1f} received FPS | {rate:.1f} KiB/s | "
            f"longest arrival gap {self.longest_gap * 1000:.0f} ms | "
            f"longest capture gap between received frames {self.longest_capture_gap * 1000:.0f} ms | "
            f"{self.missing} skipped sequence numbers | {damaged} corrupt or incomplete packets",
            flush=True,
        )


def receive(port, args, stop, latest=None):
    parser = FrameParser()
    stats = StreamStats()
    sample_at = None
    sample_frames = 0
    fps = None
    frames = iter_serial_frames(port, args.resolution, args.jpeg_quality, stop, args.timeout, parser,
                                light=args.light)
    try:
        for frame in frames:
            now = time.monotonic()
            stats.record(frame, now)
            if latest is not None:
                if sample_at is None:
                    sample_at, sample_frames = now, stats.frames
                elif now - sample_at >= 1.0:
                    fps = (stats.frames - sample_frames) / (now - sample_at)
                    sample_at, sample_frames = now, stats.frames
                latest.append((frame, fps))
            if args.frames is not None and stats.frames >= args.frames:
                return
    finally:
        frames.close()
        stats.report(parser.damaged)


def preview(port, args, stop):
    import cv2
    import numpy as np

    latest = deque(maxlen=1)
    errors = []

    def read_frames():
        try:
            receive(port, args, stop, latest)
        except OSError as exc:
            errors.append(exc)

    window = "SmartHome Camera USB"
    width, height = RESOLUTION_DIMENSIONS[RESOLUTIONS[args.resolution]]
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(window, max(width, 640), max(width, 640) * height // width)
    reader = threading.Thread(target=read_frames)
    reader.start()
    displayed_fps = None
    try:
        while reader.is_alive() or latest:
            try:
                frame, fps = latest.popleft()
            except IndexError:
                pass
            else:
                image = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
                if image is not None and image.shape[:2] == (height, width):
                    cv2.imshow(window, image)
                    if fps is not None and fps != displayed_fps:
                        cv2.setWindowTitle(window, f"{window} | {width}×{height} | {fps:.1f} received FPS")
                        displayed_fps = fps
            key = cv2.waitKey(1)
            if key in (27, ord("q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if not latest:
                stop.wait(0.004)
    finally:
        stop.set()
        reader.join()
        cv2.destroyAllWindows()
    if errors:
        raise errors[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true", help="show available serial connections")
    parser.add_argument("--port", help="serial port, such as COM5, /dev/ttyACM0 or /dev/ttyUSB0")
    parser.add_argument("--uart", action="store_true", help="use the CH340 adapter")
    parser.add_argument("--baud", type=int, default=UART_BAUD, help="UART baud rate (default: 2000000)")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="hvga")
    parser.add_argument("--jpeg-quality", type=int, default=16, help="10–63, lower is higher quality")
    parser.add_argument("--no-gui", action="store_true", help="benchmark reception without decoding")
    parser.add_argument("--frames", type=int, help="stop after this many frames (default: 300 with --no-gui)")
    parser.add_argument("--timeout", type=float, default=12, help="seconds to wait for a frame")
    add_light_argument(parser)
    args = parser.parse_args()
    if args.list_ports:
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device}\t{port.description}\t{port.hwid}")
        if not ports:
            print("No serial ports found.")
        return 0
    if not args.port:
        parser.error("--port is required unless --list-ports is used")
    validate_light(parser, args.light)
    if args.uart and args.baud <= 0:
        parser.error("--baud must be positive")
    if not 10 <= args.jpeg_quality <= 63:
        parser.error("--jpeg-quality must be between 10 and 63")
    if args.frames is not None and args.frames <= 0:
        parser.error("--frames must be positive")
    if not 0 < args.timeout < float("inf"):
        parser.error("--timeout must be positive and finite")
    if args.no_gui and args.frames is None:
        args.frames = 300

    try:
        with open_camera_port(args.port, uart=args.uart, baud=args.baud) as port:
            stop = threading.Event()
            if args.no_gui:
                receive(port, args, stop)
            else:
                preview(port, args, stop)
    except KeyboardInterrupt:
        return 130
    except (OSError, ImportError) as exc:
        parser.exit(1, f"USB camera: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
