#!/usr/bin/env python3
"""Measure JPEG delivery from the standalone ESP32-S3 camera."""

import argparse
import http.client
import sys
import time
import urllib.request


def measure(url, frame_count):
    # A direct hotspot link should work even when the laptop has a web proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    started = time.monotonic()
    with opener.open(url, timeout=5) as stream:
        boundary = stream.headers.get_param("boundary")
        if stream.headers.get_content_type() != "multipart/x-mixed-replace" or not boundary:
            raise ValueError("expected a multipart MJPEG stream")
        delimiter = b"--" + boundary.encode("ascii")
        payload_bytes = 0
        longest_gap = 0.0
        for frame in range(frame_count):
            if stream.readline(256).rstrip(b"\r\n") != delimiter:
                raise ValueError(f"missing multipart boundary at frame {frame + 1}")
            headers = http.client.parse_headers(stream)
            size = int(headers.get("Content-Length", 0))
            if headers.get_content_type() != "image/jpeg" or not 4 <= size <= 2**20:
                raise ValueError(f"invalid JPEG headers at frame {frame + 1}")
            jpeg = stream.read(size)
            if (len(jpeg) != size or jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9"
                    or stream.read(2) != b"\r\n"):
                raise ValueError(f"incomplete JPEG at frame {frame + 1}")
            received = time.monotonic()
            if frame == 0:
                first_frame = received
            else:
                payload_bytes += size
                longest_gap = max(longest_gap, received - previous_frame)
            previous_frame = received

    elapsed = previous_frame - first_frame
    print(f"First frame: {(first_frame - started) * 1000:.0f} ms")
    print(f"{frame_count} valid JPEGs | {(frame_count - 1) / elapsed:.1f} FPS | "
          f"{payload_bytes * 8 / elapsed / 1e6:.2f} Mbit/s JPEG payload")
    print(f"Longest gap between frames: {longest_gap * 1000:.0f} ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.11.1:81/stream")
    parser.add_argument("--frames", type=int, default=120)
    args = parser.parse_args()
    if args.frames < 2:
        parser.error("--frames must be at least 2")
    try:
        measure(args.url, args.frames)
    except (OSError, ValueError, http.client.HTTPException) as exc:
        print(f"Stream test failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
