#!/usr/bin/env python3
"""Measure JPEG delivery from the standalone ESP32-S3 camera."""

import argparse
import http.client
import sys
import time
import urllib.parse
import urllib.request

if __package__:
    from .camera_stream import RESOLUTIONS, iter_jpegs, set_video_config
else:
    from camera_stream import RESOLUTIONS, iter_jpegs, set_video_config


def measure(url, frame_count, resolution=None, quality=None, http_port=80):
    # A direct hotspot link should work even when the laptop has a web proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    if resolution is not None or quality is not None:
        host = urllib.parse.urlsplit(url).hostname
        set_video_config(opener, f"http://{host}:{http_port}", resolution, quality)
    started = time.monotonic()
    with opener.open(url, timeout=5) as stream:
        payload_bytes = 0
        longest_gap = 0.0
        frames = iter_jpegs(stream)
        for frame in range(frame_count):
            jpeg = next(frames, None)
            if jpeg is None:
                raise ValueError(f"stream ended after {frame} of {frame_count} frames")
            size = len(jpeg)
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
    parser.add_argument("--resolution", choices=RESOLUTIONS)
    parser.add_argument("--jpeg-quality", type=int, metavar="8–30")
    parser.add_argument("--http-port", type=int, default=80, help="Camera configuration port")
    args = parser.parse_args()
    if args.jpeg_quality is not None and not 8 <= args.jpeg_quality <= 30:
        parser.error("--jpeg-quality must be between 8 and 30")
    if args.frames < 2:
        parser.error("--frames must be at least 2")
    try:
        measure(args.url, args.frames, args.resolution, args.jpeg_quality, args.http_port)
    except (OSError, ValueError, http.client.HTTPException) as exc:
        print(f"Stream test failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
