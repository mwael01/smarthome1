"""HTTP video settings and multipart JPEG framing shared by the laptop tools."""

import http.client
import json
import time
import urllib.parse
import urllib.request

RESOLUTIONS = {"qvga": 0, "hvga": 1, "vga": 2}
RESOLUTION_DIMENSIONS = ((320, 240), (480, 320), (640, 480))


class IncompleteJpegPart(ConnectionError):
    """The connection ended before the full JPEG payload arrived."""


def set_video_config(opener, base_url, resolution=None, quality=None):
    params = {}
    if resolution is not None:
        params["res"] = RESOLUTIONS[resolution]
    if quality is not None:
        params["qual"] = quality
    if params:
        query = urllib.parse.urlencode(params)
        request = urllib.request.Request(f"{base_url}/api/config?{query}", method="POST")
        with opener.open(request, timeout=3) as response:
            if not json.load(response).get("ok"):
                raise ValueError("camera rejected the video settings")
    if resolution is not None:
        expected = RESOLUTION_DIMENSIONS[RESOLUTIONS[resolution]]
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            with opener.open(f"{base_url}/api/status", timeout=2) as response:
                status = json.load(response)
            dimensions = status.get("frameWidth"), status.get("frameHeight")
            if status.get("cameraReady") and dimensions == expected:
                return
            time.sleep(0.15)
        raise ValueError(f"camera did not report {expected[0]}×{expected[1]} frames")


def iter_jpegs(response, *, skip_damaged=False):
    boundary = response.headers.get_param("boundary")
    if response.headers.get_content_type() != "multipart/x-mixed-replace" or not boundary:
        raise ValueError("expected a multipart MJPEG stream")
    boundary = boundary.encode("ascii")
    delimiter = b"--" + boundary
    delimiters = (delimiter, boundary) if boundary.startswith(b"--") else (delimiter,)
    while True:
        line = response.readline(256)
        if line in (b"\r\n", b"\n"):
            line = response.readline(256)
        line = line.rstrip(b"\r\n")
        if not line or any(line == marker + b"--" for marker in delimiters):
            return
        # Older camera builds included the delimiter prefix in the header value.
        if line not in delimiters:
            raise ValueError("missing multipart frame boundary")
        headers = http.client.parse_headers(response)
        size = int(headers.get("Content-Length", 0))
        if headers.get_content_type() != "image/jpeg" or not 4 <= size <= 2**20:
            raise ValueError("invalid JPEG part headers")
        try:
            jpeg = response.read(size)
        except http.client.IncompleteRead as exc:
            raise IncompleteJpegPart(
                f"MJPEG connection ended mid-frame ({len(exc.partial)}/{size} bytes)"
            ) from exc
        if len(jpeg) != size:
            raise IncompleteJpegPart(f"MJPEG connection ended mid-frame ({len(jpeg)}/{size} bytes)")
        if jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
            if skip_damaged:
                continue
            raise ValueError("damaged JPEG part")
        # A complete payload can be decoded before the separator arrives.
        yield jpeg
