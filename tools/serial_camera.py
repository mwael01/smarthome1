"""Framed JPEG protocol shared by native USB, CH340 UART and the face bridge."""

from dataclasses import dataclass
import struct
import time
import zlib

try:
    from serial import Serial
except ImportError as exc:
    raise SystemExit("Install pyserial with the Python interpreter used to run this tool.") from exc

if __package__:
    from .camera_stream import RESOLUTIONS, RESOLUTION_DIMENSIONS
else:
    from camera_stream import RESOLUTIONS, RESOLUTION_DIMENSIONS


MAGIC = b"\xff\xff\x00USBC1"
HEADER = struct.Struct("<8sIIIHHI")
MAX_JPEG_SIZE = 128 * 1024
READ_SIZE = 16 * 1024
UINT32_MASK = (1 << 32) - 1
UART_BAUD = 2_000_000


@dataclass
class Frame:
    sequence: int
    capture_us: int
    dimensions: tuple
    jpeg: bytes


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.damaged = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while self.buffer:
            start = self.buffer.find(MAGIC)
            if start == -1:
                del self.buffer[: max(0, len(self.buffer) - len(MAGIC) + 1)]
                return
            del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                return
            _, size, sequence, capture_us, width, height, checksum = HEADER.unpack_from(self.buffer)
            dimensions = (width, height)
            if not 4 <= size <= MAX_JPEG_SIZE or dimensions not in RESOLUTION_DIMENSIONS:
                del self.buffer[:len(MAGIC)]
                self.damaged += 1
                continue

            end = HEADER.size + size
            if len(self.buffer) >= end:
                jpeg = bytes(self.buffer[HEADER.size:end])
                if (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")
                        and zlib.crc32(jpeg, zlib.crc32(self.buffer[:HEADER.size - 4])) == checksum):
                    del self.buffer[:end]
                    yield Frame(sequence, capture_us, dimensions, jpeg)
                    continue
            # A new packet can begin before a stalled USB write's declared end.
            next_start = self.buffer.find(MAGIC, len(MAGIC))
            if next_start == -1 and len(self.buffer) < end:
                return
            del self.buffer[:next_start if next_start != -1 else len(MAGIC)]
            self.damaged += 1


def open_camera_port(name, *, uart=False, baud=UART_BAUD):
    port = Serial(baudrate=baud if uart else 115200, timeout=0.05, write_timeout=0.5)
    # CH340 programmers may wire DTR/RTS to boot/reset.
    port.dtr = port.rts = not uart
    port.port = name
    try:
        port.open()
        port.reset_input_buffer()
    except OSError:
        port.close()
        raise
    return port


def add_light_argument(parser):
    parser.add_argument("--light", nargs=2, type=int, metavar=("GPIO", "LEVEL"),
                        help="white NeoPixel on GPIO38 or GPIO48 at level 0–255")


def validate_light(parser, light):
    if light is not None and (light[0] not in (38, 48) or not 0 <= light[1] <= 255):
        parser.error("--light requires GPIO 38 or 48 and a level from 0 to 255")


def iter_serial_frames(port, resolution, quality, stop, timeout, parser, *, light=None):
    dimensions = RESOLUTION_DIMENSIONS[RESOLUTIONS[resolution]]
    pixels = f" {light[0]} {light[1]}" if light is not None else ""
    command = f"\nSTART {resolution} {quality}{pixels}\n".encode("ascii")
    started = time.monotonic()
    next_command = started
    last_frame_at = None
    try:
        while not stop.is_set():
            now = time.monotonic()
            if now - (last_frame_at if last_frame_at is not None else started) >= timeout:
                width, height = dimensions
                raise TimeoutError(f"No {width}×{height} frames for {timeout:g}s on {port.port}")
            # Renew START because a firmware profile can turn on the light by default.
            if now >= next_command:
                port.write(command)
                next_command = now + 1.0

            chunk = port.read(min(port.in_waiting or 1, READ_SIZE))
            for frame in parser.feed(chunk):
                if frame.dimensions == dimensions:
                    last_frame_at = time.monotonic()
                    yield frame
    finally:
        try:
            port.write(b"\nSTOP\n")
        except OSError:
            pass


class SerialCamera:
    def __init__(self, port, baud, resolution, quality, light=None):
        self.port = port
        self.baud = baud
        self.resolution = resolution
        self.quality = quality
        self.light = light
        self.description = f"{port} ({baud} baud)"

    def iter_jpegs(self, *, stop):
        with open_camera_port(self.port, uart=True, baud=self.baud) as port:
            for frame in iter_serial_frames(port, self.resolution, self.quality, stop, 5, FrameParser(),
                                            light=self.light):
                yield frame.jpeg
