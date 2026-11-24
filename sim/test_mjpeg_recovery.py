"""Regression checks for the JPEG receiver on damaged and interrupted streams."""

import io
import unittest
from email.message import Message

from tools.camera_stream import IncompleteJpegPart, iter_jpegs


JPEG = b"\xff\xd8image data\xff\xd9"


class MjpegResponse(io.BytesIO):
    def __init__(self, body, boundary="frame"):
        super().__init__(body)
        self.headers = Message()
        self.headers["Content-Type"] = f"multipart/x-mixed-replace; boundary={boundary}"


def part(payload, *, size=None):
    length = len(payload) if size is None else size
    return (b"--frame\r\nContent-Type: image/jpeg\r\n"
            + f"Content-Length: {length}\r\n\r\n".encode("ascii") + payload)


class MjpegRecoveryTests(unittest.TestCase):
    def test_delivers_complete_payload_even_if_connection_closes_before_separator(self):
        self.assertEqual(list(iter_jpegs(MjpegResponse(part(JPEG)))), [JPEG])

    def test_live_reader_skips_damaged_part_and_continues_with_next_frame(self):
        body = part(JPEG) + b"\r\n" + part(JPEG[:-2] + b"xx") + b"\r\n" + part(JPEG)
        self.assertEqual(list(iter_jpegs(MjpegResponse(body), skip_damaged=True)), [JPEG, JPEG])
        with self.assertRaisesRegex(ValueError, "damaged JPEG"):
            list(iter_jpegs(MjpegResponse(body)))

    def test_disconnect_during_payload_never_yields_partial_frame(self):
        body = part(JPEG) + b"\r\n" + part(JPEG[:5], size=len(JPEG))
        frames = iter_jpegs(MjpegResponse(body), skip_damaged=True)
        self.assertEqual(next(frames), JPEG)
        with self.assertRaisesRegex(IncompleteJpegPart, "5/14 bytes"):
            next(frames)

    def test_legacy_boundary_with_prefix_still_works(self):
        body = part(JPEG) + b"\r\n--frame--\r\n"
        self.assertEqual(list(iter_jpegs(MjpegResponse(body, boundary="--frame"))), [JPEG])


if __name__ == "__main__":
    unittest.main()
