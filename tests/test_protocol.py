import io
import struct
import unittest
from jetbot_stereo.protocol import HEADER, MAX_JPEG_BYTES, pack_frame, recv_frame


class Fragmented:
    def __init__(self,data):
        self.data=io.BytesIO(data)
    def recv(self,size):
        return self.data.read(min(3,size))


class ProtocolTests(unittest.TestCase):
    def test_fragmented_consecutive_frames_preserve_sensor_and_stamp(self):
        connection=Fragmented(pack_frame(0,1794800000123456789,4,b'left-jpeg')+
                              pack_frame(1,1794800000123456799,5,b'right-jpeg'))
        self.assertEqual(recv_frame(connection),(0,1794800000123456789,4,b'left-jpeg'))
        self.assertEqual(recv_frame(connection),(1,1794800000123456799,5,b'right-jpeg'))
        with self.assertRaises(EOFError):recv_frame(connection)

    def test_rejects_oversized_packet_before_reading_body(self):
        with self.assertRaises(ValueError):
            recv_frame(Fragmented(HEADER.pack(b'JBS1',0,MAX_JPEG_BYTES+1,1,1)))

    def test_truncated_frame_does_not_publish_partial_data(self):
        packet=pack_frame(0,1,1,b'jpeg')
        with self.assertRaises(EOFError):recv_frame(Fragmented(packet[:-1]))
