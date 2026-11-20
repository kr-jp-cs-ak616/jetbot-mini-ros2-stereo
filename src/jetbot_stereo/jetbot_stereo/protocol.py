"""Bounded JPEG packets between JetPack capture and the ROS container."""
import struct

HEADER = struct.Struct('!4sBIQI')
MAGIC = b'JBS1'
MAX_JPEG_BYTES = 2 * 1024 * 1024


def pack_frame(sensor, stamp_ns, sequence, jpeg):
    if sensor not in (0, 1) or not 0 < len(jpeg) <= MAX_JPEG_BYTES:
        raise ValueError('Invalid sensor or JPEG length')
    return HEADER.pack(MAGIC, sensor, len(jpeg), stamp_ns, sequence & 0xffffffff) + jpeg


def recv_exact(connection, size):
    chunks = bytearray()
    while len(chunks) < size:
        block = connection.recv(size-len(chunks))
        if not block:
            raise EOFError('Capture connection closed')
        chunks.extend(block)
    return bytes(chunks)


def recv_frame(connection):
    magic, sensor, length, stamp, sequence = HEADER.unpack(recv_exact(connection, HEADER.size))
    if magic != MAGIC or sensor not in (0, 1) or not 0 < length <= MAX_JPEG_BYTES:
        raise ValueError('Invalid camera packet')
    return sensor, stamp, sequence, recv_exact(connection, length)
