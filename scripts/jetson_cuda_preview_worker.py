#!/usr/bin/env python3
"""Nano host worker: rectify a live stereo pair and compute depth with VPI CUDA."""

import argparse
from pathlib import Path
import struct
import sys
from time import monotonic
import zlib

import cv2
import numpy as np
import vpi
import yaml


WIDTH, HEIGHT = 320, 240
MAX_DISPARITY = 128


def read_packet(stream):
    size = stream.read(4)
    if not size:
        return None
    if len(size) != 4:
        raise EOFError('Truncated packet header')
    length, = struct.unpack('!I', size)
    if not 0 < length <= 1_000_000:
        raise ValueError('Invalid packet length')
    data = stream.read(length)
    if len(data) != length:
        raise EOFError('Truncated packet')
    return data


def write_packet(stream, payload):
    stream.write(struct.pack('!I', len(payload)))
    stream.write(payload)


def load_calibration(folder, side):
    with (folder / (side + '.yaml')).open() as source:
        cal = yaml.safe_load(source)
    if (cal['image_width'], cal['image_height']) != (640, 480):
        raise ValueError('Expected 640x480 camera calibration')
    k = np.array(cal['camera_matrix']['data']).reshape(3, 3)
    rotation = np.array(cal['rectification_matrix']['data']).reshape(3, 3)
    projection = np.array(cal['projection_matrix']['data']).reshape(3, 4)
    projection[:2] *= .5
    # Preserve the pixel-center convention when halving the rectified image.
    projection[:2, 2] -= .25
    x, y = cv2.initUndistortRectifyMap(k, np.array(cal['distortion_coefficients']['data']),
                                    rotation, projection, (WIDTH, HEIGHT), cv2.CV_32FC1)
    warp = vpi.WarpMap((WIDTH, HEIGHT))
    np.asarray(warp)[:] = np.dstack((x, y))
    return warp, projection, (x >= 2) & (x < 638) & (y >= 2) & (y < 478)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration_dir', type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    left, right = [load_calibration(args.calibration_dir, side) for side in ('left', 'right')]
    lp, rp = left[1], right[1]
    baseline = lp[0, 3] / lp[0, 0] - rp[0, 3] / rp[0, 0]
    if not np.isfinite(baseline) or baseline <= 0 or not np.isclose(lp[0, 0], rp[0, 0]):
        raise ValueError('Expected a positive rectified horizontal stereo baseline')
    bf = lp[0, 0] * baseline
    col_offset = lp[0, 2] - rp[0, 2]
    yy, xx = np.indices((HEIGHT, WIDTH))
    confidence = vpi.Image((WIDTH, HEIGHT), vpi.Format.U16)
    source, out = sys.stdin.buffer, sys.stdout.buffer
    print('VPI backend=CUDA size=320x240 max_disparity=128', file=sys.stderr, flush=True)
    while True:
        packets = [read_packet(source), read_packet(source)]
        if packets[0] is None:
            return
        if packets[1] is None:
            raise EOFError('Missing right image')
        images = [cv2.imdecode(np.frombuffer(p, np.uint8), cv2.IMREAD_GRAYSCALE) for p in packets]
        if any(image is None or image.shape != (480, 640) for image in images):
            raise ValueError('Expected a pair of valid 640x480 JPEGs')
        start = monotonic()
        with vpi.Backend.CUDA:
            rect = [vpi.asimage(image).remap(cal[0], size=(WIDTH, HEIGHT), border=vpi.Border.ZERO)
                    for image, cal in zip(images, (left, right))]
            disparity = vpi.stereodisp(rect[0].convert(vpi.Format.Y16_ER),
                                       rect[1].convert(vpi.Format.Y16_ER),
                                       maxdisp=MAX_DISPARITY, out_confmap=confidence)
            rectified = [np.array(frame.cpu(), copy=True) for frame in rect]
            raw = disparity.cpu().astype(np.float32) / 32
            cert = confidence.cpu().copy()
        gpu_ms = (monotonic() - start) * 1000

        col = np.rint(xx - raw).astype(int)
        supported = (col >= 0) & (col < WIDTH) & right[2][yy, col.clip(0, WIDTH - 1)]
        gray = rectified[0].astype(np.float32)
        texture = cv2.boxFilter(gray * gray, -1, (7, 7)) - cv2.boxFilter(gray, -1, (7, 7)) ** 2
        positive = raw - col_offset
        with np.errstate(divide='ignore', invalid='ignore'):
            metric = bf / positive
        valid = (left[2] & supported & (cert >= 32767) & (raw > 1) & (raw < MAX_DISPARITY - 1)
                 & (texture >= 25) & (metric >= .4) & (metric <= 5))
        metric[~valid] = np.nan

        out.write(struct.pack('!f', gpu_ms))
        for frame in rectified:
            write_packet(out, zlib.compress(frame.tobytes(), 1))
        write_packet(out, zlib.compress(metric.astype('<f4').tobytes(), 1))
        out.flush()


if __name__ == '__main__':
    main()
