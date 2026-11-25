#!/usr/bin/env python3
"""Publish isolated CUDA previews using an existing Nano camera stream."""

import argparse
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
from time import monotonic
import zlib

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

WIDTH, HEIGHT = 320, 240
MAX_SKEW_NS = 30_000_000


def read_exact(stream, size, deadline):
    result = bytearray()
    while len(result) < size:
        remaining = deadline - monotonic()
        if remaining <= 0 or not select.select([stream], [], [], remaining)[0]:
            raise TimeoutError('Nano CUDA worker response timed out')
        data = os.read(stream.fileno(), size - len(result))
        if not data:
            raise EOFError('Nano CUDA worker ended')
        result.extend(data)
    return result


def read_image(stream, dtype, deadline):
    size, = struct.unpack('!I', read_exact(stream, 4, deadline))
    if not 0 < size <= 1_000_000:
        raise ValueError('Invalid Nano result size')
    pixels = zlib.decompress(read_exact(stream, size, deadline))
    return np.frombuffer(pixels, dtype=dtype).reshape(HEIGHT, WIDTH)


def stamp(frame):
    return frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec


def publish(publisher, array, encoding, header, frame_id):
    msg = Image()
    msg.header.stamp = header.stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = array.strides[0]
    msg.data = array.tobytes()
    publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=float, default=4.)
    parser.add_argument('--seconds', type=float, default=15., help='Use 0 for a continuous preview')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .5 <= args.rate <= 15 or not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('Expected a rate of .5-15 Hz and nonnegative seconds')
    target = os.environ.get('JETBOT_SSH_TARGET', 'ros2')
    if not target or target.startswith('-'):
        parser.error('Invalid Nano SSH target')
    socket = os.environ.get('JETBOT_SSH_CONTROL_PATH', '/tmp/jetbot-small-area')
    cmd = ['ssh', '-T', '-S', socket, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', target,
           'cd ~/jetbot_cuda_preview && exec python3 scripts/jetson_cuda_preview_worker.py calibration/aruco-stereo-review']
    queues = {side: deque(maxlen=5) for side in ('left', 'right')}
    rclpy.init()
    node = rclpy.create_node('jetbot_cuda_preview')
    rectified = {}
    for side, queue in queues.items():
        node.create_subscription(CompressedImage, '/stereo/' + side + '/image_raw/compressed',
                                 lambda msg, target_queue=queue: target_queue.append(msg),
                                 qos_profile_sensor_data)
        rectified[side] = node.create_publisher(Image, '/jetbot_cuda/' + side + '/image_rect',
                                               qos_profile_sensor_data)
    depth_pub = node.create_publisher(Image, '/jetbot_cuda/depth', qos_profile_sensor_data)
    preview_pub = node.create_publisher(Image, '/jetbot_cuda/depth_preview', qos_profile_sensor_data)
    start = monotonic()
    deadline = start + args.seconds if args.seconds else math.inf
    next_frame = start
    last_pair = (0, 0)
    observations = deque(maxlen=200)
    preview = None
    error = None
    worker = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    try:
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if monotonic() < next_frame or not all(queues.values()):
                continue
            left = queues['left'][-1]
            right = min(queues['right'], key=lambda msg: abs(stamp(msg) - stamp(left)))
            pair = stamp(left), stamp(right)
            skew = abs(pair[0] - pair[1])
            if pair[0] <= last_pair[0] or pair[1] <= last_pair[1] or skew > MAX_SKEW_NS:
                continue
            if node.get_clock().now().nanoseconds - min(pair) > 600_000_000:
                continue
            last_pair = pair
            sent = monotonic()
            next_frame = sent + 1 / args.rate
            for msg in (left, right):
                payload = bytes(msg.data)
                worker.stdin.write(struct.pack('!I', len(payload)))
                worker.stdin.write(payload)
            worker.stdin.flush()
            response_until = monotonic() + 5
            elapsed, = struct.unpack('!f', read_exact(worker.stdout, 4, response_until))
            for side, msg in zip(('left', 'right'), (left, right)):
                frame = read_image(worker.stdout, np.uint8, response_until)
                publish(rectified[side], frame, 'mono8', msg.header, 'jetbot_cuda_' + side + '_rect_optical')
            depth = read_image(worker.stdout, '<f4', response_until)
            valid = np.isfinite(depth)
            color = np.nan_to_num(depth, nan=0.)
            view = cv2.applyColorMap(np.uint8(np.clip((color - .4) / 2.6, 0, 1) * 255), cv2.COLORMAP_TURBO)
            view[~valid] = 0
            preview = view
            optical = 'jetbot_cuda_left_rect_optical'
            publish(depth_pub, depth, '32FC1', left.header, optical)
            publish(preview_pub, view, 'bgr8', left.header, optical)
            center_values = depth[80:180, 110:210]
            center_values = center_values[np.isfinite(center_values)]
            observations.append({
                'skew_ms': skew / 1e6, 'nano_cuda_processing_ms': elapsed,
                'round_trip_ms': 1000 * (monotonic() - sent),
                'valid_fraction': float(valid.mean()),
                'center_median_m': float(np.median(center_values)) if len(center_values) else None,
            })
    except KeyboardInterrupt:
        pass
    except (OSError, EOFError, ValueError, zlib.error) as exc:
        error = str(exc)
    finally:
        worker.stdin.close()
        try:
            worker.wait(timeout=2)
        except subprocess.TimeoutExpired:
            worker.terminate()
            worker.wait(timeout=3)
        node.destroy_node()
        rclpy.try_shutdown()

    values = list(observations)
    summary = {'backend': 'VPI CUDA on Nano host', 'frames': len(values), 'error': error}
    if values:
        for key in values[0]:
            numbers = [row[key] for row in values if row[key] is not None]
            if numbers:
                summary[key + '_median'] = float(np.median(numbers))
    if args.seconds:
        directory = Path(__file__).resolve().parents[1] / 'diagnostics'
        directory.mkdir(exist_ok=True)
        path = directory / (datetime.now().strftime('%Y%m%d-%H%M%S') + '-nano-cuda-live-preview.json')
        path.write_text(json.dumps(dict(summary, samples=values), indent=2) + '\n')
        if preview is not None:
            cv2.imwrite(str(path.with_suffix('.png')), preview)
        print(path)
    print(json.dumps(summary))
    return 0 if values and not error else 1


if __name__ == '__main__':
    raise SystemExit(main())
