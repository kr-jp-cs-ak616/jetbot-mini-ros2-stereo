#!/usr/bin/env python3
"""Record synchronized camera images at a manually measured map pose."""

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def record(output, pose, group, count, map_hashes, label_source):
    # The robot is stationary. Match within half a 15 Hz frame period,
    # with a small allowance for capture jitter.
    max_skew_ns = 35_000_000
    queues = {side: deque(maxlen=8) for side in ('left', 'right')}
    pairs = []
    last_left = -1
    last_right = -1
    stale = False
    rclpy.init()
    node = rclpy.create_node('jetbot_localization_sample_recorder')
    try:
        for side in queues:
            node.create_subscription(
                Image, f'/stereo/{side}/image_rect',
                lambda msg, side=side: queues[side].append(msg), qos_profile_sensor_data,
            )
        deadline = time.monotonic() + max(10., count * 1.5)
        while len(pairs) < count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now_ns = time.time_ns()
            for left in queues['left']:
                left_ns = stamp_ns(left)
                if left_ns <= last_left or not queues['right']:
                    continue
                right = min(queues['right'], key=lambda msg: abs(stamp_ns(msg) - left_ns))
                right_ns = stamp_ns(right)
                if right_ns <= last_right or abs(right_ns - left_ns) > max_skew_ns:
                    continue
                if any(not -500_000_000 <= now_ns - ts <= 2_000_000_000
                       for ts in (left_ns, right_ns)):
                    stale = True
                    continue
                pairs.append((left, right))
                # Space samples by at least 250 ms and never reuse right images.
                last_left = left_ns + 249_999_999
                last_right = right_ns
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(pairs) != count:
        if stale:
            raise SystemExit('촬영 실패. 영상 지연 또는 Nano와 노트북 간 시계 차이를 확인하세요.')
        raise SystemExit(f'촬영 실패. 시간차 35ms 이내의 좌우 쌍 {len(pairs)}/{count}개 수신.')
    output.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-')
    with tempfile.TemporaryDirectory(prefix='.recording-', dir=output) as work:
        staging = Path(work) / (prefix + Path(work).name.split('-')[-1])
        staging.mkdir()
        frames = []
        for index, pair in enumerate(pairs):
            files = {}
            for side, message in zip(queues, pair):
                if message.encoding not in ('bgr8', 'rgb8'):
                    raise SystemExit(f'{side}: 지원하지 않는 영상 형식 {message.encoding}')
                pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
                pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
                if message.encoding == 'rgb8':
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                name = f'{index:03d}-{side}.jpg'
                if not cv2.imwrite(str(staging / name), pixels):
                    raise SystemExit(f'{side}: JPEG 저장 실패')
                files[side] = {'file': name, 'stamp_ns': stamp_ns(message)}
            frames.append(files)
        metadata = {'frame_id': 'map', 'label_source': label_source,
                    'pose': pose, 'collection_group': group, 'map_sha256': map_hashes,
                    'image_type': 'rectified', 'max_pair_skew_ms': max_skew_ns / 1e6, 'frames': frames}
        (staging / 'sample.json').write_text(json.dumps(metadata, indent=2) + '\n')
        destination = output / staging.name
        staging.rename(destination)
    print(f'{len(frames)}쌍 저장 완료: {destination}')


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', required=True, type=float, help='map X in meters')
    parser.add_argument('--y', required=True, type=float, help='map Y in meters')
    parser.add_argument('--yaw-deg', required=True, type=float, help='degrees CCW from map +X')
    parser.add_argument('--group', required=True, help='collection run ID for grouped evaluation')
    parser.add_argument('--pairs', type=int, default=10)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--approximate', action='store_true',
                        help='mark a visual map estimate and default to a separate review folder')
    args = parser.parse_args()
    if not all(map(math.isfinite, (args.x, args.y, args.yaw_deg))):
        parser.error('위치와 각도는 유한한 숫자여야 합니다.')
    if not 1 <= args.pairs <= 60 or not args.group.strip():
        parser.error('pairs는 1~60, group은 비어 있지 않아야 합니다.')
    map_dir = project / 'maps/map2-room-reference-20261118'
    hashes = {name: hashlib.sha256((map_dir / name).read_bytes()).hexdigest()
              for name in ('map2_room.yaml', 'map2_room.pgm')}
    pose = dict(x_m=args.x, y_m=args.y, yaw_deg=(args.yaw_deg + 180.) % 360. - 180.)
    folder = 'localization-review' if args.approximate else 'localization-training'
    label_source = 'approximate_map_annotation' if args.approximate else 'manual_measurement'
    output = args.output if args.output is not None else project / 'captures' / folder
    record(output, pose, args.group, args.pairs, hashes, label_source)


if __name__ == '__main__':
    main()
