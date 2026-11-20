#!/usr/bin/env python3
"""Capture a stationary visual reference, optionally run a short out-and-back test."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from record_aruco_range_sample import marker_detector, tag_observation


SIDES = ('left', 'right')
DEPTH_TOLERANCE_M = .025


def compare(reference, current):
    depth, corners = [], []
    for side in SIDES:
        a, b = reference[side], current[side]
        depth.append(float(b['center'][2] - a['center'][2]))
        delta = b['corners'] - a['corners']
        corners.append(float(np.sqrt(np.mean(np.sum(delta ** 2, axis=1)))))
    if abs(depth[0] - depth[1]) > .04:
        raise RuntimeError('Left and right depth changes disagree by more than 4 cm')
    return {'depth_delta_m': float(np.median(depth)), 'corner_error_px': float(np.median(corners))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--marker-id', type=int, default=23)
    parser.add_argument('--marker-mm', type=float, default=150.)
    parser.add_argument('--execute', action='store_true',
                        help='Move the robot. Use a clear, flat, supervised straight path.')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'diagnostics')
    args = parser.parse_args()
    if not 0 <= args.marker_id < 50 or not 20 <= args.marker_mm <= 300:
        parser.error('Use marker ID 0-49 and measured side length 20-300 mm.')

    detect = marker_detector()
    intrinsics, latest = {}, {}
    output = args.output / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '-aruco-image-return')
    output.mkdir(parents=True)
    report = {'marker_id': args.marker_id, 'marker_mm': args.marker_mm,
              'executed_motion': args.execute, 'steps': []}
    rclpy.init()
    node = rclpy.create_node('jetbot_image_based_return')

    def on_info(msg, side):
        intrinsics[side] = np.asarray(msg.p).reshape(3, 4)[:, :3]

    def on_image(msg, side):
        if side not in intrinsics or msg.encoding not in ('bgr8', 'rgb8'):
            return
        array = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        pixels = array[:, :3 * msg.width].reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY if msg.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)
        result = tag_observation(gray, intrinsics[side], args.marker_id, args.marker_mm / 1000, detect)
        if result and result['reprojection_rms_px'] < 1.8:
            latest[side] = (monotonic(), result, gray)

    for side in SIDES:
        topic = f'/stereo/{side}'
        node.create_subscription(CameraInfo, f'{topic}/camera_info',
                                 lambda m, s=side: on_info(m, s), qos_profile_sensor_data)
        node.create_subscription(Image, f'{topic}/image_rect',
                                 lambda m, s=side: on_image(m, s), qos_profile_sensor_data)

    def sample(label):
        start = monotonic()
        samples = {side: [] for side in SIDES}
        while monotonic() - start < 4:
            rclpy.spin_once(node, timeout_sec=.1)
            for side, values in samples.items():
                entry = latest.get(side)
                if entry and entry[0] >= start and (not values or values[-1][0] != entry[0]):
                    values.append(entry)
            if min(map(len, samples.values())) >= 12:
                break
        if min(map(len, samples.values())) < 8:
            raise RuntimeError(f'{label}: both cameras must see the entire marker')

        result = {}
        for side, values in samples.items():
            positions = np.array([v[1]['center_optical_m'] for v in values])
            if np.ptp(positions[:, 2]) > .035:
                raise RuntimeError(f'{label}: marker depth is unstable')
            result[side] = {'center': np.median(positions, axis=0),
                            'corners': np.median([v[1]['corners_px'] for v in values], axis=0)}
            if not cv2.imwrite(str(output / f'{label}-{side}.jpg'), values[-1][2]):
                raise RuntimeError('Unable to save the reference image')
        return result

    def move(speed, duration):
        target = os.environ.get('JETBOT_SSH_TARGET', 'ros2')
        if not target or target.startswith('-'):
            raise RuntimeError('Invalid JetBot SSH target')
        socket = os.environ.get('JETBOT_SSH_CONTROL_PATH', '/tmp/jetbot-small-area')
        command = ['timeout', '15s', 'ssh', '-S', socket, '-o', 'BatchMode=yes',
                   '-o', 'ConnectTimeout=4', target,
                   f'docker exec -i jetbot-mini-ros2 /jetbot-entrypoint.sh python3 - {speed:.2f} {duration:.2f}']
        helper = Path(__file__).with_name('jetbot_image_return_pulse.py')
        result = subprocess.run(command, input=helper.read_text(), text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        report['steps'].append({'speed_command': speed, 'duration_s': duration,
                                'motor_output': result.stdout, 'exit': result.returncode})
        if result.returncode or 'after {' not in result.stdout:
            raise RuntimeError('Motor pulse did not complete and verify zero PWM')

    try:
        reference = sample('reference')
        report['reference_depth_m'] = {side: float(s['center'][2]) for side, s in reference.items()}
        if not args.execute:
            return 0
        if min(report['reference_depth_m'].values()) < .65 or any(
            max(s['corners'][:, 0]) > 602 for s in reference.values()
        ):
            raise RuntimeError('Reference marker is too close to the camera or right frame edge')

        move(.04, .35)
        offset = compare(reference, sample('displaced'))
        report['displaced'] = offset
        if not -.16 <= offset['depth_delta_m'] <= -.025 or offset['corner_error_px'] > 45:
            raise RuntimeError('Observed departure did not match a small forward motion')
        speed = np.clip(abs(offset['depth_delta_m']) / .35, .12, .35)
        for index in range(1, 4):
            if offset['depth_delta_m'] >= -DEPTH_TOLERANCE_M:
                break
            previous = offset
            duration = round(float(np.clip(abs(offset['depth_delta_m']) / speed * .80, .10, .27)), 2)
            move(-.04, duration)
            offset = compare(reference, sample(f'return-{index}'))
            report['steps'][-1]['image_comparison'] = offset
            if offset['corner_error_px'] > previous['corner_error_px'] + 2:
                report['stop_reason'] = 'image error did not improve'
                break
        report['final'] = offset
        report['within_depth_tolerance'] = abs(offset['depth_delta_m']) <= DEPTH_TOLERANCE_M
        return 0 if report['within_depth_tolerance'] else 1
    except RuntimeError as error:
        report['error'] = str(error)
        return 1
    finally:
        path = output / 'report.json'
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(path)
        if 'error' in report:
            print(report['error'])
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
