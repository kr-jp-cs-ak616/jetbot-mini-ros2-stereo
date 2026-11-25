#!/usr/bin/env python3
"""Record a stationary ArUco observation with a tape-measured range label."""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def tag_observation(image, intrinsics, marker_id, side_m, detector, distortion=None):
    corners, identifiers, _ = detector(image)
    if identifiers is None or list(identifiers.ravel()).count(marker_id) != 1:
        return None
    index = list(identifiers.ravel()).index(marker_id)
    pixels = corners[index].reshape(4, 2).astype(np.float64)
    height, width = image.shape
    if (pixels.min() < 5 or pixels[:, 0].max() > width - 6
            or pixels[:, 1].max() > height - 6):
        return None
    half = side_m / 2
    target = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]])
    ok, rotation, translation = cv2.solvePnP(
        target, pixels, intrinsics, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or translation[2, 0] <= 0:
        return None
    projected = cv2.projectPoints(target, rotation, translation, intrinsics, distortion)[0]
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels) ** 2, axis=1))))
    if rms > 2:
        return None
    return {
        'corners_px': pixels.tolist(),
        'center_optical_m': translation.ravel().tolist(),
        'rotation_vector_rad': rotation.ravel().tolist(),
        'range_m': float(np.linalg.norm(translation)),
        'reprojection_rms_px': rms,
    }


def marker_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'ArucoDetector'):
        parameters = aruco.DetectorParameters()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        return aruco.ArucoDetector(dictionary, parameters).detectMarkers
    else:
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        def detect(image):
            return aruco.detectMarkers(image, dictionary, parameters=parameters)

        return detect


def capture(marker_id, side_m):
    detect = marker_detector()
    frames = {side: deque(maxlen=8) for side in ('left', 'right')}
    projections = {}
    pairs = []
    last_pair = [0, 0]
    rclpy.init()
    node = rclpy.create_node('jetbot_record_aruco_range_sample')

    def on_frame(message, side):
        if side not in projections or message.encoding not in ('bgr8', 'rgb8'):
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        array = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        pixels = array[:, :3 * message.width].reshape(message.height, message.width, 3)
        gray_code = cv2.COLOR_BGR2GRAY if message.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY
        gray = cv2.cvtColor(pixels, gray_code)
        intrinsics = np.asarray(projections[side]).reshape(3, 4)[:, :3]
        result = tag_observation(gray, intrinsics, marker_id, side_m, detect)
        if result is None:
            return
        result['stamp_ns'] = stamp_ns
        frames[side].append(result)
        other = 'right' if side == 'left' else 'left'
        if not frames[other]:
            return
        match = min(frames[other], key=lambda sample: abs(sample['stamp_ns'] - stamp_ns))
        if abs(match['stamp_ns'] - stamp_ns) > 30_000_000:
            return
        pair = {side: result, other: match}
        stamps = [pair[s]['stamp_ns'] for s in ('left', 'right')]
        if any(new - old < 150_000_000 for old, new in zip(last_pair, stamps)):
            return
        last_pair[:] = stamps
        pairs.append(pair)

    try:
        for side in frames:
            topic = f'/stereo/{side}'
            node.create_subscription(
                CameraInfo, f'{topic}/camera_info',
                lambda m, s=side: projections.__setitem__(s, list(m.p)), qos_profile_sensor_data,
            )
            node.create_subscription(
                Image, f'{topic}/image_rect', lambda m, s=side: on_frame(m, s), qos_profile_sensor_data,
            )
        deadline = monotonic() + 16
        while monotonic() < deadline and len(pairs) < 20:
            rclpy.spin_once(node, timeout_sec=.15)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(pairs) < 8:
        raise SystemExit(f'Sample not saved. Only {len(pairs)} synchronized detections.')
    for side in frames:
        positions = np.array([pair[side]['center_optical_m'] for pair in pairs])
        corners = np.array([pair[side]['corners_px'] for pair in pairs])
        corner_shift = np.linalg.norm(corners - np.median(corners, axis=0), axis=-1)
        if np.ptp(positions[:, 2]) > .035 or corner_shift.max() > 4:
            raise SystemExit('Sample not saved. Keep the robot and marker stationary.')
    return pairs, projections


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose-id', required=True, help='Unique name for this physical position')
    parser.add_argument('--distance-cm', type=float, required=True, help='Left lens to marker center')
    parser.add_argument('--marker-mm', type=float, required=True, help='Measured outer black edge')
    parser.add_argument('--marker-id', type=int, default=23)
    parser.add_argument('--approximate', action='store_true', help='Mark the tape measurement as approximate')
    parser.add_argument('--nominal-marker-size', action='store_true',
                        help='The marker size is taken from the print specification')
    parser.add_argument('--output', type=Path, default=project / 'captures' / 'aruco-range-training')
    args = parser.parse_args()
    if (not args.pose_id.strip() or not 15 <= args.distance_cm <= 500
            or not 20 <= args.marker_mm <= 300 or not 0 <= args.marker_id < 50):
        parser.error('Provide a pose ID, distance 15-500 cm, marker side 20-300 mm and ID 0-49.')
    pairs, projections = capture(args.marker_id, args.marker_mm / 1000)
    captured_at = datetime.now(timezone.utc)
    estimate = float(np.median([p['left']['range_m'] for p in pairs]))
    report = {
        'label_source': 'manual_tape_left_lens_center_to_marker_center',
        'group': args.pose_id, 'captured_at_utc': captured_at.isoformat(),
        'dictionary': 'DICT_4X4_50', 'marker_id': args.marker_id,
        'marker_side_mm': args.marker_mm, 'measured_range_m': args.distance_cm / 100,
        'range_approximate': args.approximate,
        'marker_size_source': 'print_specification' if args.nominal_marker_size else 'manual_measurement',
        'left_median_range_m': estimate, 'projections': projections, 'pairs': pairs,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output / captured_at.strftime('%Y%m%dT%H%M%S.%fZ.json')
    with output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(f'{len(pairs)} pairs, one measured position. Left estimate {estimate * 100:.2f} cm.')
    print(output)


if __name__ == '__main__':
    main()
