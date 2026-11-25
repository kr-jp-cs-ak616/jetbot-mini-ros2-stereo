#!/usr/bin/env python3
"""Estimate the reference map's camera pose from an upright, surveyed ArUco marker."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from record_aruco_range_sample import marker_detector, tag_observation


def camera_pose(observation, anchor):
    """Return map X, Y and yaw for stereo_camera_link at the left optical center."""
    yaw = math.radians(anchor['normal_yaw_deg'])
    normal = np.array([math.cos(yaw), math.sin(yaw), 0.])
    up = np.array([0., 0., 1.])
    map_from_tag = np.column_stack((np.cross(up, normal), up, normal))
    optical_from_tag = cv2.Rodrigues(np.array(observation['rotation_vector_rad']))[0]
    map_from_optical = map_from_tag @ optical_from_tag.T
    forward = map_from_optical[:, 2]
    if map_from_optical[2, 1] > -.75 or abs(forward[2]) > .5:
        raise ValueError('Marker and camera orientation do not resemble an upright installation.')
    tag_center = np.array([anchor['x_m'], anchor['y_m'], 0.])
    center = tag_center - map_from_optical @ observation['center_optical_m']
    return float(center[0]), float(center[1]), math.atan2(forward[1], forward[0])


def free_map_cell(grid, x, y):
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    dx, dy = x - origin.position.x, y - origin.position.y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    col = math.floor((cosine * dx + sine * dy) / grid.info.resolution)
    row = math.floor((-sine * dx + cosine * dy) / grid.info.resolution)
    if 0 <= col < grid.info.width and 0 <= row < grid.info.height:
        return grid.data[row * grid.info.width + col] == 0
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--anchor', type=Path, help='JSON with x_m, y_m, normal_yaw_deg and optional position_std_m')
    parser.add_argument('--marker-id', type=int, default=23)
    parser.add_argument('--marker-mm', type=float, default=150)
    parser.add_argument('--publish', action='store_true', help='Send the estimate to AMCL /initialpose')
    args = parser.parse_args()
    if not 0 <= args.marker_id < 50 or not 20 <= args.marker_mm <= 300:
        parser.error('Use marker ID 0-49 and a measured side of 20-300 mm.')
    if args.publish and args.anchor is None:
        parser.error('--publish requires a measured --anchor.')
    anchor = None
    position_std = .10
    if args.anchor:
        try:
            data = json.loads(args.anchor.read_text())
            anchor = {key: float(data[key]) for key in ('x_m', 'y_m', 'normal_yaw_deg')}
            if not all(map(math.isfinite, anchor.values())):
                raise ValueError('anchor coordinates must be finite')
            position_std = float(data.get('position_std_m', position_std))
            if not .03 <= position_std <= 1:
                raise ValueError('position_std_m must be between 0.03 and 1 m')
        except (OSError, KeyError, TypeError, ValueError) as error:
            parser.error(f'Invalid anchor: {error}')

    detector = marker_detector()
    latest_frame = None
    camera_info = None
    grid = None
    observations = deque(maxlen=12)
    rclpy.init()
    node = rclpy.create_node('jetbot_aruco_initial_pose')

    def on_info(message):
        nonlocal camera_info
        if (message.distortion_model == 'plumb_bob'
                and message.k[0] > 0 and message.k[4] > 0):
            camera_info = message

    def on_image(message):
        nonlocal latest_frame
        if camera_info is None or message.encoding not in ('bgr8', 'rgb8'):
            return
        if (message.width, message.height, message.header.frame_id) != (
            camera_info.width, camera_info.height, camera_info.header.frame_id
        ):
            return
        stamp = message.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if latest_frame is not None and ns - latest_frame < 100_000_000:
            return
        latest_frame = ns
        image = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        pixels = image[:, :3 * message.width].reshape(message.height, message.width, 3)
        gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY if message.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)
        observation = tag_observation(gray, np.asarray(camera_info.k).reshape(3, 3),
                                      args.marker_id, args.marker_mm / 1000, detector,
                                      distortion=np.asarray(camera_info.d))
        if observation:
            observations.append(observation)

    def on_map(message):
        nonlocal grid
        if message.header.frame_id == 'map' and message.info.resolution > 0:
            grid = message

    try:
        node.create_subscription(CameraInfo, '/stereo/left/camera_info', on_info, qos_profile_sensor_data)
        node.create_subscription(Image, '/stereo/left/image_raw', on_image, qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, '/map', on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10) if args.publish else None
        deadline = monotonic() + 8
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.15)
            if len(observations) >= 12 and (not publisher or (grid and publisher.get_subscription_count())):
                break
        if len(observations) < 8:
            raise SystemExit(f'ID {args.marker_id}: only {len(observations)} valid observations in the raw image.')
        corners = np.array([o['corners_px'] for o in observations])
        if np.linalg.norm(corners - np.median(corners, axis=0), axis=2).max() > 4:
            raise SystemExit('Keep the robot and marker stationary during initialization.')
        report = {
            'marker_id': args.marker_id, 'samples': len(observations),
            'marker_range_cm': round(float(np.median([o['range_m'] for o in observations])) * 100, 2),
        }
        if anchor is not None:
            try:
                poses = np.array([camera_pose(o, anchor) for o in observations])
            except ValueError as error:
                raise SystemExit(str(error)) from error
            x, y = np.median(poses[:, :2], axis=0)
            yaw = math.atan2(np.sin(poses[:, 2]).mean(), np.cos(poses[:, 2]).mean())
            yaw_deviation = np.arctan2(np.sin(poses[:, 2] - yaw), np.cos(poses[:, 2] - yaw))
            if np.linalg.norm(poses[:, :2] - (x, y), axis=1).max() > .10 or np.max(np.abs(yaw_deviation)) > .14:
                raise SystemExit('Marker-based pose is unstable; initialization skipped.')
            report['map_pose'] = {'x_m': round(float(x), 3), 'y_m': round(float(y), 3),
                                  'yaw_deg': round(math.degrees(yaw), 2)}
            if publisher:
                if grid is None or not publisher.get_subscription_count():
                    raise SystemExit('The reference map and AMCL /initialpose subscriber must be online.')
                if not free_map_cell(grid, x, y):
                    raise SystemExit('Marker-based pose lies outside verified free map space.')
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
                msg.pose.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.pose.orientation.w = math.cos(yaw / 2)
                covariance = [0.] * 36
                covariance[0] = covariance[7] = position_std ** 2
                covariance[35] = math.radians(12) ** 2
                msg.pose.covariance = covariance
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=.3)
                report['initialpose_published'] = True
        print(json.dumps(report))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
