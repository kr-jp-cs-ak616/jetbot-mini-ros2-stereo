#!/usr/bin/env python3
"""Probe or run a short, forward-only ArUco search with a wall-distance stop."""

import argparse
import json
import math
from time import monotonic

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from record_aruco_range_sample import marker_detector, tag_observation


SCAN_TIMEOUT = .65
STOP_DISTANCE = .45
MARKER_ID = 23
MARKER_SIZE_M = .15


def wall_clearance(scan, now_ns):
    if scan is None:
        return 'no wall scan', None
    stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
    age = (now_ns - stamp) / 1e9
    if not -.05 <= age <= SCAN_TIMEOUT:
        return 'wall scan expired', None
    if (scan.header.frame_id != 'stereo_camera_link'
            or not math.isfinite(scan.angle_min)
            or not 0 < scan.angle_increment <= math.radians(1.1)
            or not 0 < scan.range_min < STOP_DISTANCE < scan.range_max):
        return 'unexpected wall scan geometry', None

    angles = np.degrees(scan.angle_min + scan.angle_increment * np.arange(len(scan.ranges)))
    front = np.abs(angles) <= 20.1
    ranges = np.asarray(scan.ranges)[front]
    if (front.sum() < 39 or not np.isfinite(ranges).all()
            or np.any(ranges < scan.range_min) or np.any(ranges > scan.range_max)):
        return 'incomplete forward wall distance', None
    nearest = float(ranges.min())
    return ('wall too close' if nearest <= STOP_DISTANCE else None), nearest


class Trial:
    def __init__(self, vision, motor, vision_executor, motor_executor):
        self.vision = vision
        self.motor = motor
        self.vision_executor = vision_executor
        self.motor_executor = motor_executor
        self.scan = self.info = self.marker = self.motor_status = None
        self.frame_received = self.status_received = self.scan_received = -math.inf
        self.last_image_stamp = 0
        self.hits = 0
        self.detect = marker_detector()
        vision.create_subscription(LaserScan, '/partition/wall_scan', self.on_scan, qos_profile_sensor_data)
        vision.create_subscription(CameraInfo, '/stereo/left/camera_info', self.on_info, qos_profile_sensor_data)
        vision.create_subscription(Image, '/stereo/left/image_raw', self.on_image, qos_profile_sensor_data)
        motor.create_subscription(String, '/jetbot/status', self.on_status, 10)
        self.enable = motor.create_client(SetBool, '/jetbot/enable')
        self.publisher = None

    def on_scan(self, message):
        self.scan = message
        self.scan_received = monotonic()

    def on_info(self, message):
        self.info = message

    def on_image(self, message):
        info = self.info
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= self.last_image_stamp or not -.05 <= (self.vision.get_clock().now().nanoseconds - stamp) / 1e9 <= .5:
            return
        if (info is None or message.encoding not in ('bgr8', 'rgb8')
                or (message.width, message.height, message.header.frame_id)
                != (info.width, info.height, info.header.frame_id)
                or info.distortion_model != 'plumb_bob' or min(info.k[0], info.k[4]) <= 0):
            return
        self.last_image_stamp = stamp
        rows = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        color = rows[:, :message.width * 3].reshape(message.height, message.width, 3)
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY if message.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)
        self.marker = tag_observation(gray, np.asarray(info.k).reshape(3, 3), MARKER_ID, MARKER_SIZE_M,
                                      self.detect, distortion=np.asarray(info.d))
        self.hits = self.hits + 1 if self.marker is not None else 0
        self.frame_received = monotonic()

    def on_status(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            status = None
        self.motor_status = status if isinstance(status, dict) else None
        self.status_received = monotonic()

    def spin(self):
        self.vision_executor.spin_once(timeout_sec=.02)
        self.motor_executor.spin_once(timeout_sec=.01)

    def conditions(self, enabled):
        if monotonic() - self.scan_received > SCAN_TIMEOUT and self.scan is not None:
            raise RuntimeError('wall scan expired')
        reason, nearest = wall_clearance(self.scan, self.vision.get_clock().now().nanoseconds)
        if reason:
            raise RuntimeError(reason)
        now = monotonic()
        if now - self.frame_received > .4:
            raise RuntimeError('camera frame missing or expired')
        if now - self.status_received > .5 or not self.motor_status:
            raise RuntimeError('motor status missing or expired')
        status = self.motor_status
        if (status.get('dry_run') is not False or status.get('fault') is not None
                or status.get('enabled') is not enabled or (not enabled and status.get('pwm') != [0, 0])):
            raise RuntimeError('motor status does not permit driving')
        expected = 1 if self.publisher is not None else 0
        if self.motor.count_publishers('/cmd_vel') != expected:
            raise RuntimeError('another motor command publisher is present')
        return nearest

    def set_enabled(self, enabled):
        if not self.enable.service_is_ready():
            raise RuntimeError('motor enable service unavailable')
        future = self.enable.call_async(SetBool.Request(data=enabled))
        deadline = monotonic() + 1.5
        while not future.done() and monotonic() < deadline:
            self.spin()
        if not future.done() or not future.result().success:
            raise RuntimeError('motor enable/disable request failed')

    def drive(self):
        self.conditions(enabled=False)
        self.publisher = self.motor.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        attempted_enable = False
        try:
            deadline = monotonic() + 1
            while not self.publisher.get_subscription_count() and monotonic() < deadline:
                self.spin()
            if not self.publisher.get_subscription_count():
                raise RuntimeError('motor driver does not subscribe to /cmd_vel')
            self.conditions(enabled=False)
            attempted_enable = True
            self.set_enabled(True)
            deadline = monotonic() + 1
            while not (self.motor_status or {}).get('enabled') and monotonic() < deadline:
                self.spin()
            search_end = monotonic() + 1.5
            trial_end = monotonic() + 3.
            acquired = False
            command = Twist()
            command.linear.x = .04
            while monotonic() < trial_end:
                self.spin()
                self.conditions(enabled=True)
                if self.marker is None:
                    if acquired or monotonic() >= search_end:
                        return 'ID 23 lost' if acquired else 'ID 23 not found within search window'
                else:
                    if self.hits < 3:
                        self.publisher.publish(Twist())
                        continue
                    acquired = True
                    x, _, z = self.marker['center_optical_m']
                    if z <= .55:
                        return 'stopped in front of ID 23'
                    if abs(math.atan2(x, z)) > math.radians(8):
                        return 'ID 23 outside the forward approach corridor'
                self.publisher.publish(command)
            return 'trial time limit reached'
        finally:
            if attempted_enable:
                self.publisher.publish(Twist())
                requested_at = monotonic()
                self.set_enabled(False)
                deadline = monotonic() + 1.5
                while monotonic() < deadline:
                    if (self.status_received >= requested_at and self.motor_status
                            and self.motor_status.get('enabled') is False
                            and self.motor_status.get('pwm') == [0, 0]):
                        print('Motor stop confirmed')
                        break
                    self.spin()
                else:
                    raise RuntimeError('motor stop was not acknowledged')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Enable bounded forward driving after preflight')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    vision_context, motor_context = Context(), Context()
    rclpy.init(context=vision_context, domain_id=97)
    rclpy.init(context=motor_context, domain_id=16)
    vision = rclpy.create_node('id23_wall_guard_vision', context=vision_context)
    motor = rclpy.create_node('id23_wall_guard_motor', context=motor_context)
    vision_executor = SingleThreadedExecutor(context=vision_context)
    motor_executor = SingleThreadedExecutor(context=motor_context)
    vision_executor.add_node(vision)
    motor_executor.add_node(motor)
    trial = Trial(vision, motor, vision_executor, motor_executor)
    try:
        deadline = monotonic() + 5.
        while monotonic() < deadline:
            trial.spin()
        print(f'ID 23 consecutive detections: {trial.hits}')
        print(f'Motor status: {json.dumps(trial.motor_status)}')
        nearest = trial.conditions(enabled=False)
        print(f'Preflight passed, nearest confirmed wall {nearest:.2f} m')
        if args.execute:
            print(trial.drive())
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f'Trial stopped: {error or "interrupted"}')
        return 2
    finally:
        vision_executor.shutdown()
        motor_executor.shutdown()
        vision.destroy_node()
        motor.destroy_node()
        rclpy.try_shutdown(context=vision_context)
        rclpy.try_shutdown(context=motor_context)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
