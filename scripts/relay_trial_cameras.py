#!/usr/bin/env python3
"""Use a second stereo calibration without changing the camera or motor services."""

import argparse
from pathlib import Path
import signal

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage

from jetbot_stereo.calibration import load_calibration, validate_pair
from jetbot_stereo.decode_stereo import Decoder


def subscribe_camera(source, target, side, calibration):
    prefix = f'/stereo/{side}'
    camera_info = target.create_publisher(
        CameraInfo, f'{prefix}/camera_info', qos_profile_sensor_data)
    compressed = target.create_publisher(
        CompressedImage, f'{prefix}/image_raw/compressed', qos_profile_sensor_data)
    info = CameraInfo(width=640, height=480, distortion_model=calibration['distortion_model'])
    for attribute, key in [('k', 'camera_matrix'), ('d', 'distortion_coefficients'),
                           ('r', 'rectification_matrix'), ('p', 'projection_matrix')]:
        setattr(info, attribute, [float(value) for value in calibration[key]['data']])

    def relay(message):
        info.header = message.header
        camera_info.publish(info)
        compressed.publish(message)

    source.create_subscription(
        CompressedImage, f'{prefix}/image_raw/compressed', relay, qos_profile_sensor_data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration_dir', type=Path)
    args = parser.parse_args()
    sides = ('left', 'right')
    calibrations = [load_calibration(args.calibration_dir / f'{side}.yaml', 640, 480)
                    for side in sides]
    baseline = validate_pair(*calibrations)

    # Domain 16 carries the Nano camera stream. The RViz trial runs in domain 97.
    source_context = Context()
    rclpy.init(context=source_context, domain_id=16, signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(domain_id=97, signal_handler_options=SignalHandlerOptions.NO)
    source = rclpy.create_node('calibration_camera_input', context=source_context)
    target = rclpy.create_node('calibration_camera_output')
    decoder = Decoder()
    source_executor = SingleThreadedExecutor(context=source_context)
    target_executor = SingleThreadedExecutor()
    source_executor.add_node(source)
    target_executor.add_node(decoder)
    for side, calibration in zip(sides, calibrations):
        subscribe_camera(source, target, side, calibration)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    target.get_logger().info(f'Calibration trial baseline: {baseline * 1000:.2f} mm')
    try:
        while True:
            source_executor.spin_once(timeout_sec=.02)
            for _ in range(7):
                target_executor.spin_once(timeout_sec=0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        source_executor.shutdown()
        target_executor.shutdown()
        decoder.destroy_node()
        target.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown(context=source_context)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
