"""Run inside the Nano driver container. Relay camera frames and enforce a command timeout."""
import base64
import json
import os
import sys
from queue import Empty, SimpleQueue
from threading import Thread
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from jetbot_mini_ros2.motion_test import MotionTest


def main():
    rclpy.init(args=[])
    node = MotionTest()
    commands = SimpleQueue()
    armed = False
    execute = os.environ.get('JETBOT_REPLAY_ENABLE') == '1'

    def emit(record):
        print(json.dumps(record, separators=(',', ':')), flush=True)

    def stream(message, side):
        stamp = message.header.stamp
        emit({'side': side, 'stamp': stamp.sec * 10**9 + stamp.nanosec,
              'jpg': base64.b64encode(message.data).decode('ascii')})

    def receive():
        try:
            for line in sys.stdin:
                commands.put((monotonic(), json.loads(line)))
        finally:
            commands.put((monotonic(), {'action': 'stop'}))

    for side in ('left', 'right'):
        node.create_subscription(CompressedImage, f'/stereo/{side}/image_raw/compressed',
                                 lambda msg, s=side: stream(msg, s), qos_profile_sensor_data)
    Thread(target=receive, daemon=True).start()
    last_status = 0.
    command_time = -1.
    armed_at = -1.
    twist = Twist()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = monotonic()
            report = node.report
            if report and now - last_status >= .06:
                emit({'motor': report})
                last_status = now
            pending = []
            while True:
                try:
                    pending.append(commands.get_nowait())
                except Empty:
                    break
            if any(command['action'] == 'stop' for _, command in pending):
                break
            arm_requested = any(command['action'] == 'arm' for _, command in pending)
            if arm_requested:
                if not execute or armed or not report or report['fault'] or report['dry_run']:
                    raise RuntimeError('Motor arm refused')
                if (report['enabled'] or report['pwm'] != [0, 0] or now - node.received_at > .4
                        or node.count_publishers('/cmd_vel') != 1
                        or node.publisher.get_subscription_count() < 1):
                    raise RuntimeError('Motor arm preflight failed')
                armed = True
                node.enable(True)
                command_time = armed_at = monotonic()
            if pending and pending[-1][1]['action'] == 'velocity':
                received, command = pending[-1]
                linear, angular = command['linear'], command['angular']
                if linear not in (0., .04) or angular not in (-.5, 0., .5) or (linear and angular):
                    raise RuntimeError('Invalid velocity')
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                command_time = max(armed_at, received)
            if armed:
                if (not report or report['fault'] or now - node.received_at > .4
                        or node.count_publishers('/cmd_vel') != 1):
                    raise RuntimeError('Motor status or command publisher changed')
                if now - armed_at > .5 and not report['enabled']:
                    raise RuntimeError('Motor no longer armed')
                if now - command_time > .30:
                    raise RuntimeError('Laptop command watchdog expired')
                node.publisher.publish(twist)
    finally:
        try:
            if armed:
                node.publisher.publish(Twist())
                node.enable(False)
                node.wait_until(lambda: node.report and not node.report['enabled']
                                and node.report['pwm'] == [0, 0], timeout=2.)
                emit({'stopped': node.report})
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
