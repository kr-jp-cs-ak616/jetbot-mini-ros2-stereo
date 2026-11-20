"""Execute one bounded motion pulse inside the existing Nano driver container."""

import json
import sys
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from jetbot_mini_ros2.motion_test import MotionTest


def main():
    speed, duration = map(float, sys.argv[1:])
    if speed not in (-.04, .04) or not .10 <= duration <= .40:
        raise SystemExit('Invalid bounded pulse')

    rclpy.init(args=[])
    node = MotionTest()
    attempted_enable = False
    try:
        node.wait_until(lambda: node.report is not None and node.report['battery_voltage'] is not None
                        and node.publisher.get_subscription_count() > 0, timeout=5.0)
        if (node.report.get('fault') or node.report['enabled'] or node.report['dry_run']
                or node.report['pwm'] != [0, 0] or node.count_publishers('/cmd_vel') != 1):
            raise RuntimeError('Motor preflight refused: ' + json.dumps(node.report))
        print('before', json.dumps(node.report), flush=True)

        attempted_enable = True
        node.enable(True)
        command = Twist()
        command.linear.x = speed
        deadline = monotonic() + duration
        motor_report = None
        while monotonic() < deadline:
            if (node.report.get('fault') or node.count_publishers('/cmd_vel') != 1
                    or monotonic() - node.received_at > .4):
                raise RuntimeError('Motor status or command publisher changed')
            node.publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.02)
            if node.report['pwm'] != [0, 0]:
                motor_report = node.report
        print('pulse', json.dumps({'seconds': duration, 'acknowledged': motor_report}), flush=True)
        if motor_report is None:
            raise RuntimeError('No nonzero PWM observed')
    finally:
        try:
            if attempted_enable:
                node.publisher.publish(Twist())
                node.enable(False)
                node.wait_until(lambda: not node.report['enabled'] and node.report['pwm'] == [0, 0],
                                timeout=2.0)
                print('after', json.dumps(node.report), flush=True)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
