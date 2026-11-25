#!/usr/bin/env python3
"""Record original JPEGs, capture times and motor telemetry inside the Nano."""

import argparse
import json
import shutil
import signal
import time
from contextlib import ExitStack
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String


SIDES = ('left', 'right')
MAX_SECONDS = 30 * 60
MIN_FREE_BYTES = 512 * 1024 * 1024


def write_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2) + '\n')
    tmp.replace(path)


def frame_header(message):
    stamp = message.header.stamp
    return {
        'capture_ns': stamp.sec * 1_000_000_000 + stamp.nanosec,
        'frame_id': message.header.frame_id,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    directory = parser.parse_args().directory
    started_ns = time.time_ns()
    frame_counts = dict.fromkeys(SIDES, 0)
    offsets = dict.fromkeys(SIDES, 0)
    camera_info = {}
    cmd_count = status_count = 0
    last_status = None
    frame_times = {}
    last_status_time = float('-inf')
    finished = False
    ready = False
    reason = 'stop requested'
    rclpy.init(args=[])
    node = rclpy.create_node('jetbot_manual_drive_recorder')

    def request_stop(*_):
        nonlocal finished
        finished = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)

    with ExitStack() as files:
        videos = {
            side: files.enter_context((directory / f'{side}.mjpeg').open('xb'))
            for side in SIDES
        }
        timeline = files.enter_context((directory / 'events.jsonl').open('x'))

        def event(kind, **values):
            timeline.write(json.dumps({'kind': kind, 'received_ns': time.time_ns(), **values}) + '\n')

        def frame(message, side):
            data = bytes(message.data)
            videos[side].write(data)
            event('frame', side=side, index=frame_counts[side], offset=offsets[side],
                  length=len(data), format=message.format, **frame_header(message))
            offsets[side] += len(data)
            frame_counts[side] += 1
            frame_times[side] = time.monotonic()

        def info(message, side):
            calibration = {
                'width': message.width, 'height': message.height,
                'distortion_model': message.distortion_model, 'd': list(message.d),
                'k': list(message.k), 'r': list(message.r), 'p': list(message.p),
            }
            if camera_info.get(side) != calibration:
                camera_info[side] = calibration
                event('camera_info', side=side, **calibration, **frame_header(message))

        def command(message):
            nonlocal cmd_count
            cmd_count += 1
            event('cmd_vel', linear_x=message.linear.x, angular_z=message.angular.z)

        def status(message):
            nonlocal last_status, last_status_time, status_count
            try:
                data = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if not isinstance(data, dict):
                return
            status_count += 1
            last_status = data
            last_status_time = time.monotonic()
            event('motor_status', status=data)

        for side in SIDES:
            prefix = f'/stereo/{side}'
            node.create_subscription(
                CompressedImage, f'{prefix}/image_raw/compressed',
                lambda m, s=side: frame(m, s), qos_profile_sensor_data)
            node.create_subscription(CameraInfo, f'{prefix}/camera_info',
                                     lambda m, s=side: info(m, s), qos_profile_sensor_data)
        node.create_subscription(Twist, '/cmd_vel', command, 50)
        node.create_subscription(String, '/jetbot/status', status, 30)
        start = time.monotonic()
        next_flush = start + .5
        next_space_check = start
        next_report = start

        try:
            while not finished and not (directory / 'stop.requested').exists():
                rclpy.spin_once(node, timeout_sec=.1)
                now = time.monotonic()
                if now - start >= MAX_SECONDS:
                    reason = 'recording time limit'
                    break
                if now >= next_flush:
                    for stream in (*videos.values(), timeline):
                        stream.flush()
                    next_flush = now + .5
                if now >= next_space_check:
                    if shutil.disk_usage(directory).free < MIN_FREE_BYTES:
                        reason = 'insufficient space'
                        break
                    next_space_check = now + 3
                if not ready and now >= next_report:
                    problem = None
                    if (min(frame_counts.values()) < 8 or len(camera_info) < 2
                            or any(now - frame_times.get(side, -10) > .75 for side in SIDES)):
                        problem = 'waiting for fresh stereo video and camera calibration'
                    elif last_status is None or now - last_status_time > .65:
                        problem = 'waiting for motor status'
                    elif (last_status.get('fault') or last_status.get('pwm') != [0, 0]
                          or last_status.get('enabled') is not False
                          or node.count_publishers('/cmd_vel') != 0):
                        problem = 'motors must be idle; close any other keyboard controller'
                    if problem:
                        write_json(directory / 'waiting.json', {'reason': problem})
                    else:
                        for stream in (*videos.values(), timeline):
                            stream.flush()
                        write_json(directory / 'ready.json', {
                            'frames': frame_counts, 'motor_status': last_status,
                        })
                        ready = True
                    next_report = now + .4
        except Exception as error:
            reason = f'{type(error).__name__}: {error}'
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    stop_ns = time.time_ns()
    write_json(directory / 'summary.json', {
        'started_ns': started_ns, 'finished_ns': stop_ns,
        'duration_s': (stop_ns - started_ns) / 1e9, 'ready': ready,
        'stop_reason': reason, 'frames': frame_counts, 'jpeg_bytes': offsets,
        'command_samples': cmd_count, 'motor_status_samples': status_count,
        'final_motor_status': last_status,
    })
    return 0 if reason == 'stop requested' else 1


if __name__ == '__main__':
    raise SystemExit(main())
