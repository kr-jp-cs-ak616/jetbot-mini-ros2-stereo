#!/usr/bin/env python3
"""Replay a recorded stereo route, with an independent live front-clearance check."""
import argparse
import base64
from collections import Counter, deque
from contextlib import suppress
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import sys
from subprocess import PIPE, Popen, TimeoutExpired
from threading import Event, Lock, Thread
from time import monotonic, sleep

import cv2
import numpy as np

from replay_route_vision import RouteVision


PROJECT = Path(__file__).resolve().parents[1]
CAMERAS = ('left', 'right')
COMMANDS = {'stop': (0., 0.), 'forward': (.04, 0.), 'left': (0., .5), 'right': (0., -.5)}
FRAME_AGE = .43


def load_route(report):
    entries = json.loads(report.read_text())['route']
    route = []
    for item in sorted(entries, key=lambda step: step['recorded_start_ns']):
        mode, seconds, label = item['mode'], float(item['seconds']), item['label']
        valid_mode = mode in ('left', 'right', 'forward')
        if not valid_mode or not .1 <= seconds <= (1.65 if mode == 'forward' else .4):
            raise ValueError(f'Invalid recorded movement: {item}')
        if not label or Path(label).name != label:
            raise ValueError('Invalid route label')
        paths = [report.parent / f'{label}-before-{side}.jpg' for side in CAMERAS]
        frames = [cv2.imread(str(path)) for path in paths]
        if any(frame is None or frame.shape != (480, 640, 3) for frame in frames):
            raise ValueError(f'Missing or invalid reference frame: {label}')
        route.append((mode, seconds, label, frames))
    if not route or len(route) > 180:
        raise ValueError('Invalid route length')
    return route


class Link:
    def __init__(self, folder, execute, target, socket, onboard=False):
        bridge = Path(__file__).with_name('replay_route_bridge.py')
        env = dict(os.environ, JETBOT_REPLAY_ENABLE=str(int(execute)))
        if onboard:
            argv = [sys.executable, '-u', str(bridge)]
        else:
            command = shlex.join([
                'docker', 'exec', '-i', '-e', f'JETBOT_REPLAY_ENABLE={int(execute)}',
                'jetbot-mini-ros2', '/jetbot-entrypoint.sh', 'python3', '-u', '-c', bridge.read_text(),
            ])
            argv = ['ssh', '-S', socket, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', target, command]
        self.process = Popen(argv, env=env, stdin=PIPE, stdout=PIPE, text=True, bufsize=1)
        self.folder = folder
        self.frames = {side: deque(maxlen=4) for side in CAMERAS}
        self.motor = None
        self.motor_at = -1.
        self.stopped = None
        self.lock = Lock()
        self.reader_done = Event()
        self.heartbeat_done = Event()
        self.heartbeat = None
        self.requested = ('stop', 0.)
        self.reader = Thread(target=self.read, daemon=True)
        self.reader.start()

    def read(self):
        try:
            with (self.folder / 'frames-and-motor.jsonl').open('w') as log:
                for line in self.process.stdout:
                    now = monotonic()
                    log.write(line)
                    packet = json.loads(line)
                    with self.lock:
                        if 'side' in packet:
                            side = packet['side']
                            if not self.frames[side] or packet['stamp'] > self.frames[side][-1][1]['stamp']:
                                self.frames[side].append((now, packet))
                        elif 'motor' in packet:
                            self.motor, self.motor_at = packet['motor'], now
                        elif 'stopped' in packet:
                            self.stopped = packet['stopped']
        finally:
            self.reader_done.set()

    def recent_pair(self, previous):
        with self.lock:
            options = [(left, right) for left in self.frames['left'] for right in self.frames['right']
                       if min(left[0], right[0]) >= monotonic() - FRAME_AGE
                       and abs(left[1]['stamp'] - right[1]['stamp']) <= 50_000_000]
        if not options:
            return None
        pair = max(options, key=lambda p: min(item[1]['stamp'] for item in p))
        stamps = tuple(packet['stamp'] for _, packet in pair)
        if stamps[0] <= previous[0] or stamps[1] <= previous[1]:
            return None
        images = [cv2.imdecode(np.frombuffer(base64.b64decode(packet['jpg']), np.uint8), cv2.IMREAD_COLOR)
                  for _, packet in pair]
        if any(image is None or image.shape != (480, 640, 3) for image in images):
            return None
        return stamps, min(item[0] for item in pair), images

    def send(self, action, **fields):
        self.process.stdin.write(json.dumps({'action': action, **fields}) + '\n')
        self.process.stdin.flush()

    def velocity(self, mode, deadline=None):
        valid_until = monotonic() + .20
        self.requested = (mode, valid_until if deadline is None else min(valid_until, deadline))

    def arm(self):
        self.send('arm')
        self.heartbeat = Thread(target=self.send_velocity, daemon=True)
        self.heartbeat.start()

    def send_velocity(self):
        while not self.heartbeat_done.is_set():
            mode, valid_until = self.requested
            if monotonic() > valid_until:
                mode = 'stop'
            lin, ang = COMMANDS[mode]
            try:
                self.send('velocity', linear=lin, angular=ang)
            except (BrokenPipeError, ValueError):
                return
            self.heartbeat_done.wait(.055)

    def close(self):
        self.heartbeat_done.set()
        if self.heartbeat is not None:
            self.heartbeat.join(timeout=1)
        with suppress(BrokenPipeError):
            self.send('stop')
        with suppress(BrokenPipeError):
            self.process.stdin.close()
        try:
            self.process.wait(timeout=6)
        except TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=2)
        self.reader.join(timeout=1)
        return self.stopped


def marker_command(target, goal_id=22):
    distance, bearing = target
    if distance < .44:
        raise RuntimeError(f'ID {goal_id} is already inside the minimum approach range')
    if abs(bearing) > 5:
        return 'right' if bearing > 0 else 'left'
    return 'goal' if distance <= .58 else 'forward'


def motion_test(vision, link, marker_pause, stop_file):
    """Make a single short forward pulse using a stationary marker as the reference."""
    def record(**values):
        line = json.dumps(values)
        print(line, flush=True)
        with (link.folder / 'decisions.jsonl').open('a') as stream:
            stream.write(line + '\n')

    def check_link():
        if link.reader_done.is_set() or (stop_file is not None and stop_file.exists()):
            raise RuntimeError('Motion test interrupted')

    def fresh_motor():
        report = link.motor
        if report and monotonic() - link.motor_at <= .4 and not report['fault'] and not report['dry_run']:
            return report
        return None

    stamps = (0, 0)
    armed = False
    try:
        start_deadline = monotonic() + 9
        clear = 0
        while monotonic() < start_deadline:
            check_link()
            pair = link.recent_pair(stamps)
            if pair:
                stamps, frame_time, frames = pair
                report = fresh_motor()
                still = bool(report and report['pwm'] == [0, 0] and not report['enabled'])
                marker_pause.observe(frames[0], stamps[0], still, True, record)
                guard, _ = vision.inspect(frames)
                clear = clear + 1 if guard is None else 0
                pose = marker_pause.result
                if (still and pose and pose['range_m'] >= .52 and clear >= 2
                        and stamps[0] - pose['capture_ns'] <= 350_000_000
                        and monotonic() - frame_time <= FRAME_AGE):
                    break
            sleep(.012)
        else:
            raise RuntimeError('Motion test held: stationary marker and fresh front clearance required')

        link.arm()
        armed = True
        arm_deadline = monotonic() + .8
        while not ((fresh_motor() or {}).get('enabled')):
            check_link()
            if monotonic() >= arm_deadline:
                raise RuntimeError('Motor arm was not confirmed')
            sleep(.012)

        # Acquire a new clearance result with the motors still at zero after arming.
        pair = link.recent_pair(stamps)
        if pair is None:
            raise RuntimeError('No fresh frame after arming')
        stamps, frame_time, frames = pair
        guard, _ = vision.inspect(frames)
        if guard or monotonic() - frame_time > FRAME_AGE:
            raise RuntimeError(f'Motion test held after arming: {guard or "camera frame expired"}')

        record(test='forward', requested_seconds=.24, marker=pose['marker_pause'])
        deadline = monotonic() + .24
        moving = None
        while monotonic() < deadline:
            check_link()
            report = fresh_motor()
            if not report or not report['enabled'] or monotonic() - frame_time > FRAME_AGE:
                break
            link.velocity('forward', deadline)
            if all(value > 0 for value in report['pwm']):
                moving = report['pwm']
            sleep(.012)
        link.velocity('stop')
        record(motion_acknowledged=bool(moving), pwm=moving)
        if not moving:
            raise RuntimeError('No forward PWM observed')
    finally:
        stopped = link.close()
        if armed:
            record(stop_confirmed=bool(stopped and stopped['pwm'] == [0, 0] and not stopped['enabled']),
                   motor_status=stopped)
            if not stopped or stopped['pwm'] != [0, 0] or stopped['enabled']:
                raise RuntimeError('Motor stop acknowledgement missing')


def patrol(vision, link, marker_pause, stop_file):
    """Patrol briefly using the best currently confirmed camera heading."""
    def record(**values):
        line = json.dumps(values)
        print(line, flush=True)
        with (link.folder / 'decisions.jsonl').open('a') as stream:
            stream.write(line + '\n')

    armed = False
    stamps = (0, 0)
    headings = {0: 'forward', -18: 'left', 18: 'right'}
    clear_streak = {angle: 0 for angle in headings}
    last_result = None
    command = 'stop'
    turn_until = 0.
    last_clear = monotonic()
    finish_at = last_clear + 18
    motion_observed = False
    try:
        while monotonic() < finish_at and not link.reader_done.is_set():
            now = monotonic()
            if stop_file is not None and stop_file.exists():
                record(stopped_by='stop request')
                break
            report = link.motor
            if armed and (not report or report['fault'] or now - link.motor_at > .4):
                raise RuntimeError('Motor status unavailable during patrol')
            if report and any(report['pwm']):
                motion_observed = True
            pair = link.recent_pair(stamps)
            if pair:
                stamps, received_at, frames = pair
                marker_pause.observe(frames[0], stamps[0], bool(report and report['pwm'] == [0, 0]),
                                     command == 'stop', record)
                if marker_pause.pending is not None:
                    link.velocity('stop')
                    command = 'stop'
                    turn_until = 0.
                checks, _ = vision.inspect_sectors(frames, tuple(headings))
                now = monotonic()
                if now - received_at > FRAME_AGE:
                    checks = dict.fromkeys(headings, 'camera data expired')
                for angle in headings:
                    clear_streak[angle] = clear_streak[angle] + 1 if checks[angle] is None else 0
                if checks != last_result:
                    record(sectors={headings[angle]: problem or 'clear' for angle, problem in checks.items()})
                    last_result = checks
                if any(count >= 2 for count in clear_streak.values()):
                    last_clear = now
                direction = next((angle for angle in headings if clear_streak[angle] >= 2), None)
                if not armed and direction is not None and marker_pause.pending is None:
                    if not report or now - link.motor_at > .4 or report['enabled'] or report['fault'] or report['dry_run'] or any(report['pwm']):
                        raise RuntimeError('Motor preflight refused')
                    link.arm()
                    armed = True
                if not armed or not report or not report['enabled'] or marker_pause.pending is not None:
                    next_command = 'stop'
                elif turn_until:
                    angle = -18 if command == 'left' else 18
                    next_command = command if now < turn_until and checks[angle] is None else 'stop'
                    if next_command == 'stop':
                        turn_until = 0.
                        clear_streak = dict.fromkeys(headings, 0)
                elif direction is None:
                    next_command = 'stop'
                else:
                    next_command = headings[direction]
                    if direction:
                        turn_until = now + .22
                if next_command != command:
                    record(patrol_command=next_command)
                command = next_command
                link.velocity(command, min(received_at + FRAME_AGE, turn_until or finish_at))
            if now - last_clear > 4.5:
                record(stopped_by='no confirmed travel direction')
                break
            sleep(.012)
        record(patrol_motion_observed=motion_observed)
    finally:
        stopped = link.close()
        if armed:
            confirmed = bool(stopped and not stopped['enabled'] and stopped['pwm'] == [0, 0])
            record(stop_confirmed=confirmed, motor=stopped)
            if not confirmed:
                raise RuntimeError('Patrol stop acknowledgement missing')


def run(route, vision, link, execute, log, inspect_only=False, marker_pause=None, stop_file=None, goal_id=22):
    def record(**data):
        log.write(json.dumps(data) + '\n')
        log.flush()
        print(json.dumps(data, ensure_ascii=False), flush=True)

    stamps = (0, 0)
    frame_time = last_send = 0.
    arm_time = None
    start = monotonic()
    blocked_since = start
    elapsed = 0.
    goal_hits = 0
    clearance_history = deque(maxlen=3)
    index = 0
    last_target = None
    guard = 'waiting for stereo'
    alignment_ready = False
    correction_until = 0.
    correction_mode = 'stop'
    pending = 'stop'
    target_until = 0.
    current = 'stop'
    end = start + 180
    last_report = None
    inspection_started = None
    try:
        while monotonic() < end and not link.reader_done.is_set():
            if stop_file is not None and stop_file.exists():
                record(stopped_by='stop request')
                return
            now = monotonic()
            delta = now - start
            start = now
            motor_moving = link.motor and link.motor['pwm'] != [0, 0]
            if current != 'stop' and motor_moving and correction_mode == 'stop' and not last_target:
                elapsed += delta
            pair = link.recent_pair(stamps)
            if pair:
                stamps, frame_time, frames = pair
                if marker_pause is not None and (arm_time is not None or inspect_only):
                    marker_pause.observe(frames[0], stamps[0], not motor_moving, current == 'stop', record)
                    if marker_pause.pending is not None and not inspect_only:
                        # Refresh zero while the clearance and location calculations run.
                        link.velocity('stop')
                        current = 'stop'
                guard, markers = vision.inspect(frames)
                mode, duration, label, reference = route[min(index, len(route) - 1)]
                target = None
                if goal_id == 25 and index >= 17:
                    target = marker_pause.approach(frames, goal_id)
                elif goal_id == 22 and index >= 25:
                    target = markers.get(goal_id)
                if target is not None:
                    last_target = target
                    target_until = frame_time + FRAME_AGE
                if inspect_only and link.motor:
                    if inspection_started is None:
                        inspection_started = now
                        record(front_guard=guard or 'clear', markers=markers, motor=link.motor)
                    if marker_pause is None or (marker_pause.present and marker_pause.pending is None):
                        return
                    if monotonic() - inspection_started > 4:
                        record(marker_pose='unavailable')
                        return
                    continue
                if target is not None and not (marker_pause and marker_pause.pending is not None):
                    goal_hits = goal_hits + 1 if marker_command(last_target, goal_id) == 'goal' else 0
                    if goal_hits >= 2:
                        record(goal_id=goal_id, range_m=last_target[0], bearing_deg=last_target[1])
                        return
                if not alignment_ready and not last_target:
                    offset = vision.route_alignment(reference, frames[0])
                    if arm_time is None and offset is None:
                        # Wait for a recognizable first frame before accepting the start.
                        pass
                    elif offset is not None and (abs(offset['right_m']) > .16
                            or abs(offset['forward_m']) > .20 or abs(offset['yaw_right_deg']) > 20):
                        raise RuntimeError(f'Route alignment outside limits at {label}: {offset}')
                    else:
                        alignment_ready = True
                        record(step=index + 1, label=label, tracking=offset or 'recorded commands')
                        if offset and mode == 'forward' and abs(offset['yaw_right_deg']) > 7:
                            yaw = offset['yaw_right_deg']
                            correction_mode = 'left' if yaw > 0 else 'right'
                            correction_until = .12 if abs(yaw) <= 10 else .20
                        if offset and mode == 'forward':
                            elapsed = max(-.20, min(.25, offset['forward_m'] / .115))
                summary = (guard, tuple(sorted(markers)))
                if summary != last_report:
                    record(front_guard=guard or 'clear', markers=markers)
                    last_report = summary
                if guard and 'within' in guard:
                    clearance_history.clear()
                clearance_history.append(guard is None)
            now = monotonic()
            if now - frame_time > FRAME_AGE:
                guard = 'stereo frames missing or stale'
                clearance_history.clear()
            cleared = sum(clearance_history)
            if now - link.motor_at > .5 or not link.motor or link.motor.get('fault'):
                if arm_time is not None:
                    raise RuntimeError('Motor status missing or faulted')
            elif arm_time is None and alignment_ready and cleared >= 2:
                if link.motor['enabled'] or link.motor['pwm'] != [0, 0] or link.motor['dry_run']:
                    raise RuntimeError('Motor is not idle')
                if not execute:
                    record(probe='passed', motor=link.motor)
                    return
                link.arm()
                arm_time = now
                last_send = now
                record(motor='arm requested')
            if arm_time is None:
                if now - blocked_since > 9:
                    raise RuntimeError(f'Preflight blocked: {guard or "start image or motor status unavailable"}')
                sleep(.015)
                continue
            if now - arm_time > .9 and not link.motor['enabled']:
                raise RuntimeError('Motor did not confirm arm')
            if cleared < 2 or not link.motor['enabled']:
                pending = 'stop'
                blocked_since = blocked_since or now
                if now - max(blocked_since, arm_time) > 4:
                    raise RuntimeError(f'Forward check did not recover: {guard}')
            else:
                blocked_since = 0.
                if marker_pause is not None and marker_pause.pending is not None:
                    pending = 'stop'
                elif last_target:
                    if now > target_until:
                        raise RuntimeError(f'ID {goal_id} lost during approach')
                    pending = marker_command(last_target, goal_id)
                    if pending == 'goal':
                        pending = 'stop'
                elif correction_mode != 'stop':
                    if current == correction_mode and motor_moving:
                        correction_until -= delta
                    pending = correction_mode if correction_until > 0 else 'stop'
                    if correction_until <= 0:
                        correction_mode = 'stop'
                elif elapsed >= route[index][1]:
                    index += 1
                    if index == len(route) or (goal_id == 25 and index >= 28):
                        raise RuntimeError(f'Recorded path ended before visual confirmation of ID {goal_id}')
                    elapsed = 0.
                    alignment_ready = False
                    pending = route[index][0] if route[index][0] == current else 'stop'
                elif alignment_ready:
                    pending = route[index][0]
            if pending != current or now - last_send >= .075:
                link.velocity(pending)
                current, last_send = pending, now
            sleep(.012)
        raise RuntimeError('Camera link ended or replay time limit reached')
    finally:
        stopped = link.close()
        if execute and arm_time is not None:
            record(stop_confirmed=bool(stopped and stopped['pwm'] == [0, 0] and not stopped['enabled']),
                   motor_status=stopped)
            if not stopped or stopped['pwm'] != [0, 0] or stopped['enabled']:
                raise RuntimeError('Motor stop acknowledgement missing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('route_report', type=Path, help='offline-replay-validation.json')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--offline', action='store_true', help='Inspect all saved camera pairs with no robot connection')
    mode.add_argument('--execute', action='store_true', help='Enable motion after the live preflight (default is stationary)')
    parser.add_argument('--calibration', type=Path, default=PROJECT / 'calibration/aruco-stereo-review')
    mode.add_argument('--inspect', action='store_true', help='Inspect one live pair and motor status without arming')
    mode.add_argument('--motion-test', action='store_true', help='Run one short guarded forward pulse')
    mode.add_argument('--patrol', action='store_true', help='Run a short locally planned patrol with live clearance checks')
    parser.add_argument('--onboard', action='store_true', help='Connect to the camera and motor locally in the Nano container')
    parser.add_argument('--output', type=Path, help='Write the current run to this new directory')
    parser.add_argument('--stop-file', type=Path, help='Stop and disarm when this file appears')
    parser.add_argument('--anchors', type=Path, help='Directory containing ArUco map anchors for stationary pose updates')
    parser.add_argument('--goal-id', type=int, choices=(22, 25), default=22)
    args = parser.parse_args()
    if (args.motion_test or args.patrol) and (not args.onboard or not args.anchors):
        parser.error('--motion-test and --patrol require --onboard and --anchors')
    if args.goal_id == 25 and not args.anchors:
        parser.error('ID 25 requires --anchors')
    cv2.setNumThreads(2)
    vision = RouteVision(args.calibration, guard_scale=.375 if args.onboard else 1.)
    route = load_route(args.route_report)
    if args.offline:
        counts = Counter()
        for movement, _, label, frames in route:
            block, markers = vision.inspect(frames)
            counts[block or 'clear'] += 1
            print(f'{label}: {movement}, {block or "clear"}, markers {sorted(markers)}')
        print(dict(counts))
        return
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder = args.output or args.route_report.parent / f'guarded-replay-{stamp}'
    folder.mkdir()
    marker_pause = None
    if args.anchors:
        from replay_marker_pause import MarkerPause
        marker_pause = MarkerPause(args.anchors, args.calibration)
    print(f'Recording: {folder}', flush=True)
    with (folder / 'decisions.jsonl').open('w') as log:
        link = Link(folder, args.execute or args.motion_test or args.patrol, os.environ.get('JETBOT_SSH_TARGET', 'ros2'),
                    os.environ.get('JETBOT_SSH_CONTROL_PATH', '/tmp/jetbot-small-area'), args.onboard)
        if args.motion_test:
            motion_test(vision, link, marker_pause, args.stop_file)
        elif args.patrol:
            patrol(vision, link, marker_pause, args.stop_file)
        else:
            run(route, vision, link, args.execute, log, args.inspect, marker_pause, args.stop_file, args.goal_id)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyboardInterrupt) as error:
        raise SystemExit(f'Replay ended: {error or "interrupted"}')
