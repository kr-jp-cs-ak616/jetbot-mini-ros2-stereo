"""Pause at a known ArUco marker and record a stationary map pose estimate."""
from collections import deque
import json
import math
from time import monotonic

import cv2
import numpy as np

from aruco_initial_pose import camera_pose
from record_aruco_range_sample import marker_detector, tag_observation
from jetbot_stereo.calibration import load_calibration


class MarkerPause:
    def __init__(self, anchors, calibration):
        self.anchors = {}
        for file in sorted(anchors.glob('aruco-id*-anchor.json')):
            anchor = json.loads(file.read_text())
            identifier = int(anchor['marker_id'])
            values = [float(anchor[key]) for key in ('x_m', 'y_m', 'normal_yaw_deg', 'side_mm')]
            if identifier in self.anchors or not 0 <= identifier < 50 or not all(map(math.isfinite, values)):
                raise ValueError(f'Invalid marker anchor {file}')
            if not 20 <= values[3] <= 300:
                raise ValueError(f'Invalid marker size in {file}')
            self.anchors[identifier] = anchor
        if not self.anchors:
            raise ValueError(f'No marker anchors in {anchors}')
        cal = load_calibration(calibration / 'left.yaml', 640, 480)
        self.k = np.asarray(cal['camera_matrix']['data']).reshape(3, 3)
        self.d = np.asarray(cal['distortion_coefficients']['data'])
        right = load_calibration(calibration / 'right.yaml', 640, 480)
        self.right_k = np.asarray(right['camera_matrix']['data']).reshape(3, 3)
        self.right_d = np.asarray(right['distortion_coefficients']['data'])
        left_rect = np.asarray(cal['rectification_matrix']['data']).reshape(3, 3)
        right_rect = np.asarray(right['rectification_matrix']['data']).reshape(3, 3)
        projection = np.asarray(right['projection_matrix']['data']).reshape(3, 4)
        self.baseline = -projection[0, 3] / projection[0, 0]
        self.right_to_left = left_rect.T @ right_rect
        self.right_origin_left = left_rect.T @ np.array([self.baseline, 0., 0.])
        self.detect = marker_detector()
        self.present = set()
        self.previous = set()
        self.last_seen = {}
        self.pending = None
        self.started = 0.
        self.stationary_since = None
        self.poses = deque(maxlen=12)
        self.ranges = deque(maxlen=12)
        self.result = None

    def approach(self, frames, identifier):
        """Cross-check two image-based range estimates without relying on aligned rows."""
        size_m = float(self.anchors[identifier]['side_mm']) / 1000
        centers = []
        for frame, k, d in zip(frames, (self.k, self.right_k), (self.d, self.right_d)):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            observation = tag_observation(gray, k, identifier, size_m, self.detect, d)
            if observation is None:
                return None
            centers.append(np.asarray(observation['center_optical_m']))
        left, right = centers
        right = self.right_to_left @ right + self.right_origin_left
        if np.linalg.norm(left - right) > .12:
            return None
        center = (left + right - self.right_origin_left) / 2
        x, _, z = center
        return (float(np.linalg.norm(center)), math.degrees(math.atan2(x, z)))

    def observe(self, frame, stamp, motor_stopped, zero_requested, record):
        now = monotonic()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = self.detect(gray)
        identifiers = set(map(int, result[1].ravel())) & self.anchors.keys() if result[1] is not None else set()
        for identifier in identifiers:
            self.last_seen[identifier] = now
        self.present = {identifier for identifier in self.present
                        if now - self.last_seen.get(identifier, 0.) < 1.5}
        new = (identifiers & self.previous) - self.present
        self.previous = identifiers
        if self.pending is None and new:
            self.pending = min(new)
            self.started = now
            self.stationary_since = None
            self.poses.clear()
            self.ranges.clear()
            self.present.update(new)
            record(marker_pause=self.pending, state='stopping')
            return
        if self.pending is None:
            return
        anchor = self.anchors[self.pending]
        if motor_stopped and zero_requested:
            if self.stationary_since is None:
                self.stationary_since = now
            if now - self.stationary_since >= .12 and self.pending in identifiers:
                observation = tag_observation(gray, self.k, self.pending, float(anchor['side_mm']) / 1000,
                                              lambda _: result, self.d)
                if observation is not None:
                    try:
                        self.poses.append(camera_pose(observation, anchor))
                        self.ranges.append(float(observation['range_m']))
                    except ValueError:
                        pass
        else:
            self.stationary_since = None
            self.poses.clear()
            self.ranges.clear()
        if self.stationary_since is not None and now - self.stationary_since >= .65 and len(self.poses) >= 3:
            positions = np.asarray(self.poses)
            center = np.median(positions[:, :2], axis=0)
            yaws = positions[:, 2]
            yaw = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())
            yaw_error = np.arctan2(np.sin(yaws-yaw), np.cos(yaws-yaw))
            if np.linalg.norm(positions[:, :2] - center, axis=1).max() <= .12 and abs(yaw_error).max() <= .20:
                self.result = dict(marker_pause=self.pending, state='pose_estimated', capture_ns=stamp,
                                   x_m=float(center[0]), y_m=float(center[1]), yaw_deg=math.degrees(yaw),
                                   range_m=float(np.median(self.ranges)), samples=len(self.poses),
                                   provisional=bool(anchor.get('provisional', False)))
                record(**self.result)
                self.pending = None
                return
        if now - self.started > 2.0:
            record(marker_pause=self.pending, state='pose_unavailable', samples=len(self.poses))
            self.pending = None
