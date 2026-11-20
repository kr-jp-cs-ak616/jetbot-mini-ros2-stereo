"""Sample observed wall segments for the AMCL likelihood-field model."""
from collections import deque
import math

import numpy as np

from .wall_tracks import line_model, match_score

ANGLE_MIN = math.radians(-45)
ANGLE_STEP = math.radians(1)
BEAM_COUNT = 91
RANGE_MIN = 0.15
RANGE_MAX = 5.0
MIN_SCAN_BEAMS = 5


class WallScanConfirmation:
    """Require three matching observations within five recent image frames."""

    def __init__(self):
        self.history = deque(maxlen=4)
        self.last_stamp = -math.inf

    def clear(self):
        self.history.clear()
        self.last_stamp = -math.inf

    def confirm(self, walls, observation_stamp, rotation, translation):
        if not math.isfinite(observation_stamp) or observation_stamp <= self.last_stamp:
            return []
        if observation_stamp - self.last_stamp > .55:
            self.history.clear()
        self.last_stamp = observation_stamp
        while self.history and observation_stamp - self.history[0][0] > 1.2:
            self.history.popleft()

        current = []
        for wall in walls:
            optical = np.asarray(wall['floor_endpoints_optical'], dtype=float)
            if optical.shape != (2, 3):
                continue
            base = np.column_stack((optical[:, 2], -optical[:, 0], -optical[:, 1]))
            endpoints = (base @ rotation.T + translation)[:, :2]
            try:
                line_model(endpoints)
            except ValueError:
                continue
            current.append((wall, endpoints))

        confirmed = []
        for wall, endpoints in current:
            matches = sum(
                any(match_score(endpoints, prev, angle_deg=10., distance=.10, gap=.12)
                    is not None for prev in frame)
                for _, frame in self.history)
            if matches >= 2:
                confirmed.append(wall)
        self.history.append((observation_stamp, [endpoints for _, endpoints in current]))
        return confirmed


def wall_ranges(walls):
    angles = ANGLE_MIN + ANGLE_STEP * np.arange(BEAM_COUNT)
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    ranges = np.full(BEAM_COUNT, np.nan)
    for wall in walls:
        optical = np.asarray(wall['floor_endpoints_optical'])
        a, b = np.column_stack((optical[:, 2], -optical[:, 0]))
        edge = b - a
        denominator = rays[:, 0] * edge[1] - rays[:, 1] * edge[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = (a[0] * edge[1] - a[1] * edge[0]) / denominator
            fraction = (a[0] * rays[:, 1] - a[1] * rays[:, 0]) / denominator
        hits = (fraction >= 0) & (fraction <= 1) & (distance >= RANGE_MIN) & (distance < RANGE_MAX)
        ranges[hits] = np.fmin(ranges[hits], distance[hits])
    return ranges.tolist()
