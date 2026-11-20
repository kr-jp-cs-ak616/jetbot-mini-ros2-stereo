"""Check stereo odometry against a repeatedly observed nearby partition."""

import numpy as np

from .wall_tracks import line_model, match_score


class ReferenceMotionGuard:
    """Keep the first verified partition as an independent geometric reference."""

    def __init__(self):
        self.origin = None
        self.reference = None
        self.blocked = False

    @staticmethod
    def nearby_segments(walls, rotation, position):
        segments = []
        for wall in walls:
            optical = np.asarray(wall['floor_endpoints_optical'], dtype=float)
            if optical.shape != (2, 3) or not np.isfinite(optical).all():
                continue
            base = optical[:, [2, 0, 1]] * [1., -1., -1.]
            if not .20 <= np.linalg.norm(base[:, :2].mean(axis=0)) <= 1.65:
                continue
            world = (base @ rotation.T + position)[:, :2]
            if np.linalg.norm(world[1] - world[0]) >= .20:
                segments.append((np.linalg.norm(base[:, :2].mean(axis=0)), world))
        return segments

    def check(self, walls, confirmed, rotation, position):
        if self.blocked:
            return 'odometry_wall_conflict_reinitialize'
        if not np.isfinite(rotation).all() or not np.isfinite(position).all():
            return 'invalid_odometry_transform'
        if self.origin is None:
            self.origin = position.copy()
        displacement = position[:2] - self.origin[:2]
        if self.reference is None:
            # Establish a reference from a stationary start, before any uncertain motion.
            if np.linalg.norm(displacement) > .08:
                return 'waiting_for_stationary_wall_reference'
            segments = self.nearby_segments(confirmed, rotation, position)
            if not segments:
                return 'waiting_for_confirmed_nearby_wall'
            self.reference = min(segments, key=lambda segment: segment[0])[1]
            self.origin = position.copy()
            return None

        tangent, normal, offset = line_model(self.reference)
        matches = []
        for _, segment in self.nearby_segments(walls, rotation, position):
            if match_score(self.reference, segment, angle_deg=10., distance=.85, gap=.12) is None:
                continue
            # Match the observed span along the wall. Check its normal offset separately
            # so a bad odometry translation cannot make the disagreement invisible.
            matches.append(float(segment.mean(axis=0) @ normal + offset))
        if not matches:
            return 'waiting_for_same_wall'
        if np.ptp(matches) > .16:
            return 'ambiguous_wall_observation'
        disagreement = abs(float(np.median(matches)))
        if disagreement > .25 and abs(float(displacement @ normal)) > .22:
            self.blocked = True
            return 'odometry_wall_conflict_reinitialize'
        if disagreement > .14:
            return 'waiting_for_consistent_odometry'
        if abs(float(displacement @ tangent)) > .20:
            return 'wall_cannot_verify_parallel_motion'
        return None
