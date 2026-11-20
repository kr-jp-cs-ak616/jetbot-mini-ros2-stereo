import numpy as np

from jetbot_stereo.reference_motion import ReferenceMotionGuard


def panel(distance):
    return {'floor_endpoints_optical': [[-.25, .099, distance], [.25, .099, distance]]}


def check(guard, distance, x, y=0.):
    walls = [panel(distance)] if distance is not None else []
    return guard.check(walls, walls, np.eye(3), np.array([x, y, 0.]))


def test_real_short_motion_with_the_same_partition_is_accepted():
    guard = ReferenceMotionGuard()
    assert check(guard, .91, 0.) is None
    assert check(guard, 1.06, -.15) is None
    assert check(guard, .91, 0.) is None


def test_cumulative_odometry_error_is_held_and_cannot_pass_on_the_next_scan():
    guard = ReferenceMotionGuard()
    assert check(guard, .91, 0.) is None
    assert check(guard, .96, -.15) is None
    assert check(guard, .99, -.25) == 'waiting_for_consistent_odometry'
    assert check(guard, 1.026, -.622) == 'odometry_wall_conflict_reinitialize'
    assert check(guard, .91, 0.) == 'odometry_wall_conflict_reinitialize'


def test_missing_or_ambiguous_wall_cannot_validate_a_pose():
    guard = ReferenceMotionGuard()
    assert check(guard, None, 0.) == 'waiting_for_confirmed_nearby_wall'
    assert check(guard, .91, 0.) is None
    assert check(guard, None, -.05) == 'waiting_for_same_wall'
    walls = [panel(.91), panel(1.2)]
    assert guard.check(walls, walls, np.eye(3), np.zeros(3)) == 'ambiguous_wall_observation'


def test_single_wall_does_not_verify_large_parallel_motion():
    guard = ReferenceMotionGuard()
    assert check(guard, .91, 0.) is None
    assert check(guard, .91, 0., .25) == 'wall_cannot_verify_parallel_motion'
