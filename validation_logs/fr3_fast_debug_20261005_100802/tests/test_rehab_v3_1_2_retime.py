import math

from rehab_robot_control.v3.fr3_test_runner import (
    QUINTIC_SMOOTHSTEP_AMAX,
    QUINTIC_SMOOTHSTEP_VMAX,
    rest_to_rest_segment_duration,
)


def test_quintic_constants():
    assert math.isclose(QUINTIC_SMOOTHSTEP_VMAX, 1.875, rel_tol=0, abs_tol=1e-12)
    assert 5.77 < QUINTIC_SMOOTHSTEP_AMAX < 5.78


def test_duration_respects_velocity_and_acceleration_bound_formula():
    q0 = [0.0] * 7
    q1 = [0.002] + [0.0] * 6
    t = rest_to_rest_segment_duration(q0, q1, 0.005, 0.025, margin=1.08)
    vmax = QUINTIC_SMOOTHSTEP_VMAX * 0.002 / t
    amax = QUINTIC_SMOOTHSTEP_AMAX * 0.002 / (t * t)
    assert vmax <= 0.005
    assert amax <= 0.025
    assert t > 0.0


def test_duration_nonzero_for_duplicate_knot():
    q = [0.1] * 7
    assert rest_to_rest_segment_duration(q, q, 0.005, 0.025) >= 0.20
