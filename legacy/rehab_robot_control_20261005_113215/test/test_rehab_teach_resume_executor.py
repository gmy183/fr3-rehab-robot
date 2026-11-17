import math

from rehab_robot_control.rehab_teach_resume_executor import (
    project_to_segment,
)


def test_projection_middle():
    t, p, d = project_to_segment(
        (0.5, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )

    assert math.isclose(
        t,
        0.5,
        abs_tol=1e-12,
    )

    assert p == (
        0.5,
        0.0,
        0.0,
    )

    assert math.isclose(
        d,
        0.0,
        abs_tol=1e-12,
    )


def test_projection_cross_track():
    t, _, d = project_to_segment(
        (0.5, 0.002, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )

    assert math.isclose(
        t,
        0.5,
        abs_tol=1e-12,
    )

    assert math.isclose(
        d,
        0.002,
        abs_tol=1e-12,
    )
