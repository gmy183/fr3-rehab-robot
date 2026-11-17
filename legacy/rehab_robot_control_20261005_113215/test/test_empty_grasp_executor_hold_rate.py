import inspect

from rehab_robot_control import (
    empty_grasp_executor as executor,
)


def test_node_tracks_last_xhand_refresh():
    src = inspect.getsource(
        executor.CombinedNode.__init__
    )

    assert (
        "last_xhand_refresh_time"
        in src
    )


def test_fr3_action_hold_is_rate_limited():
    src = inspect.getsource(
        executor.maintain_xhand_once
    )

    assert 'micro["refresh_hz"]' in src
    assert "period = 1.0 / refresh_hz" in src
    assert "last_xhand_refresh_time" in src
    assert "(now - last) >= period" in src


def test_motion_guard_is_not_relaxed():
    src = inspect.getsource(
        executor.check_xhand_sample
    )

    assert "max_abs_joint_motion_rad" in src
    assert "motion guard exceeded" in src
