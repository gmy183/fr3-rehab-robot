import inspect

from rehab_robot_control import (
    empty_grasp_executor as executor,
)


def test_execute_ptp_requires_expected_current():
    sig = inspect.signature(
        executor.execute_ptp
    )

    assert (
        "expected_current"
        in sig.parameters
    )


def test_freshness_guard_precedes_ptp_send():
    src = inspect.getsource(
        executor.execute_ptp
    )

    refresh_index = src.index(
        "refresh_fr3_state("
    )

    send_index = src.index(
        "client.send_goal_async("
    )

    assert refresh_index < send_index


def test_pose_guard_precedes_ptp_send():
    src = inspect.getsource(
        executor.execute_ptp
    )

    pose_index = src.index(
        "fr3_pose_error("
    )

    send_index = src.index(
        "client.send_goal_async("
    )

    assert pose_index < send_index
