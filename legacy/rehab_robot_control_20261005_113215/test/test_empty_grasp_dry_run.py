import pytest

from rehab_robot_control.empty_grasp_dry_run import (
    EmptyGraspExecutionBlocked,
    build_dry_run_plan,
    require_real_execution_gate,
)


def test_dry_run_plan():
    plan = build_dry_run_plan()

    assert len(
        plan["fr3_goals"]
    ) == 3

    assert (
        plan["xhand_single"][
            "delta_rad"
        ]
        == pytest.approx(0.005)
    )

    assert (
        plan["xhand_single"][
            "effort_limit_ma"
        ]
        == 150
    )

    assert (
        plan["xhand_whole"][
            "effort_limit_ma"
        ]
        == 220
    )


def test_real_execute_is_fail_closed():
    with pytest.raises(
        EmptyGraspExecutionBlocked,
        match="not verified",
    ):
        require_real_execution_gate()
