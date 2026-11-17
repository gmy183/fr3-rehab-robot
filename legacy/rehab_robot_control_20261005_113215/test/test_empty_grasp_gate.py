import pytest

from rehab_robot_control.empty_grasp_dry_run import (
    EmptyGraspExecutionBlocked,
    build_dry_run_plan,
    require_real_execution_gate,
)


def test_dry_run_sequence_is_first_stage_order():
    plan = build_dry_run_plan()

    assert plan["sequence"] == [
        "preflight_readonly",
        "xhand_single_joint_0p005",
        "xhand_whole_hand_0p005",
        "fr3_relative_pregrasp",
        "xhand_grasp",
        "fr3_relative_grasp",
        "fr3_rehab_stage_1",
        "controlled_finish",
    ]


def test_dry_run_has_three_fr3_goals():
    plan = build_dry_run_plan()

    assert len(
        plan["fr3_goals"]
    ) == 3


def test_dry_run_xhand_single_contract():
    plan = build_dry_run_plan()
    single = plan["xhand_single"]

    assert (
        single["active_joints"]
        == ("index_joint1",)
    )
    assert single["delta_rad"] == pytest.approx(
        0.005
    )
    assert single["effort_limit_ma"] == 150


def test_dry_run_whole_group_keeps_excluded_joint_out():
    plan = build_dry_run_plan()
    whole = plan["xhand_whole"]

    assert "index_bend_joint" not in (
        whole["active_joints"]
    )

    assert len(
        whole["active_joints"]
    ) == 9


def test_real_execution_gate_is_still_closed():
    with pytest.raises(
        EmptyGraspExecutionBlocked
    ):
        require_real_execution_gate()
