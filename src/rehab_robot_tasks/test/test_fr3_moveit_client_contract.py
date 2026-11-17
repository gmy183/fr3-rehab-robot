import ast
import math
from pathlib import Path

import pytest

from rehab_robot_tasks.fr3_moveit_client import (
    FR3_JOINT_NAMES,
    FR3MoveItClient,
    FR3MoveItClientError,
    MoveItReadiness,
)


def test_official_moveit_interface_names():
    assert FR3MoveItClient.MOVE_GROUP_ACTION == "/move_action"

    assert (
        FR3MoveItClient.EXECUTE_TRAJECTORY_ACTION
        == "/execute_trajectory"
    )

    assert (
        FR3MoveItClient.CARTESIAN_PATH_SERVICE
        == "/compute_cartesian_path"
    )

    assert (
        FR3MoveItClient.POSITION_IK_SERVICE
        == "/compute_ik"
    )

    assert (
        FR3MoveItClient.MOTION_PLAN_SERVICE
        == "/plan_kinematic_path"
    )

    assert FR3MoveItClient.PLANNING_GROUP == "fr3_arm"

    assert len(FR3_JOINT_NAMES) == 7


def test_readiness_contract():
    state = MoveItReadiness(
        move_group=True,
        execute_trajectory=False,
        cartesian_path=True,
        position_ik=True,
        motion_plan=True,
    )

    assert state.planning_ready is True
    assert state.execution_ready is False


def test_validate_joint_target():
    values = FR3MoveItClient.validate_joint_target(
        [0.0] * 7
    )

    assert values == (0.0,) * 7


def test_reject_wrong_joint_count():
    with pytest.raises(FR3MoveItClientError):
        FR3MoveItClient.validate_joint_target(
            [0.0] * 6
        )


def test_reject_nonfinite_joint_target():
    bad = [0.0] * 7
    bad[3] = math.nan

    with pytest.raises(FR3MoveItClientError):
        FR3MoveItClient.validate_joint_target(
            bad
        )


def test_task_layer_has_no_forbidden_fr3_backend():
    root = (
        Path.home()
        / "rehab_robot_ws"
        / "src"
        / "rehab_robot_tasks"
        / "rehab_robot_tasks"
    )

    forbidden_modules = {
        "rehab_robot_control",
        "fr3_ptp_client",
        "fr3_ptp_contract",
        "trajectory_manager",
    }

    forbidden_symbols = {
        "FollowJointTrajectory",
        "JointTrajectoryController",
    }

    for path in root.glob("*.py"):
        tree = ast.parse(
            path.read_text(),
            filename=str(path),
        )

        for node in ast.walk(tree):

            if isinstance(node, ast.Import):
                modules = [
                    alias.name
                    for alias in node.names
                ]

            elif isinstance(node, ast.ImportFrom):
                modules = [
                    node.module or ""
                ]

                for alias in node.names:
                    assert (
                        alias.name
                        not in forbidden_symbols
                    )

            else:
                continue

            for module in modules:
                assert not any(
                    module == forbidden
                    or module.startswith(
                        forbidden + "."
                    )
                    or module.endswith(
                        "." + forbidden
                    )
                    for forbidden
                    in forbidden_modules
                )

                assert ".v3" not in module


def test_joint_plan_goal_is_plan_only():
    client = object.__new__(FR3MoveItClient)

    goal = client.build_joint_plan_goal(
        [0.0] * 7
    )

    assert goal.planning_options.plan_only is True
    assert goal.request.group_name == "fr3_arm"
    assert len(goal.request.goal_constraints) == 1
    assert (
        len(
            goal.request
            .goal_constraints[0]
            .joint_constraints
        )
        == 7
    )


def test_joint_move_goal_enables_official_movegroup_execution():
    client = object.__new__(FR3MoveItClient)

    goal = client.build_joint_move_goal(
        [0.0] * 7
    )

    assert goal.planning_options.plan_only is False
    assert goal.request.group_name == "fr3_arm"

    # Execution remains through MoveGroup only.
    assert (
        FR3MoveItClient.MOVE_GROUP_ACTION
        == "/move_action"
    )
