#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import yaml
import xacro

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import (
    LaunchDescription,
    LaunchService,
)

from launch_ros.actions import Node


def load_yaml(
    package_name,
    relative_path,
):
    share = Path(
        get_package_share_directory(
            package_name
        )
    )

    path = (
        share
        / relative_path
    )

    data = yaml.safe_load(
        path.read_text(
            encoding="utf-8"
        )
    )

    if not isinstance(
        data,
        dict,
    ):
        raise RuntimeError(
            f"invalid YAML: {path}"
        )

    return data


def render_robot_description(
    robot_ip,
):
    share = Path(
        get_package_share_directory(
            "franka_description"
        )
    )

    urdf_path = (
        share
        / "robots"
        / "fr3"
        / "fr3.urdf.xacro"
    )

    doc = xacro.process_file(
        str(urdf_path),
        mappings={
            "ros2_control": "false",
            "hand": "false",
            "robot_type": "fr3",
            "arm_prefix": "",
            "robot_ip": str(robot_ip),
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
        },
    )

    xml = doc.toxml()

    required = (
        "fr3_joint1",
        "fr3_joint7",
        "fr3_link0",
        "fr3_link8",
    )

    for token in required:
        if token not in xml:
            raise RuntimeError(
                f"URDF missing {token}"
            )

    return xml


def render_robot_semantic():
    share = Path(
        get_package_share_directory(
            "franka_description"
        )
    )

    srdf_path = (
        share
        / "robots"
        / "fr3"
        / "fr3.srdf.xacro"
    )

    doc = xacro.process_file(
        str(srdf_path),
        mappings={
            "hand": "false",
            "arm_prefix": "",
        },
    )

    xml = doc.toxml()

    if "fr3_arm" not in xml:
        raise RuntimeError(
            "SRDF missing fr3_arm"
        )

    return xml


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--robot-ip",
        required=True,
    )

    args = parser.parse_args()

    robot_description = (
        render_robot_description(
            args.robot_ip
        )
    )

    robot_semantic = (
        render_robot_semantic()
    )

    kinematics = load_yaml(
        "franka_fr3_moveit_config",
        "config/kinematics.yaml",
    )

    joint_limits = load_yaml(
        "franka_fr3_moveit_config",
        "config/fr3_joint_limits.yaml",
    )

    ompl = load_yaml(
        "franka_fr3_moveit_config",
        "config/ompl_planning.yaml",
    )

    ompl_pipeline = {
        "move_group": {
            "planning_plugins": [
                "ompl_interface/OMPLPlanner"
            ],

            "request_adapters": [
                "default_planning_request_adapters/"
                "ResolveConstraintFrames",

                "default_planning_request_adapters/"
                "ValidateWorkspaceBounds",

                "default_planning_request_adapters/"
                "CheckStartStateBounds",

                "default_planning_request_adapters/"
                "CheckStartStateCollision",
            ],

            "response_adapters": [
                "default_planning_response_adapters/"
                "AddTimeOptimalParameterization",

                "default_planning_response_adapters/"
                "ValidateSolution",

                "default_planning_response_adapters/"
                "DisplayMotionPath",
            ],

            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_pipeline[
        "move_group"
    ].update(
        ompl
    )

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    safety_execution = {
        # Critical:
        # this node exists only to compute plans.
        "allow_trajectory_execution": False,

        # It must never manipulate ros2_control.
        "moveit_manage_controllers": False,
    }

    node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",

        parameters=[
            {
                "robot_description":
                    robot_description
            },

            {
                "robot_description_semantic":
                    robot_semantic
            },

            {
                "robot_description_kinematics":
                    kinematics
            },

            {
                "robot_description_planning":
                    joint_limits
            },

            ompl_pipeline,
            planning_scene_monitor,
            safety_execution,
        ],
    )

    print(
        "[PASS] FR3 URDF rendered "
        "with empty arm_prefix"
    )

    print(
        "[PASS] FR3 SRDF rendered "
        "with planning group fr3_arm"
    )

    print(
        "[SAFE] ros2_control node "
        "is NOT part of this launch"
    )

    print(
        "[SAFE] trajectory execution "
        "is disabled"
    )

    service = LaunchService()

    service.include_launch_description(
        LaunchDescription(
            [
                node
            ]
        )
    )

    return service.run()


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
