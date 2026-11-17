"""Thin MoveIt 2 adapter for FR3 rehabilitation tasks.

The task layer communicates only with official MoveIt ROS interfaces.
Planning is supported here; trajectory execution is intentionally not
exposed yet.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import (
    ExecuteTrajectory,
    MoveGroup,
)
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionIK,
)


FR3_JOINT_NAMES = (
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
)


class FR3MoveItClientError(RuntimeError):
    pass


@dataclass(frozen=True)
class MoveItReadiness:
    move_group: bool
    execute_trajectory: bool
    cartesian_path: bool
    position_ik: bool
    motion_plan: bool

    @property
    def planning_ready(self) -> bool:
        return (
            self.move_group
            and self.cartesian_path
            and self.position_ik
            and self.motion_plan
        )

    @property
    def execution_ready(self) -> bool:
        return (
            self.planning_ready
            and self.execute_trajectory
        )


class FR3MoveItClient:
    """Minimal adapter around the official MoveIt 2 ROS API."""

    MOVE_GROUP_ACTION = "/move_action"
    EXECUTE_TRAJECTORY_ACTION = "/execute_trajectory"

    CARTESIAN_PATH_SERVICE = "/compute_cartesian_path"
    POSITION_IK_SERVICE = "/compute_ik"
    MOTION_PLAN_SERVICE = "/plan_kinematic_path"

    PLANNING_GROUP = "fr3_arm"

    def __init__(self, node: Node) -> None:
        self._node = node

        self._move_group = ActionClient(
            node,
            MoveGroup,
            self.MOVE_GROUP_ACTION,
        )

        self._execute_trajectory = ActionClient(
            node,
            ExecuteTrajectory,
            self.EXECUTE_TRAJECTORY_ACTION,
        )

        self._cartesian_path = node.create_client(
            GetCartesianPath,
            self.CARTESIAN_PATH_SERVICE,
        )

        self._position_ik = node.create_client(
            GetPositionIK,
            self.POSITION_IK_SERVICE,
        )

        self._motion_plan = node.create_client(
            GetMotionPlan,
            self.MOTION_PLAN_SERVICE,
        )

    def readiness(self) -> MoveItReadiness:
        """Query endpoint availability without sending any request."""
        return MoveItReadiness(
            move_group=self._move_group.server_is_ready(),
            execute_trajectory=(
                self._execute_trajectory.server_is_ready()
            ),
            cartesian_path=(
                self._cartesian_path.service_is_ready()
            ),
            position_ik=(
                self._position_ik.service_is_ready()
            ),
            motion_plan=(
                self._motion_plan.service_is_ready()
            ),
        )

    @staticmethod
    def validate_joint_target(
        positions: Sequence[float],
    ) -> tuple[float, ...]:

        values = tuple(
            float(value)
            for value in positions
        )

        if len(values) != len(FR3_JOINT_NAMES):
            raise FR3MoveItClientError(
                "FR3 joint target must contain exactly 7 values"
            )

        if not all(
            math.isfinite(value)
            for value in values
        ):
            raise FR3MoveItClientError(
                "FR3 joint target contains non-finite value"
            )

        return values

    def build_joint_plan_goal(
        self,
        target_positions: Sequence[float],
        *,
        tolerance_rad: float = 0.001,
        allowed_planning_time_s: float = 3.0,
        planning_attempts: int = 1,
    ) -> MoveGroup.Goal:
        """Build an official MoveGroup plan-only goal."""

        target = self.validate_joint_target(
            target_positions
        )

        tolerance = float(tolerance_rad)

        if (
            not math.isfinite(tolerance)
            or tolerance <= 0.0
        ):
            raise FR3MoveItClientError(
                "joint tolerance must be finite and > 0"
            )

        planning_time = float(
            allowed_planning_time_s
        )

        if (
            not math.isfinite(planning_time)
            or planning_time <= 0.0
        ):
            raise FR3MoveItClientError(
                "allowed planning time must be finite and > 0"
            )

        attempts = int(planning_attempts)

        if attempts < 1:
            raise FR3MoveItClientError(
                "planning attempts must be >= 1"
            )

        constraints = Constraints()
        constraints.name = "fr3_joint_target"

        for name, position in zip(
            FR3_JOINT_NAMES,
            target,
        ):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0

            constraints.joint_constraints.append(
                joint
            )

        goal = MoveGroup.Goal()

        goal.request.group_name = (
            self.PLANNING_GROUP
        )

        goal.request.num_planning_attempts = (
            attempts
        )

        goal.request.allowed_planning_time = (
            planning_time
        )

        goal.request.goal_constraints = [
            constraints
        ]

        # Safety boundary for the current development stage.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def wait_for_planning_server(
        self,
        *,
        timeout_sec: float = 5.0,
    ) -> bool:
        return self._move_group.wait_for_server(
            timeout_sec=float(timeout_sec)
        )

    def build_joint_move_goal(
        self,
        target_positions: Sequence[float],
        *,
        tolerance_rad: float = 0.001,
        allowed_planning_time_s: float = 3.0,
        planning_attempts: int = 1,
    ) -> MoveGroup.Goal:
        """Build an official MoveGroup plan-and-execute goal."""

        goal = self.build_joint_plan_goal(
            target_positions,
            tolerance_rad=tolerance_rad,
            allowed_planning_time_s=allowed_planning_time_s,
            planning_attempts=planning_attempts,
        )

        goal.planning_options.plan_only = False
        return goal

    def send_joint_move_async(
        self,
        target_positions: Sequence[float],
        *,
        tolerance_rad: float = 0.001,
        allowed_planning_time_s: float = 3.0,
        planning_attempts: int = 1,
        server_timeout_s: float = 5.0,
    ):
        """Send one official MoveGroup plan-and-execute request."""

        if not self.wait_for_planning_server(
            timeout_sec=server_timeout_s
        ):
            raise FR3MoveItClientError(
                "MoveGroup action server unavailable"
            )

        goal = self.build_joint_move_goal(
            target_positions,
            tolerance_rad=tolerance_rad,
            allowed_planning_time_s=allowed_planning_time_s,
            planning_attempts=planning_attempts,
        )

        return self._move_group.send_goal_async(goal)

    def send_joint_plan_async(
        self,
        target_positions: Sequence[float],
        *,
        tolerance_rad: float = 0.001,
        allowed_planning_time_s: float = 3.0,
        planning_attempts: int = 1,
        server_timeout_s: float = 5.0,
    ):
        """Send a MoveGroup PLAN-ONLY request.

        This method cannot execute the returned trajectory.
        """

        if not self.wait_for_planning_server(
            timeout_sec=server_timeout_s
        ):
            raise FR3MoveItClientError(
                "MoveGroup action server unavailable"
            )

        goal = self.build_joint_plan_goal(
            target_positions,
            tolerance_rad=tolerance_rad,
            allowed_planning_time_s=(
                allowed_planning_time_s
            ),
            planning_attempts=planning_attempts,
        )

        return self._move_group.send_goal_async(
            goal
        )
