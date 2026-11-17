from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    load_joint_limits,
)

from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


SERVICE_NAME = "/compute_cartesian_path"

TEST_DISTANCE_M = 0.005
MAX_STEP_M = 0.001

MIN_FRACTION = 0.995

VELOCITY_SCALE = 0.05
ACCELERATION_SCALE = 0.05

MAX_CARTESIAN_SPEED_M_S = 0.010


class CartesianPlanOnlyError(
    RuntimeError
):
    pass


def duration_seconds(
    value,
):
    return (
        float(value.sec)
        + float(value.nanosec) * 1e-9
    )


class PlanOnlyNode(Node):

    def __init__(self):
        super().__init__(
            "rehab_cartesian_plan_only"
        )

        self.latest_joint_state = None
        self.latest_joint_time = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            20,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = (
            TransformListener(
                self.tf_buffer,
                self,
            )
        )

        self.client = self.create_client(
            GetCartesianPath,
            SERVICE_NAME,
        )

    def _joint_callback(
        self,
        msg,
    ):
        names = set(
            msg.name
        )

        if not set(
            JOINT_NAMES
        ).issubset(names):
            return

        self.latest_joint_state = msg
        self.latest_joint_time = (
            time.monotonic()
        )


def wait_joint_state(
    node,
    timeout_s=3.0,
):
    deadline = (
        time.monotonic()
        + timeout_s
    )

    while (
        time.monotonic()
        < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        if (
            node.latest_joint_state
            is not None
        ):
            age = (
                time.monotonic()
                - node.latest_joint_time
            )

            if age <= 0.10:
                return (
                    node.latest_joint_state
                )

    raise CartesianPlanOnlyError(
        "fresh FR3 /joint_states unavailable"
    )


def ordered_joint_positions(
    msg,
):
    mapping = dict(
        zip(
            msg.name,
            msg.position,
        )
    )

    values = tuple(
        float(
            mapping[name]
        )
        for name in JOINT_NAMES
    )

    if not all(
        math.isfinite(v)
        for v in values
    ):
        raise CartesianPlanOnlyError(
            "non-finite FR3 joint state"
        )

    return values


def wait_transform(
    node,
    base,
    tip,
    timeout_s=3.0,
):
    deadline = (
        time.monotonic()
        + timeout_s
    )

    while (
        time.monotonic()
        < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        try:
            return (
                node.tf_buffer
                .lookup_transform(
                    base,
                    tip,
                    Time(),
                )
            )

        except Exception:
            pass

    raise CartesianPlanOnlyError(
        f"TF unavailable: {base} -> {tip}"
    )


def validate_current_joint_limits(
    q,
):
    limits = load_joint_limits()

    for name, value in zip(
        JOINT_NAMES,
        q,
    ):
        item = limits[name]

        lower = float(
            item["safe_lower_rad"]
        )

        upper = float(
            item["safe_upper_rad"]
        )

        if not (
            lower
            <= value
            <= upper
        ):
            raise CartesianPlanOnlyError(
                f"current {name}={value:.6f} "
                "outside project safe limits"
            )


def build_request(
    *,
    topology,
    joint_state,
    transform,
):
    req = (
        GetCartesianPath.Request()
    )

    req.header.frame_id = (
        topology.base_link
    )

    req.group_name = (
        topology.planning_group
    )

    req.link_name = (
        topology.tip_link
    )

    q = ordered_joint_positions(
        joint_state
    )

    req.start_state.joint_state.name = (
        list(
            JOINT_NAMES
        )
    )

    req.start_state.joint_state.position = (
        list(q)
    )

    req.start_state.is_diff = False

    current = Pose()

    current.position.x = float(
        transform.transform.translation.x
    )

    current.position.y = float(
        transform.transform.translation.y
    )

    current.position.z = float(
        transform.transform.translation.z
    )

    current.orientation.x = float(
        transform.transform.rotation.x
    )

    current.orientation.y = float(
        transform.transform.rotation.y
    )

    current.orientation.z = float(
        transform.transform.rotation.z
    )

    current.orientation.w = float(
        transform.transform.rotation.w
    )

    target = Pose()

    target.position.x = (
        current.position.x
    )

    target.position.y = (
        current.position.y
    )

    target.position.z = (
        current.position.z
        + TEST_DISTANCE_M
    )

    target.orientation.x = (
        current.orientation.x
    )

    target.orientation.y = (
        current.orientation.y
    )

    target.orientation.z = (
        current.orientation.z
    )

    target.orientation.w = (
        current.orientation.w
    )

    req.waypoints = [
        target
    ]

    req.max_step = (
        MAX_STEP_M
    )

    req.jump_threshold = 0.0

    if hasattr(
        req,
        "prismatic_jump_threshold",
    ):
        req.prismatic_jump_threshold = (
            0.0
        )

    if hasattr(
        req,
        "revolute_jump_threshold",
    ):
        req.revolute_jump_threshold = (
            0.0
        )

    req.avoid_collisions = True

    if hasattr(
        req,
        "max_velocity_scaling_factor",
    ):
        req.max_velocity_scaling_factor = (
            VELOCITY_SCALE
        )

    if hasattr(
        req,
        "max_acceleration_scaling_factor",
    ):
        req.max_acceleration_scaling_factor = (
            ACCELERATION_SCALE
        )

    if hasattr(
        req,
        "cartesian_speed_limited_link",
    ):
        req.cartesian_speed_limited_link = (
            topology.tip_link
        )

    if hasattr(
        req,
        "max_cartesian_speed",
    ):
        req.max_cartesian_speed = (
            MAX_CARTESIAN_SPEED_M_S
        )

    return (
        req,
        current,
        target,
    )


def validate_solution(
    response,
):
    if (
        response.error_code.val
        != MoveItErrorCodes.SUCCESS
    ):
        raise CartesianPlanOnlyError(
            "MoveIt Cartesian planning "
            f"failed: error_code="
            f"{response.error_code.val}"
        )

    fraction = float(
        response.fraction
    )

    if fraction < MIN_FRACTION:
        raise CartesianPlanOnlyError(
            "Cartesian path incomplete: "
            f"fraction={fraction:.6f}"
        )

    trajectory = (
        response.solution
        .joint_trajectory
    )

    names = tuple(
        trajectory.joint_names
    )

    if set(names) != set(
        JOINT_NAMES
    ):
        raise CartesianPlanOnlyError(
            "trajectory joint set mismatch: "
            f"{names}"
        )

    if len(
        trajectory.points
    ) < 2:
        raise CartesianPlanOnlyError(
            "Cartesian trajectory contains "
            "fewer than 2 points"
        )

    index = {
        name: i
        for i, name in enumerate(
            names
        )
    }

    limits = load_joint_limits()

    previous_time = -1.0

    max_velocity = 0.0
    max_step = 0.0

    previous_positions = None

    rows = []

    for point_index, point in enumerate(
        trajectory.points
    ):
        if len(
            point.positions
        ) != len(names):
            raise CartesianPlanOnlyError(
                "trajectory position "
                "dimension mismatch"
            )

        t = duration_seconds(
            point.time_from_start
        )

        if t < previous_time:
            raise CartesianPlanOnlyError(
                "trajectory time is "
                "not monotonic"
            )

        previous_time = t

        ordered = []

        for name in JOINT_NAMES:

            value = float(
                point.positions[
                    index[name]
                ]
            )

            if not math.isfinite(
                value
            ):
                raise CartesianPlanOnlyError(
                    f"{name} trajectory "
                    "position non-finite"
                )

            item = limits[name]

            lower = float(
                item[
                    "safe_lower_rad"
                ]
            )

            upper = float(
                item[
                    "safe_upper_rad"
                ]
            )

            if not (
                lower
                <= value
                <= upper
            ):
                raise CartesianPlanOnlyError(
                    f"{name} trajectory "
                    f"point outside safe limits: "
                    f"{value:.6f}"
                )

            ordered.append(
                value
            )

        if (
            previous_positions
            is not None
        ):
            max_step = max(
                max_step,
                max(
                    abs(a - b)
                    for a, b in zip(
                        ordered,
                        previous_positions,
                    )
                ),
            )

        previous_positions = (
            ordered
        )

        if point.velocities:

            if len(
                point.velocities
            ) != len(names):
                raise CartesianPlanOnlyError(
                    "trajectory velocity "
                    "dimension mismatch"
                )

            max_velocity = max(
                max_velocity,
                max(
                    abs(
                        float(v)
                    )
                    for v in (
                        point.velocities
                    )
                ),
            )

        rows.append(
            {
                "index": point_index,
                "time_s": t,
                "positions": {
                    name: ordered[i]
                    for i, name in enumerate(
                        JOINT_NAMES
                    )
                },
            }
        )

    return {
        "fraction": fraction,
        "point_count": len(
            trajectory.points
        ),
        "duration_s": (
            duration_seconds(
                trajectory.points[-1]
                .time_from_start
            )
        ),
        "max_joint_step_rad": (
            max_step
        ),
        "max_reported_joint_velocity_rad_s": (
            max_velocity
        ),
        "trajectory": rows,
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
        required=True,
    )

    args = parser.parse_args()

    topology = (
        discover_fr3_moveit_topology()
    )

    rclpy.init()

    node = PlanOnlyNode()

    try:
        print("=" * 60)
        print(
            "FR3 CARTESIAN PLAN-ONLY V1"
        )
        print(
            "NO EXECUTION / NO MOTION"
        )
        print("=" * 60)

        if not (
            node.client.wait_for_service(
                timeout_sec=10.0
            )
        ):
            raise CartesianPlanOnlyError(
                f"{SERVICE_NAME} unavailable"
            )

        print(
            "[PASS] GetCartesianPath "
            "service ready"
        )

        joint_state = (
            wait_joint_state(
                node
            )
        )

        q = ordered_joint_positions(
            joint_state
        )

        validate_current_joint_limits(
            q
        )

        print(
            "q_start =",
            [
                round(v, 6)
                for v in q
            ],
        )

        transform = wait_transform(
            node,
            topology.base_link,
            topology.tip_link,
        )

        (
            request,
            current_pose,
            target_pose,
        ) = build_request(
            topology=topology,
            joint_state=joint_state,
            transform=transform,
        )

        print(
            "current flange xyz = "
            f"[{current_pose.position.x:.6f}, "
            f"{current_pose.position.y:.6f}, "
            f"{current_pose.position.z:.6f}]"
        )

        print(
            "target flange xyz  = "
            f"[{target_pose.position.x:.6f}, "
            f"{target_pose.position.y:.6f}, "
            f"{target_pose.position.z:.6f}]"
        )

        print(
            "plan-only delta    = "
            "[0, 0, +0.005] m "
            "in fr3_link0"
        )

        future = (
            node.client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=20.0,
        )

        if not future.done():
            raise CartesianPlanOnlyError(
                "GetCartesianPath timed out"
            )

        response = (
            future.result()
        )

        if response is None:
            raise CartesianPlanOnlyError(
                "GetCartesianPath returned None"
            )

        result = validate_solution(
            response
        )

        output = {
            "mode": "PLAN_ONLY",
            "executed": False,

            "planning_group": (
                topology.planning_group
            ),

            "base_link": (
                topology.base_link
            ),

            "tip_link": (
                topology.tip_link
            ),

            "test_translation_m": {
                "x": 0.0,
                "y": 0.0,
                "z": TEST_DISTANCE_M,
            },

            "start_joint_position": {
                name: q[i]
                for i, name in enumerate(
                    JOINT_NAMES
                )
            },

            "current_flange_pose": {
                "x": current_pose.position.x,
                "y": current_pose.position.y,
                "z": current_pose.position.z,
                "qx": current_pose.orientation.x,
                "qy": current_pose.orientation.y,
                "qz": current_pose.orientation.z,
                "qw": current_pose.orientation.w,
            },

            "target_flange_pose": {
                "x": target_pose.position.x,
                "y": target_pose.position.y,
                "z": target_pose.position.z,
                "qx": target_pose.orientation.x,
                "qy": target_pose.orientation.y,
                "qz": target_pose.orientation.z,
                "qw": target_pose.orientation.w,
            },

            **result,
        }

        Path(
            args.output
        ).write_text(
            json.dumps(
                output,
                indent=2,
            ),
            encoding="utf-8",
        )

        print()
        print(
            f"fraction = "
            f"{result['fraction']:.6f}"
        )

        print(
            f"points   = "
            f"{result['point_count']}"
        )

        print(
            f"duration = "
            f"{result['duration_s']:.6f} s"
        )

        print(
            "max joint step = "
            f"{result['max_joint_step_rad']:.6f} rad"
        )

        print(
            "max reported joint velocity = "
            f"{result['max_reported_joint_velocity_rad_s']:.6f} "
            "rad/s"
        )

        print()
        print(
            "[PASS] REAL-STATE CARTESIAN "
            "PLAN GENERATED"
        )

        print(
            "[SAFE] trajectory was NOT executed"
        )

        print(
            "[SAFE] no FollowJointTrajectory goal sent"
        )

        print(
            "[SAFE] no PTP goal sent"
        )

        print(
            "[SAFE] Cartesian real-execution "
            "gate remains CLOSED"
        )

        print(
            "result_file =",
            args.output,
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    try:
        raise SystemExit(
            main()
        )

    except Exception as exc:
        print(
            "[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] no trajectory "
            "execution path exists "
            "in this module"
        )

        raise SystemExit(2)
