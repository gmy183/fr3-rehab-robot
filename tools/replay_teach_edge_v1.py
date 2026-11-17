#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import time
import yaml

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from franka_msgs.msg import FrankaRobotState

from rehab_robot_control.fr3_ptp_client import FR3PTPClient
from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    validate_target,
)
from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


ACTION = "/action_server/ptp_motion"
SERVICE = "/compute_cartesian_path"

PTP_SPEED = 0.015

MAX_SEGMENT_M = 0.015
MAX_START_POINT_ERROR_M = 0.005
MAX_SEGMENT_FINAL_ERROR_M = 0.005

MAX_JOINT_DELTA_RAD = 0.100
MAX_FINAL_JOINT_ERROR_RAD = 0.010

MIN_FRACTION = 0.995
IDLE_TIMEOUT_S = 5.0


class ReplayError(RuntimeError):
    pass


def dist(a, b):
    return math.sqrt(
        sum(
            (float(x) - float(y)) ** 2
            for x, y in zip(a, b)
        )
    )


def max_delta(a, b):
    return max(
        abs(float(x) - float(y))
        for x, y in zip(a, b)
    )


class ReplayNode(Node):

    def __init__(self):

        super().__init__(
            "generic_teach_edge_replay_v1"
        )

        self.q = None
        self.q_time = None

        self.mode = None
        self.mode_time = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_cb,
            20,
        )

        self.create_subscription(
            FrankaRobotState,
            "/franka_robot_state_broadcaster/robot_state",
            self._state_cb,
            20,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.cart_client = self.create_client(
            GetCartesianPath,
            SERVICE,
        )

    def _joint_cb(self, msg):

        if not set(JOINT_NAMES).issubset(msg.name):
            return

        mapping = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        self.q = tuple(
            float(mapping[name])
            for name in JOINT_NAMES
        )

        self.q_time = time.monotonic()

    def _state_cb(self, msg):

        self.mode = int(
            msg.robot_mode
        )

        self.mode_time = time.monotonic()


def fresh(node, timeout=3.0):

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:

        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        now = time.monotonic()

        if (
            node.q is not None
            and node.mode is not None
            and node.q_time is not None
            and node.mode_time is not None
            and now - node.q_time <= 0.10
            and now - node.mode_time <= 0.10
        ):
            return (
                validate_target(node.q),
                int(node.mode),
            )

    raise ReplayError(
        "fresh FR3 state unavailable"
    )


def wait_idle(node, timeout=IDLE_TIMEOUT_S):

    deadline = time.monotonic() + timeout
    last_mode = None

    while time.monotonic() < deadline:

        q, mode = fresh(
            node,
            timeout=1.0,
        )

        last_mode = mode

        if mode == 1:
            return q

        if mode == 2:
            print(
                "[WAIT] robot_mode=2 MOVE, "
                "waiting for IDLE..."
            )

        else:
            print(
                f"[WAIT] robot_mode={mode}, "
                "waiting for IDLE..."
            )

        time.sleep(0.10)

    raise ReplayError(
        "FR3 did not return to IDLE; "
        f"last robot_mode={last_mode}"
    )


def flange_pose(node, base, tip):

    tf = node.tf_buffer.lookup_transform(
        base,
        tip,
        Time(),
        timeout=Duration(
            seconds=2.0
        ),
    )

    t = tf.transform.translation
    q = tf.transform.rotation

    xyz = (
        float(t.x),
        float(t.y),
        float(t.z),
    )

    quat = (
        float(q.x),
        float(q.y),
        float(q.z),
        float(q.w),
    )

    return xyz, quat


def load_teach(
    path,
    from_id,
    to_id,
):

    with open(
        path,
        "r",
        encoding="utf-8",
    ) as f:
        data = yaml.safe_load(f)

    profile = data["teach_path"]

    points = {
        p["id"]: tuple(
            float(v)
            for v in p["position_m"]
        )
        for p in profile["points"]
    }

    if from_id not in points:
        raise ReplayError(
            f"missing teach point: {from_id}"
        )

    if to_id not in points:
        raise ReplayError(
            f"missing teach point: {to_id}"
        )

    return (
        profile["frame"],
        profile["tool"],
        points[from_id],
        points[to_id],
    )


def build_targets(
    start_xyz,
    target_xyz,
):

    length = dist(
        start_xyz,
        target_xyz,
    )

    count = max(
        1,
        math.ceil(
            length / MAX_SEGMENT_M
        ),
    )

    targets = []

    for i in range(
        1,
        count + 1,
    ):

        alpha = i / count

        xyz = tuple(
            a + alpha * (b - a)
            for a, b in zip(
                start_xyz,
                target_xyz,
            )
        )

        targets.append(xyz)

    return length, targets


def plan_segment(
    node,
    topology,
    base,
    tip,
    q_start,
    target_xyz,
):

    current_xyz, quat = flange_pose(
        node,
        base,
        tip,
    )

    segment_distance = dist(
        current_xyz,
        target_xyz,
    )

    print(
        "  actual segment distance = "
        f"{segment_distance*1000:.2f} mm"
    )

    if (
        segment_distance
        > MAX_SEGMENT_M + 0.001
    ):
        raise ReplayError(
            "live segment exceeds safety limit"
        )

    if not node.cart_client.wait_for_service(
        timeout_sec=5.0
    ):
        raise ReplayError(
            "GetCartesianPath unavailable"
        )

    req = GetCartesianPath.Request()

    req.header.frame_id = base
    req.group_name = topology.planning_group
    req.link_name = tip

    req.start_state.joint_state.name = (
        list(JOINT_NAMES)
    )

    req.start_state.joint_state.position = (
        list(q_start)
    )

    req.start_state.is_diff = False

    target = Pose()

    target.position.x = target_xyz[0]
    target.position.y = target_xyz[1]
    target.position.z = target_xyz[2]

    target.orientation.x = quat[0]
    target.orientation.y = quat[1]
    target.orientation.z = quat[2]
    target.orientation.w = quat[3]

    req.waypoints = [target]

    req.max_step = 0.001
    req.jump_threshold = 0.0
    req.avoid_collisions = True

    future = node.cart_client.call_async(
        req
    )

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=20.0,
    )

    if not future.done():
        raise ReplayError(
            "Cartesian planning timeout"
        )

    response = future.result()

    if response is None:
        raise ReplayError(
            "Cartesian planner returned None"
        )

    if (
        response.error_code.val
        != MoveItErrorCodes.SUCCESS
    ):
        raise ReplayError(
            "MoveIt planning failed"
        )

    fraction = float(
        response.fraction
    )

    print(
        f"  fraction = {fraction:.6f}"
    )

    if fraction < MIN_FRACTION:
        raise ReplayError(
            "Cartesian segment incomplete"
        )

    trajectory = (
        response.solution.joint_trajectory
    )

    if not trajectory.points:
        raise ReplayError(
            "empty trajectory"
        )

    index = {
        name: i
        for i, name in enumerate(
            trajectory.joint_names
        )
    }

    if not set(
        JOINT_NAMES
    ).issubset(index):
        raise ReplayError(
            "trajectory joint mapping mismatch"
        )

    q_target = tuple(
        float(
            trajectory.points[-1]
            .positions[
                index[name]
            ]
        )
        for name in JOINT_NAMES
    )

    q_target = validate_target(
        q_target
    )

    joint_delta = max_delta(
        q_start,
        q_target,
    )

    print(
        "  max joint delta = "
        f"{joint_delta:.6f} rad"
    )

    if (
        joint_delta
        > MAX_JOINT_DELTA_RAD
    ):
        raise ReplayError(
            "planned joint displacement too large"
        )

    return q_target


def execute_segment(
    node,
    client,
    q_target,
    label,
):

    q_live = wait_idle(
        node
    )

    goal = client.build_goal(
        q_target,
        maximum_velocity_rad_s=(
            PTP_SPEED
        ),
        goal_tolerance=0.001,
    )

    future = client.send_goal_async(
        goal,
        execute=True,
    )

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=5.0,
    )

    if not future.done():
        raise ReplayError(
            f"{label}: acceptance timeout"
        )

    handle = future.result()

    if (
        handle is None
        or not handle.accepted
    ):
        raise ReplayError(
            f"{label}: PTP rejected"
        )

    print(
        f"  [{label}] PTP accepted"
    )

    result_future = (
        handle.get_result_async()
    )

    rclpy.spin_until_future_complete(
        node,
        result_future,
        timeout_sec=30.0,
    )

    if not result_future.done():

        try:
            cancel_future = (
                handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                node,
                cancel_future,
                timeout_sec=2.0,
            )

        except Exception:
            pass

        raise ReplayError(
            f"{label}: PTP timeout"
        )

    wrapped = result_future.result()

    if wrapped is None:
        raise ReplayError(
            f"{label}: result missing"
        )

    result = wrapped.result

    target_status = getattr(
        getattr(
            result,
            "target_status",
            None,
        ),
        "status",
        None,
    )

    message = getattr(
        result,
        "error_message",
        "",
    )

    print(
        f"  [{label}] ROS status = "
        f"{wrapped.status}"
    )

    print(
        f"  [{label}] PTP status = "
        f"{target_status}"
    )

    if message:
        print(
            f"  [{label}] message = "
            f"{message}"
        )

    if (
        wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        raise ReplayError(
            f"{label}: PTP failed"
        )

    print(
        f"  [{label}] target reached, "
        "waiting IDLE..."
    )

    q_final = wait_idle(
        node
    )

    joint_error = max_delta(
        q_final,
        q_target,
    )

    print(
        f"  [{label}] joint error = "
        f"{joint_error:.6f} rad"
    )

    if (
        joint_error
        > MAX_FINAL_JOINT_ERROR_RAD
    ):
        raise ReplayError(
            f"{label}: joint final error too large"
        )

    return q_final


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--teach-file",
        required=True,
    )

    parser.add_argument(
        "--from-point",
        required=True,
    )

    parser.add_argument(
        "--to-point",
        required=True,
    )

    args = parser.parse_args()

    (
        base,
        tip,
        taught_start,
        taught_target,
    ) = load_teach(
        args.teach_file,
        args.from_point,
        args.to_point,
    )

    topology = (
        discover_fr3_moveit_topology()
    )

    if (
        base != topology.base_link
        or tip != topology.tip_link
    ):
        raise ReplayError(
            "teach frames do not match "
            "MoveIt topology"
        )

    rclpy.init()

    node = ReplayNode()

    try:

        q_start = wait_idle(
            node
        )

        live_start, _ = flange_pose(
            node,
            base,
            tip,
        )

        start_error = dist(
            live_start,
            taught_start,
        )

        taught_edge_length = dist(
            taught_start,
            taught_target,
        )

        print("=" * 64)

        print(
            f"GENERIC TEACH EDGE: "
            f"{args.from_point} -> "
            f"{args.to_point}"
        )

        print("=" * 64)

        print(
            "current xyz =",
            [
                round(v, 6)
                for v in live_start
            ],
        )

        print(
            f"{args.from_point} xyz =",
            [
                round(v, 6)
                for v in taught_start
            ],
        )

        print(
            f"{args.to_point} xyz   =",
            [
                round(v, 6)
                for v in taught_target
            ],
        )

        print(
            "start point error = "
            f"{start_error*1000:.3f} mm"
        )

        print(
            "taught edge length = "
            f"{taught_edge_length*1000:.1f} mm"
        )

        if (
            start_error
            > MAX_START_POINT_ERROR_M
        ):
            raise ReplayError(
                "robot is not sufficiently close "
                f"to {args.from_point}"
            )

        # Use actual live position as interpolation
        # start after confirming that it is close
        # to the taught start point.
        live_length, targets = (
            build_targets(
                live_start,
                taught_target,
            )
        )

        print(
            "live path length = "
            f"{live_length*1000:.1f} mm"
        )

        print(
            "subsegments =",
            len(targets),
        )

        print(
            "max nominal segment ≈ "
            f"{live_length/len(targets)*1000:.1f} mm"
        )

        print()

        print(
            "Execution:"
        )

        print(
            f"  {args.from_point}"
            f" -> {args.to_point}"
        )

        print(
            f"  speed = "
            f"{PTP_SPEED:.3f} rad/s"
        )

        print(
            "  orientation = hold current"
        )

        print(
            "  each segment waits for IDLE"
        )

        print(
            "  any fault = STOP"
        )

        print(
            "  no automatic return"
        )

        confirm_string = (
            f"EXECUTE-{args.from_point}-"
            f"{args.to_point}"
        )

        answer = input(
            f"输入 {confirm_string} 才执行："
        ).strip()

        if answer != confirm_string:

            print(
                "[CANCEL] no motion"
            )

            return 2

        client = FR3PTPClient(
            node,
            ACTION,
        )

        if not client.wait_ready(
            timeout_sec=3.0
        ):
            raise ReplayError(
                "PTP action unavailable"
            )

        total_segments = len(
            targets
        )

        for index, target_xyz in enumerate(
            targets,
            start=1,
        ):

            print()

            print(
                "=" * 20
                + f" SEGMENT "
                f"{index}/{total_segments} "
                + "=" * 20
            )

            q_live = wait_idle(
                node
            )

            current_xyz, _ = flange_pose(
                node,
                base,
                tip,
            )

            print(
                "  current =",
                [
                    round(v, 6)
                    for v in current_xyz
                ],
            )

            print(
                "  target  =",
                [
                    round(v, 6)
                    for v in target_xyz
                ],
            )

            q_target = plan_segment(
                node,
                topology,
                base,
                tip,
                q_live,
                target_xyz,
            )

            execute_segment(
                node,
                client,
                q_target,
                f"S{index}",
            )

            achieved_xyz, _ = flange_pose(
                node,
                base,
                tip,
            )

            xyz_error = dist(
                achieved_xyz,
                target_xyz,
            )

            print(
                "  Cartesian error = "
                f"{xyz_error*1000:.3f} mm"
            )

            if (
                xyz_error
                > MAX_SEGMENT_FINAL_ERROR_M
            ):
                raise ReplayError(
                    f"segment {index}: "
                    "Cartesian error too large"
                )

        final_xyz, _ = flange_pose(
            node,
            base,
            tip,
        )

        final_error = dist(
            final_xyz,
            taught_target,
        )

        print()

        print("=" * 64)

        print(
            f"[PASS] {args.from_point} -> "
            f"{args.to_point} COMPLETE"
        )

        print(
            "final target error = "
            f"{final_error*1000:.3f} mm"
        )

        print(
            "[PASS] generic segmented teach "
            "replay works"
        )

        print(
            f"[SAFE] robot remains at "
            f"{args.to_point}"
        )

        print("=" * 64)

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

        print()

        print(
            "[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] current task stopped"
        )

        print(
            "[SAFE] no automatic return"
        )

        raise SystemExit(2)
