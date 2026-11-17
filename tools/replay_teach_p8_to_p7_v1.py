#!/usr/bin/env python3

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


TEACH_FILE = (
    "/home/gu/rehab_robot_ws/teach/"
    "teach_path_8pts_v1.yaml"
)

ACTION = "/action_server/ptp_motion"
SERVICE = "/compute_cartesian_path"

PTP_SPEED = 0.015

MAX_SEGMENT_M = 0.015
START_NEAR_P8_M = 0.010

FINAL_XYZ_TOL_M = 0.005
FINAL_JOINT_TOL_RAD = 0.010

MAX_JOINT_DELTA_RAD = 0.100


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
            "teach_p8_to_p7_v1"
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
                node.mode,
            )

    raise ReplayError(
        "fresh FR3 state unavailable"
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


def load_points():

    with open(
        TEACH_FILE,
        "r",
        encoding="utf-8",
    ) as f:
        data = yaml.safe_load(f)

    profile = data["teach_path"]

    result = {}

    for point in profile["points"]:
        result[
            point["id"]
        ] = tuple(
            float(v)
            for v in point["position_m"]
        )

    return (
        profile["frame"],
        profile["tool"],
        result,
    )


def plan_segment(
    node,
    topology,
    q_start,
    quat,
    target_xyz,
):

    current_xyz, _ = flange_pose(
        node,
        topology.base_link,
        topology.tip_link,
    )

    distance = dist(
        current_xyz,
        target_xyz,
    )

    print(
        f"segment distance = "
        f"{distance*1000:.2f} mm"
    )

    if distance > MAX_SEGMENT_M:
        raise ReplayError(
            "segment exceeds 15 mm"
        )

    request = GetCartesianPath.Request()

    request.header.frame_id = (
        topology.base_link
    )

    request.group_name = (
        topology.planning_group
    )

    request.link_name = (
        topology.tip_link
    )

    request.start_state.joint_state.name = (
        list(JOINT_NAMES)
    )

    request.start_state.joint_state.position = (
        list(q_start)
    )

    request.start_state.is_diff = False

    pose = Pose()

    pose.position.x = target_xyz[0]
    pose.position.y = target_xyz[1]
    pose.position.z = target_xyz[2]

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    request.waypoints = [pose]

    request.max_step = 0.001
    request.jump_threshold = 0.0
    request.avoid_collisions = True

    future = node.cart_client.call_async(
        request
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
        f"fraction = {fraction:.6f}"
    )

    if fraction < 0.995:
        raise ReplayError(
            "Cartesian plan incomplete"
        )

    trajectory = (
        response.solution.joint_trajectory
    )

    if not trajectory.points:
        raise ReplayError(
            "empty trajectory"
        )

    names = tuple(
        trajectory.joint_names
    )

    index = {
        name: i
        for i, name in enumerate(names)
    }

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
        "max joint delta = "
        f"{joint_delta:.6f} rad"
    )

    if (
        joint_delta
        > MAX_JOINT_DELTA_RAD
    ):
        raise ReplayError(
            "planned joint motion too large"
        )

    return q_target


def execute_ptp(
    node,
    client,
    q_target,
    label,
):

    q_now, mode = fresh(node)

    if mode != 1:
        raise ReplayError(
            f"{label}: FR3 not IDLE"
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
        f"[{label}] PTP goal accepted"
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
            cancel = (
                handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                node,
                cancel,
                timeout_sec=2.0,
            )

        except Exception:
            pass

        raise ReplayError(
            f"{label}: result timeout"
        )

    wrapped = result_future.result()

    if wrapped is None:
        raise ReplayError(
            f"{label}: missing result"
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

    error_message = getattr(
        result,
        "error_message",
        "",
    )

    print(
        f"[{label}] ROS status = "
        f"{wrapped.status}"
    )

    print(
        f"[{label}] PTP status = "
        f"{target_status}"
    )

    if error_message:
        print(
            f"[{label}] message = "
            f"{error_message}"
        )

    if (
        wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        raise ReplayError(
            f"{label}: PTP aborted"
        )

    q_final, _ = fresh(node)

    error = max_delta(
        q_final,
        q_target,
    )

    print(
        f"[{label}] joint error = "
        f"{error:.6f} rad"
    )

    if (
        error
        > FINAL_JOINT_TOL_RAD
    ):
        raise ReplayError(
            f"{label}: joint error too large"
        )


def main():

    base, tip, points = (
        load_points()
    )

    p8 = points["P8"]
    p7 = points["P7"]

    midpoint = tuple(
        (a + b) / 2.0
        for a, b in zip(
            p8,
            p7,
        )
    )

    topology = (
        discover_fr3_moveit_topology()
    )

    if (
        base != topology.base_link
        or tip != topology.tip_link
    ):
        raise ReplayError(
            "teach frame mismatch"
        )

    rclpy.init()

    node = ReplayNode()

    try:
        q, mode = fresh(node)

        if mode != 1:
            raise ReplayError(
                f"robot_mode={mode}, "
                "expected IDLE=1"
            )

        current_xyz, quat = (
            flange_pose(
                node,
                base,
                tip,
            )
        )

        start_error = dist(
            current_xyz,
            p8,
        )

        print("=" * 60)
        print("TAUGHT PATH SEGMENT: P8 -> P7")
        print("=" * 60)

        print(
            "current =",
            [
                round(v, 6)
                for v in current_xyz
            ],
        )

        print(
            "P8      =",
            [
                round(v, 6)
                for v in p8
            ],
        )

        print(
            "MID     =",
            [
                round(v, 6)
                for v in midpoint
            ],
        )

        print(
            "P7      =",
            [
                round(v, 6)
                for v in p7
            ],
        )

        print(
            f"current/P8 error = "
            f"{start_error*1000:.3f} mm"
        )

        if (
            start_error
            > START_NEAR_P8_M
        ):
            raise ReplayError(
                "robot is not sufficiently "
                "close to P8"
            )

        if not node.cart_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise ReplayError(
                "GetCartesianPath unavailable"
            )

        client = FR3PTPClient(
            node,
            ACTION,
        )

        if not client.wait_ready(
            timeout_sec=3.0
        ):
            raise ReplayError(
                "PTP server unavailable"
            )

        print()
        print(
            "Path:"
        )

        print(
            "  P8 -> MID (~12 mm)"
        )

        print(
            "  MID -> P7 (~12 mm)"
        )

        print(
            "Orientation = hold current"
        )

        print(
            f"PTP speed = "
            f"{PTP_SPEED:.3f} rad/s"
        )

        print(
            "Fault = STOP, no auto return"
        )

        answer = input(
            "输入 EXECUTE-P8-P7 才执行："
        ).strip()

        if (
            answer
            != "EXECUTE-P8-P7"
        ):
            print(
                "[CANCEL] no motion"
            )

            return 2

        # Segment 1
        print()
        print(
            "========== SEGMENT 1/2 =========="
        )

        q1, mode1 = fresh(node)

        if mode1 != 1:
            raise ReplayError(
                "FR3 not IDLE before segment 1"
            )

        _, quat1 = flange_pose(
            node,
            base,
            tip,
        )

        target1 = plan_segment(
            node,
            topology,
            q1,
            quat1,
            midpoint,
        )

        execute_ptp(
            node,
            client,
            target1,
            "P8-MID",
        )

        xyz1, _ = flange_pose(
            node,
            base,
            tip,
        )

        error1 = dist(
            xyz1,
            midpoint,
        )

        print(
            "MID Cartesian error = "
            f"{error1*1000:.3f} mm"
        )

        if (
            error1
            > FINAL_XYZ_TOL_M
        ):
            raise ReplayError(
                "midpoint Cartesian error too large"
            )

        # Segment 2
        print()
        print(
            "========== SEGMENT 2/2 =========="
        )

        q2, mode2 = fresh(node)

        if mode2 != 1:
            raise ReplayError(
                "FR3 not IDLE before segment 2"
            )

        _, quat2 = flange_pose(
            node,
            base,
            tip,
        )

        target2 = plan_segment(
            node,
            topology,
            q2,
            quat2,
            p7,
        )

        execute_ptp(
            node,
            client,
            target2,
            "MID-P7",
        )

        xyz2, _ = flange_pose(
            node,
            base,
            tip,
        )

        error2 = dist(
            xyz2,
            p7,
        )

        print()
        print(
            "P7 Cartesian error = "
            f"{error2*1000:.3f} mm"
        )

        if (
            error2
            > FINAL_XYZ_TOL_M
        ):
            raise ReplayError(
                "P7 Cartesian error too large"
            )

        print()
        print(
            "[PASS] P8 -> P7 TEACH REPLAY COMPLETE"
        )

        print(
            "[SAFE] robot remains at P7"
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
        print()
        print(
            "[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] stop current task"
        )

        print(
            "[SAFE] no automatic return/recovery"
        )

        raise SystemExit(2)
