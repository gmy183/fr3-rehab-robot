#!/usr/bin/env python3

import math
import time

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

P8 = (
    0.259547,
    0.521461,
    0.302780,
)

P7 = (
    0.282329,
    0.515231,
    0.299230,
)

MID = tuple(
    (a + b) / 2.0
    for a, b in zip(P8, P7)
)

PTP_SPEED = 0.015

MAX_START_MID_ERROR_M = 0.005
MAX_SEGMENT_M = 0.015

MAX_JOINT_DELTA_RAD = 0.100

FINAL_XYZ_TOL_M = 0.005
FINAL_JOINT_TOL_RAD = 0.010


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


class Node(Node):

    def __init__(self):
        super().__init__(
            "resume_teach_mid_to_p7"
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

        self.cart = self.create_client(
            GetCartesianPath,
            SERVICE,
        )

    def _joint_cb(self, msg):

        if not set(JOINT_NAMES).issubset(msg.name):
            return

        m = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        self.q = tuple(
            float(m[n])
            for n in JOINT_NAMES
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


def wait_idle(node, timeout=5.0):

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

        print(
            f"[WAIT] robot_mode={mode}, "
            "waiting for IDLE..."
        )

        time.sleep(0.10)

    raise ReplayError(
        "FR3 did not return to IDLE "
        f"within {timeout:.1f}s; "
        f"last robot_mode={last_mode}"
    )


def flange(node):

    tf = node.tf_buffer.lookup_transform(
        "fr3_link0",
        "fr3_link8",
        Time(),
        timeout=Duration(
            seconds=2.0
        ),
    )

    t = tf.transform.translation
    q = tf.transform.rotation

    return (
        (
            float(t.x),
            float(t.y),
            float(t.z),
        ),
        (
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        ),
    )


def plan(node, topology, q_start, quat):

    xyz, _ = flange(node)

    d = dist(
        xyz,
        P7,
    )

    print(
        f"current -> P7 distance = "
        f"{d*1000:.2f} mm"
    )

    if d > MAX_SEGMENT_M:
        raise ReplayError(
            "remaining segment exceeds 15 mm"
        )

    if not node.cart.wait_for_service(
        timeout_sec=5.0
    ):
        raise ReplayError(
            "GetCartesianPath unavailable"
        )

    req = GetCartesianPath.Request()

    req.header.frame_id = "fr3_link0"
    req.group_name = topology.planning_group
    req.link_name = "fr3_link8"

    req.start_state.joint_state.name = (
        list(JOINT_NAMES)
    )

    req.start_state.joint_state.position = (
        list(q_start)
    )

    req.start_state.is_diff = False

    pose = Pose()

    pose.position.x = P7[0]
    pose.position.y = P7[1]
    pose.position.z = P7[2]

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    req.waypoints = [pose]

    req.max_step = 0.001
    req.jump_threshold = 0.0
    req.avoid_collisions = True

    future = node.cart.call_async(req)

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

    if (
        response is None
        or response.error_code.val
        != MoveItErrorCodes.SUCCESS
    ):
        raise ReplayError(
            "Cartesian planning failed"
        )

    print(
        f"fraction = "
        f"{float(response.fraction):.6f}"
    )

    if float(response.fraction) < 0.995:
        raise ReplayError(
            "Cartesian plan incomplete"
        )

    jt = response.solution.joint_trajectory

    if not jt.points:
        raise ReplayError(
            "empty trajectory"
        )

    index = {
        name: i
        for i, name in enumerate(
            jt.joint_names
        )
    }

    q_target = tuple(
        float(
            jt.points[-1].positions[
                index[name]
            ]
        )
        for name in JOINT_NAMES
    )

    q_target = validate_target(
        q_target
    )

    delta = max_delta(
        q_start,
        q_target,
    )

    print(
        f"max joint delta = "
        f"{delta:.6f} rad"
    )

    if delta > MAX_JOINT_DELTA_RAD:
        raise ReplayError(
            "joint displacement too large"
        )

    return q_target


def execute(node, q_target):

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
            "goal acceptance timeout"
        )

    handle = future.result()

    if (
        handle is None
        or not handle.accepted
    ):
        raise ReplayError(
            "PTP goal rejected"
        )

    print(
        "[MID-P7] PTP goal accepted"
    )

    rf = handle.get_result_async()

    rclpy.spin_until_future_complete(
        node,
        rf,
        timeout_sec=30.0,
    )

    if not rf.done():

        try:
            cf = handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                node,
                cf,
                timeout_sec=2.0,
            )

        except Exception:
            pass

        raise ReplayError(
            "PTP result timeout"
        )

    wrapped = rf.result()

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
        f"[MID-P7] ROS status = "
        f"{wrapped.status}"
    )

    print(
        f"[MID-P7] PTP status = "
        f"{target_status}"
    )

    if message:
        print(
            f"[MID-P7] message = "
            f"{message}"
        )

    if (
        wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        raise ReplayError(
            "PTP action failed"
        )

    # Important fix:
    # Action TARGET_REACHED may arrive before
    # robot_mode has settled back to IDLE.
    print(
        "[MID-P7] target reached; "
        "waiting for IDLE settle..."
    )

    q_idle = wait_idle(
        node,
        timeout=5.0,
    )

    print(
        "[PASS] FR3 returned to IDLE"
    )

    return q_idle


def main():

    topology = (
        discover_fr3_moveit_topology()
    )

    rclpy.init()

    node = Node()

    try:

        q_start = wait_idle(
            node,
            timeout=5.0,
        )

        xyz, quat = flange(node)

        mid_error = dist(
            xyz,
            MID,
        )

        p7_error = dist(
            xyz,
            P7,
        )

        print("=" * 60)
        print("RESUME TAUGHT PATH: MID -> P7")
        print("=" * 60)

        print(
            "current =",
            [
                round(v, 6)
                for v in xyz
            ],
        )

        print(
            "MID     =",
            [
                round(v, 6)
                for v in MID
            ],
        )

        print(
            "P7      =",
            [
                round(v, 6)
                for v in P7
            ],
        )

        print(
            f"current/MID error = "
            f"{mid_error*1000:.3f} mm"
        )

        print(
            f"current/P7 distance = "
            f"{p7_error*1000:.3f} mm"
        )

        if (
            mid_error
            > MAX_START_MID_ERROR_M
        ):
            raise ReplayError(
                "robot is no longer near MID; "
                "resume blocked"
            )

        q_target = plan(
            node,
            topology,
            q_start,
            quat,
        )

        q_live = wait_idle(
            node,
            timeout=3.0,
        )

        if (
            max_delta(
                q_live,
                q_start,
            )
            > 0.005
        ):
            raise ReplayError(
                "robot moved after planning"
            )

        print()
        print(
            "Motion = MID -> P7"
        )

        print(
            "Orientation = hold current"
        )

        print(
            f"PTP speed = "
            f"{PTP_SPEED:.3f} rad/s"
        )

        print(
            "NO automatic return"
        )

        answer = input(
            "输入 EXECUTE-MID-P7 才执行："
        ).strip()

        if (
            answer
            != "EXECUTE-MID-P7"
        ):
            print(
                "[CANCEL] no motion"
            )

            return 2

        q_final = execute(
            node,
            q_target,
        )

        xyz_final, _ = flange(node)

        joint_error = max_delta(
            q_final,
            q_target,
        )

        xyz_error = dist(
            xyz_final,
            P7,
        )

        print()
        print(
            f"final joint error = "
            f"{joint_error:.6f} rad"
        )

        print(
            f"P7 Cartesian error = "
            f"{xyz_error*1000:.3f} mm"
        )

        if (
            joint_error
            > FINAL_JOINT_TOL_RAD
        ):
            raise ReplayError(
                "final joint error too large"
            )

        if (
            xyz_error
            > FINAL_XYZ_TOL_M
        ):
            raise ReplayError(
                "P7 Cartesian error too large"
            )

        print()
        print(
            "[PASS] MID -> P7 COMPLETE"
        )

        print(
            "[PASS] IDLE settle fix verified"
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
            "[SAFE] no automatic return"
        )

        raise SystemExit(2)
