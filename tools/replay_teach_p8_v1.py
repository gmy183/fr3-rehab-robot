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

MAX_CART_DIST = 0.020
MAX_JOINT_DELTA = 0.100
PTP_SPEED = 0.015

FINAL_JOINT_TOL = 0.010
FINAL_XYZ_TOL = 0.005


class ReplayError(RuntimeError):
    pass


def dist3(a, b):
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
            "teach_p8_replay_v1"
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

        m = dict(
            zip(msg.name, msg.position)
        )

        self.q = tuple(
            float(m[n])
            for n in JOINT_NAMES
        )

        self.q_time = time.monotonic()

    def _state_cb(self, msg):
        self.mode = int(msg.robot_mode)
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


def pose(node, base, tip):
    tf = node.tf_buffer.lookup_transform(
        base,
        tip,
        Time(),
        timeout=Duration(seconds=2.0),
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


def load_p8():
    with open(
        TEACH_FILE,
        "r",
        encoding="utf-8",
    ) as f:
        d = yaml.safe_load(f)

    profile = d["teach_path"]

    for p in profile["points"]:
        if p["id"] == "P8":
            return (
                profile["frame"],
                profile["tool"],
                tuple(
                    float(v)
                    for v in p["position_m"]
                ),
            )

    raise ReplayError("P8 missing")


def main():

    base, tip, target_xyz = load_p8()

    topology = discover_fr3_moveit_topology()

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
        q_start, mode = fresh(node)

        if mode != 1:
            raise ReplayError(
                f"robot_mode={mode}, expected IDLE=1"
            )

        current_xyz, current_quat = pose(
            node,
            base,
            tip,
        )

        distance = dist3(
            current_xyz,
            target_xyz,
        )

        print("=" * 60)
        print("CURRENT -> TAUGHT P8")
        print("=" * 60)

        print(
            "current xyz =",
            [round(v, 6) for v in current_xyz],
        )

        print(
            "P8 xyz      =",
            [round(v, 6) for v in target_xyz],
        )

        print(
            f"distance    = {distance*1000:.2f} mm"
        )

        if distance > MAX_CART_DIST:
            raise ReplayError(
                "P8 is more than 20 mm away; "
                "single-step replay blocked"
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

        req.start_state.joint_state.name = list(
            JOINT_NAMES
        )

        req.start_state.joint_state.position = list(
            q_start
        )

        req.start_state.is_diff = False

        target = Pose()

        target.position.x = target_xyz[0]
        target.position.y = target_xyz[1]
        target.position.z = target_xyz[2]

        target.orientation.x = current_quat[0]
        target.orientation.y = current_quat[1]
        target.orientation.z = current_quat[2]
        target.orientation.w = current_quat[3]

        req.waypoints = [target]
        req.max_step = 0.001
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = node.cart_client.call_async(req)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=20.0,
        )

        if not future.done():
            raise ReplayError(
                "Cartesian plan timeout"
            )

        response = future.result()

        if (
            response is None
            or response.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            raise ReplayError(
                "Cartesian plan failed"
            )

        fraction = float(response.fraction)

        print(
            f"fraction    = {fraction:.6f}"
        )

        if fraction < 0.995:
            raise ReplayError(
                "Cartesian plan incomplete"
            )

        jt = response.solution.joint_trajectory

        if not jt.points:
            raise ReplayError(
                "trajectory empty"
            )

        names = tuple(jt.joint_names)

        index = {
            n: i
            for i, n in enumerate(names)
        }

        q_target = tuple(
            float(
                jt.points[-1].positions[
                    index[n]
                ]
            )
            for n in JOINT_NAMES
        )

        q_target = validate_target(
            q_target
        )

        joint_delta = max_delta(
            q_start,
            q_target,
        )

        print(
            f"max joint delta = "
            f"{joint_delta:.6f} rad"
        )

        print(
            f"trajectory points = "
            f"{len(jt.points)}"
        )

        if joint_delta > MAX_JOINT_DELTA:
            raise ReplayError(
                "joint motion too large"
            )

        q_live, mode_live = fresh(node)

        if mode_live != 1:
            raise ReplayError(
                "robot left IDLE before send"
            )

        if max_delta(q_live, q_start) > 0.005:
            raise ReplayError(
                "robot moved after planning"
            )

        print()
        print(
            "Orientation: hold current"
        )

        print(
            f"PTP speed: {PTP_SPEED:.3f} rad/s"
        )

        print(
            "NO automatic return"
        )

        answer = input(
            "输入 EXECUTE-P8 才执行："
        ).strip()

        if answer != "EXECUTE-P8":
            print("[CANCEL] no motion")
            return 2

        q_now, mode_now = fresh(node)

        if mode_now != 1:
            raise ReplayError(
                "FR3 is no longer IDLE"
            )

        if max_delta(q_now, q_start) > 0.005:
            raise ReplayError(
                "start state changed"
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

        goal = client.build_goal(
            q_target,
            maximum_velocity_rad_s=(
                PTP_SPEED
            ),
            goal_tolerance=0.001,
        )

        send_future = client.send_goal_async(
            goal,
            execute=True,
        )

        rclpy.spin_until_future_complete(
            node,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            raise ReplayError(
                "PTP acceptance timeout"
            )

        handle = send_future.result()

        if (
            handle is None
            or not handle.accepted
        ):
            raise ReplayError(
                "PTP goal rejected"
            )

        print(
            "[EXECUTE] PTP goal accepted"
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
            raise ReplayError(
                "PTP result timeout"
            )

        wrapped = result_future.result()

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
            "ROS status       =",
            wrapped.status,
        )

        print(
            "PTP target status=",
            target_status,
        )

        if error_message:
            print(
                "PTP message     =",
                error_message,
            )

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise ReplayError(
                "PTP action aborted"
            )

        q_final, _ = fresh(node)

        final_xyz, _ = pose(
            node,
            base,
            tip,
        )

        q_error = max_delta(
            q_final,
            q_target,
        )

        xyz_error = dist3(
            final_xyz,
            target_xyz,
        )

        print()
        print(
            f"final joint error = "
            f"{q_error:.6f} rad"
        )

        print(
            f"final P8 error    = "
            f"{xyz_error*1000:.3f} mm"
        )

        if q_error > FINAL_JOINT_TOL:
            raise ReplayError(
                "joint final error too large"
            )

        if xyz_error > FINAL_XYZ_TOL:
            raise ReplayError(
                "Cartesian final error too large"
            )

        print()
        print(
            "[PASS] P8 TEACH REPLAY COMPLETE"
        )

        print(
            "[SAFE] robot remains at P8"
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    try:
        raise SystemExit(main())

    except Exception as exc:
        print()
        print(
            "[FAULT] "
            f"{type(exc).__name__}: {exc}"
        )

        print(
            "[SAFE] no automatic return"
        )

        raise SystemExit(2)
