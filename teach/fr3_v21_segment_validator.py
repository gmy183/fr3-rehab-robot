from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import JointState

from rehab_robot_tasks.fr3_moveit_client import (
    FR3_JOINT_NAMES,
    FR3MoveItClient,
)


PATH_FILE = (
    Path.home()
    / "rehab_robot_ws"
    / "teach"
    / "teach_path_8pts_v2_fullpose.yaml"
)

JOINT_TOL_RAD = 0.020

# Functional verification speed.
VELOCITY_SCALING = 0.03
ACCELERATION_SCALING = 0.03

# Robot must already be close to recorded source point.
SOURCE_GATE_RAD = 0.080

# Do not allow planning to wander far outside the
# taught endpoint envelope.
PLAN_ENVELOPE_MARGIN_RAD = 0.080

# Planned joint-space path should not make a large detour.
MAX_PATH_RATIO = 1.50

# Temporary functional final tolerance.
FINAL_GATE_RAD = 0.050

STATE_FRESHNESS_S = 0.25


QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ValidationError(RuntimeError):
    pass


class Client(FR3MoveItClient):
    def plan_scaled_async(
        self,
        target,
        *,
        timeout_sec=5.0,
    ):
        if not self.wait_for_planning_server(
            timeout_sec=timeout_sec
        ):
            raise ValidationError(
                "MoveGroup unavailable"
            )

        goal = self.build_joint_plan_goal(
            target,
            tolerance_rad=JOINT_TOL_RAD,
            allowed_planning_time_s=5.0,
            planning_attempts=3,
        )

        goal.request.max_velocity_scaling_factor = (
            VELOCITY_SCALING
        )

        goal.request.max_acceleration_scaling_factor = (
            ACCELERATION_SCALING
        )

        return self._move_group.send_goal_async(
            goal
        )

    def execute_trajectory_async(
        self,
        trajectory,
    ):
        if not self._execute_trajectory.wait_for_server(
            timeout_sec=5.0
        ):
            raise ValidationError(
                "/execute_trajectory unavailable"
            )

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        return self._execute_trajectory.send_goal_async(
            goal
        )


class NodeImpl(Node):
    def __init__(self):
        super().__init__(
            "fr3_v21_segment_validator"
        )

        self.q = None
        self.rx_time = None

        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._joint_cb,
            QOS,
        )

        self.controller_client = (
            self.create_client(
                ListControllers,
                "/controller_manager/list_controllers",
            )
        )

    def _joint_cb(self, msg):
        table = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        if not all(
            name in table
            for name in FR3_JOINT_NAMES
        ):
            return

        self.q = [
            float(table[name])
            for name in FR3_JOINT_NAMES
        ]

        self.rx_time = time.monotonic()


def spin_until(
    node,
    predicate,
    timeout_s,
):
    end = time.monotonic() + timeout_s

    while (
        rclpy.ok()
        and time.monotonic() < end
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        if predicate():
            return True

    return False


def wait_future(
    node,
    future,
    timeout_s,
):
    return spin_until(
        node,
        lambda: future.done(),
        timeout_s,
    )


def fresh_state(
    node,
    timeout_s=5.0,
):
    ok = spin_until(
        node,
        lambda: (
            node.q is not None
            and node.rx_time is not None
            and (
                time.monotonic()
                - node.rx_time
            ) < STATE_FRESHNESS_S
        ),
        timeout_s,
    )

    if not ok:
        raise ValidationError(
            "fresh /franka/joint_states unavailable"
        )

    return list(node.q)


def require_controllers(node):
    if not node.controller_client.wait_for_service(
        timeout_sec=5.0
    ):
        raise ValidationError(
            "controller manager unavailable"
        )

    future = node.controller_client.call_async(
        ListControllers.Request()
    )

    if not wait_future(
        node,
        future,
        5.0,
    ):
        raise ValidationError(
            "controller query timeout"
        )

    response = future.result()

    states = {
        c.name: c.state
        for c in response.controller
    }

    required = (
        "fr3_arm_controller",
        "joint_state_broadcaster",
        "franka_robot_state_broadcaster",
    )

    print("controller states:")

    for name in required:
        state = states.get(
            name,
            "<missing>",
        )

        print(
            f"  {name}: {state}"
        )

        if state != "active":
            raise ValidationError(
                f"{name} not active"
            )


def load_points():
    root = yaml.safe_load(
        PATH_FILE.read_text(
            encoding="utf-8",
        )
    )["teach_path"]

    points = {
        item["id"]: item
        for item in root["points"]
    }

    if len(points) != 8:
        raise ValidationError(
            "V2.1 path must contain 8 points"
        )

    return points


def max_abs_error(a, b):
    return max(
        abs(x - y)
        for x, y in zip(a, b)
    )


def vector_norm(values):
    return math.sqrt(
        sum(
            value * value
            for value in values
        )
    )


def inspect_trajectory(
    trajectory,
    q_start,
    q_target,
):
    jt = trajectory.joint_trajectory

    if len(jt.points) < 2:
        raise ValidationError(
            "trajectory contains too few points"
        )

    names = list(jt.joint_names)

    if not all(
        name in names
        for name in FR3_JOINT_NAMES
    ):
        raise ValidationError(
            "planned trajectory missing FR3 joints"
        )

    indexes = [
        names.index(name)
        for name in FR3_JOINT_NAMES
    ]

    q_path = []

    times = []

    for point in jt.points:
        q = [
            float(
                point.positions[index]
            )
            for index in indexes
        ]

        q_path.append(q)

        t = (
            float(point.time_from_start.sec)
            + float(
                point.time_from_start.nanosec
            ) * 1e-9
        )

        times.append(t)

    if not all(
        b > a
        for a, b in zip(
            times[:-1],
            times[1:],
        )
    ):
        raise ValidationError(
            "trajectory time not increasing"
        )

    planned_start_error = (
        max_abs_error(
            q_path[0],
            q_start,
        )
    )

    planned_final_error = (
        max_abs_error(
            q_path[-1],
            q_target,
        )
    )

    if planned_start_error > 0.080:
        raise ValidationError(
            "planned start differs from "
            f"live state: {planned_start_error:.6f} rad"
        )

    if planned_final_error > 0.050:
        raise ValidationError(
            "planned final differs from "
            f"target: {planned_final_error:.6f} rad"
        )

    # Endpoint envelope gate.
    worst_excursion = 0.0

    for q in q_path:
        for index in range(7):
            low = min(
                q_start[index],
                q_target[index],
            )

            high = max(
                q_start[index],
                q_target[index],
            )

            if q[index] < low:
                excursion = (
                    low - q[index]
                )

            elif q[index] > high:
                excursion = (
                    q[index] - high
                )

            else:
                excursion = 0.0

            worst_excursion = max(
                worst_excursion,
                excursion,
            )

    if (
        worst_excursion
        > PLAN_ENVELOPE_MARGIN_RAD
    ):
        raise ValidationError(
            "planned joint excursion too large: "
            f"{worst_excursion:.6f} rad"
        )

    direct_vector = [
        b - a
        for a, b in zip(
            q_start,
            q_target,
        )
    ]

    direct_length = vector_norm(
        direct_vector
    )

    path_length = 0.0

    for a, b in zip(
        q_path[:-1],
        q_path[1:],
    ):
        delta = [
            y - x
            for x, y in zip(a, b)
        ]

        path_length += vector_norm(
            delta
        )

    if direct_length < 1e-9:
        path_ratio = 1.0
    else:
        path_ratio = (
            path_length
            / direct_length
        )

    if path_ratio > MAX_PATH_RATIO:
        raise ValidationError(
            "planned path detour too large: "
            f"ratio={path_ratio:.3f}"
        )

    return {
        "points": len(q_path),
        "duration_s": times[-1],
        "start_error": planned_start_error,
        "final_error": planned_final_error,
        "worst_excursion": worst_excursion,
        "direct_length": direct_length,
        "path_length": path_length,
        "path_ratio": path_ratio,
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--from-point",
        required=True,
    )

    parser.add_argument(
        "--to-point",
        required=True,
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    args = parser.parse_args()

    points = load_points()

    if args.from_point not in points:
        raise ValidationError(
            f"unknown source {args.from_point}"
        )

    if args.to_point not in points:
        raise ValidationError(
            f"unknown target {args.to_point}"
        )

    q_source = [
        float(v)
        for v in points[
            args.from_point
        ]["joint_position_rad"]
    ]

    q_target = [
        float(v)
        for v in points[
            args.to_point
        ]["joint_position_rad"]
    ]

    print("=" * 72)
    print(
        "FR3 V2.1 SEGMENT TRAJECTORY VALIDATOR"
    )
    print("=" * 72)

    print(
        f"segment = "
        f"{args.from_point} -> "
        f"{args.to_point}"
    )

    print(
        f"velocity scaling = "
        f"{VELOCITY_SCALING:.3f}"
    )

    print(
        f"acceleration scaling = "
        f"{ACCELERATION_SCALING:.3f}"
    )

    print(
        "source = recorded 7-joint configuration"
    )

    print(
        "target = recorded 7-joint configuration"
    )

    print(
        "backend = official MoveIt 2"
    )

    rclpy.init()

    node = NodeImpl()

    client = Client(node)

    try:
        require_controllers(node)

        q_live = fresh_state(node)

        source_error = max_abs_error(
            q_live,
            q_source,
        )

        print()
        print("========== START ==========")

        print(
            f"source gate error = "
            f"{source_error:.6f} rad"
        )

        if source_error > SOURCE_GATE_RAD:
            raise ValidationError(
                "robot is not at recorded "
                f"{args.from_point}: "
                f"{source_error:.6f} rad > "
                f"{SOURCE_GATE_RAD:.6f} rad"
            )

        print(
            "[PASS] robot is at recorded "
            f"{args.from_point}"
        )

        print()
        print("========== PLAN ==========")

        future = (
            client.plan_scaled_async(
                q_target
            )
        )

        if not wait_future(
            node,
            future,
            8.0,
        ):
            raise ValidationError(
                "MoveGroup plan goal timeout"
            )

        handle = future.result()

        if (
            handle is None
            or not handle.accepted
        ):
            raise ValidationError(
                "MoveGroup plan rejected"
            )

        result_future = (
            handle.get_result_async()
        )

        if not wait_future(
            node,
            result_future,
            20.0,
        ):
            raise ValidationError(
                "MoveGroup plan result timeout"
            )

        wrapped = result_future.result()

        if wrapped is None:
            raise ValidationError(
                "empty planning result"
            )

        result = wrapped.result

        code = int(
            result.error_code.val
        )

        print(
            f"action_status     = "
            f"{wrapped.status}"
        )

        print(
            f"moveit_error_code = "
            f"{code}"
        )

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
            or code
            != MoveItErrorCodes.SUCCESS
        ):
            raise ValidationError(
                "MoveGroup planning failed"
            )

        trajectory = (
            result.planned_trajectory
        )

        audit = inspect_trajectory(
            trajectory,
            q_live,
            q_target,
        )

        print(
            f"trajectory_points = "
            f"{audit['points']}"
        )

        print(
            f"duration          = "
            f"{audit['duration_s']:.3f} s"
        )

        print(
            f"path_ratio        = "
            f"{audit['path_ratio']:.3f}"
        )

        print(
            f"max excursion     = "
            f"{audit['worst_excursion']:.6f} rad"
        )

        print(
            f"planned end error = "
            f"{audit['final_error']:.6f} rad"
        )

        print(
            "[PASS] trajectory audit"
        )

        if not args.execute:
            print()
            print("=" * 72)
            print(
                "[PASS] SEGMENT PLAN-ONLY"
            )
            print(
                "[SAFE] NO FR3 motion sent"
            )
            print("=" * 72)
            return

        token = (
            f"EXECUTE-"
            f"{args.from_point}-"
            f"{args.to_point}"
        )

        print()
        print("=" * 72)
        print("REAL FR3 SEGMENT MOTION")
        print(
            "workspace clear / "
            "operator ready at E-stop"
        )
        print(
            "NO retry / NO auto-return / "
            "NO ErrorRecovery"
        )
        print("=" * 72)

        answer = input(
            f"Type exactly {token} : "
        ).strip()

        if answer != token:
            raise ValidationError(
                "confirmation mismatch"
            )

        # Re-check actual robot state after
        # operator confirmation.
        q_pre = fresh_state(node)

        drift = max_abs_error(
            q_pre,
            q_live,
        )

        if drift > 0.030:
            raise ValidationError(
                "robot moved after planning: "
                f"{drift:.6f} rad"
            )

        require_controllers(node)

        print()
        print(
            "[EXECUTE] exact audited "
            "MoveIt trajectory"
        )

        send_future = (
            client.execute_trajectory_async(
                trajectory
            )
        )

        if not wait_future(
            node,
            send_future,
            8.0,
        ):
            raise ValidationError(
                "execution goal timeout"
            )

        exec_handle = (
            send_future.result()
        )

        if (
            exec_handle is None
            or not exec_handle.accepted
        ):
            raise ValidationError(
                "execution goal rejected"
            )

        print(
            "[PASS] execution goal accepted"
        )

        exec_result_future = (
            exec_handle.get_result_async()
        )

        timeout_s = max(
            30.0,
            audit["duration_s"] * 3.0,
        )

        if not wait_future(
            node,
            exec_result_future,
            timeout_s,
        ):
            raise ValidationError(
                "execution timeout"
            )

        exec_wrapped = (
            exec_result_future.result()
        )

        if exec_wrapped is None:
            raise ValidationError(
                "empty execution result"
            )

        exec_code = int(
            exec_wrapped
            .result
            .error_code
            .val
        )

        print()
        print("========== RESULT ==========")

        print(
            f"action_status     = "
            f"{exec_wrapped.status}"
        )

        print(
            f"moveit_error_code = "
            f"{exec_code}"
        )

        if (
            exec_wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
            or exec_code
            != MoveItErrorCodes.SUCCESS
        ):
            raise ValidationError(
                "trajectory execution failed"
            )

        time.sleep(0.5)

        q_final = fresh_state(node)

        final_error = max_abs_error(
            q_final,
            q_target,
        )

        print(
            f"final max joint error = "
            f"{final_error:.6f} rad"
        )

        if final_error > FINAL_GATE_RAD:
            raise ValidationError(
                "final joint error exceeds "
                f"{FINAL_GATE_RAD:.3f} rad"
            )

        print()
        print("=" * 72)
        print(
            f"[PASS] "
            f"{args.from_point} -> "
            f"{args.to_point} "
            "REAL SEGMENT"
        )
        print(
            "[PASS] official MoveIt trajectory"
        )
        print(
            "[SAFE] no retry"
        )
        print(
            "[SAFE] no auto-return"
        )
        print("=" * 72)

    except Exception as exc:
        print()
        print(
            f"[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] task stopped"
        )

        print(
            "[SAFE] no automatic retry"
        )

        print(
            "[SAFE] no automatic return"
        )

        print(
            "[SAFE] no ErrorRecovery"
        )

        raise SystemExit(2)

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
