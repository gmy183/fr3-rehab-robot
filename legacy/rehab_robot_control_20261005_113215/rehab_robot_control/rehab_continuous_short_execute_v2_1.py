from __future__ import annotations

import argparse
import json
import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class ExecutionError(RuntimeError):
    pass


def duration_msg(seconds):

    from builtin_interfaces.msg import Duration

    sec = int(
        math.floor(seconds)
    )

    nanosec = int(
        round(
            (seconds - sec) * 1e9
        )
    )

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    msg = Duration()

    msg.sec = sec
    msg.nanosec = nanosec

    return msg


class Executor(Node):

    def __init__(self):

        super().__init__(
            "rehab_continuous_short_execute_v2_1"
        )

        self.latest_joint_state = None

        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._joint_cb,
            20,
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fr3_arm_controller/"
            "follow_joint_trajectory",
        )

    def _joint_cb(self, msg):

        self.latest_joint_state = msg

    def live_q(self):

        deadline = time.monotonic() + 3.0

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            msg = self.latest_joint_state

            if msg is None:
                continue

            mapping = {
                name: float(value)
                for name, value
                in zip(
                    msg.name,
                    msg.position,
                )
            }

            names = [
                f"fr3_joint{i}"
                for i in range(1, 8)
            ]

            if all(
                name in mapping
                for name in names
            ):
                return [
                    mapping[name]
                    for name in names
                ]

        raise ExecutionError(
            "live joint state unavailable"
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--plan",
        required=True,
    )

    args = parser.parse_args()

    with open(
        args.plan,
        "r",
        encoding="utf-8",
    ) as f:
        plan = json.load(f)

    if (
        plan.get("schema")
        != "rehab_continuous_short_v2_1"
    ):
        raise ExecutionError(
            "unexpected plan schema"
        )

    if (
        plan.get("execution_eligible")
        is not True
    ):
        raise ExecutionError(
            "plan is not execution eligible"
        )

    if (
        plan.get("execution_enabled")
        is not False
    ):
        raise ExecutionError(
            "unexpected execution flag"
        )

    rclpy.init()

    node = Executor()

    try:

        live = node.live_q()

        planned = plan["start_q"]

        start_error = max(
            abs(a - b)
            for a, b in zip(
                live,
                planned,
            )
        )

        print(
            f"live/planned start q error = "
            f"{start_error:.6f} rad"
        )

        if start_error > 0.003:
            raise ExecutionError(
                "robot moved after planning; "
                "re-plan required"
            )

        if not (
            node.client.wait_for_server(
                timeout_sec=5.0
            )
        ):
            raise ExecutionError(
                "FollowJointTrajectory "
                "server unavailable"
            )

        goal = (
            FollowJointTrajectory.Goal()
        )

        goal.trajectory.joint_names = list(
            plan["joint_names"]
        )

        for item in plan[
            "trajectory_points"
        ]:

            point = JointTrajectoryPoint()

            point.positions = [
                float(v)
                for v in item["positions"]
            ]

            point.velocities = [
                float(v)
                for v in item["velocities"]
            ]

            point.accelerations = [
                float(v)
                for v in item[
                    "accelerations"
                ]
            ]

            point.time_from_start = (
                duration_msg(
                    float(
                        item[
                            "time_from_start_s"
                        ]
                    )
                )
            )

            goal.trajectory.points.append(
                point
            )

        goal.goal_time_tolerance = (
            duration_msg(3.0)
        )

        print(
            f"trajectory points = "
            f"{len(goal.trajectory.points)}"
        )

        print(
            f"planned duration = "
            f"{plan['duration_s']:.3f} s"
        )

        send_future = (
            node.client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            node,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            raise ExecutionError(
                "trajectory goal acceptance "
                "timeout"
            )

        handle = send_future.result()

        if handle is None:
            raise ExecutionError(
                "missing goal handle"
            )

        if not handle.accepted:
            raise ExecutionError(
                "trajectory goal rejected"
            )

        print(
            "[EXEC] trajectory goal accepted"
        )

        result_future = (
            handle.get_result_async()
        )

        timeout = (
            float(plan["duration_s"])
            + 12.0
        )

        rclpy.spin_until_future_complete(
            node,
            result_future,
            timeout_sec=timeout,
        )

        if not result_future.done():

            print(
                "[FAULT] trajectory result timeout"
            )

            cancel_future = (
                handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                node,
                cancel_future,
                timeout_sec=3.0,
            )

            raise ExecutionError(
                "trajectory result timeout"
            )

        wrapped = result_future.result()

        if wrapped is None:
            raise ExecutionError(
                "missing trajectory result"
            )

        status = int(
            wrapped.status
        )

        result = wrapped.result

        print(
            f"ROS action status = {status}"
        )

        print(
            f"trajectory error_code = "
            f"{result.error_code}"
        )

        print(
            f"trajectory error_string = "
            f"{result.error_string!r}"
        )

        if (
            status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise ExecutionError(
                "ROS action did not succeed"
            )

        if result.error_code != 0:
            raise ExecutionError(
                "FollowJointTrajectory "
                "reported controller error"
            )

        print(
            "[PASS] FollowJointTrajectory "
            "completed successfully"
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
            f"[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] NO AUTOMATIC RETRY"
        )

        print(
            "[SAFE] NO AUTOMATIC RETURN"
        )

        raise SystemExit(2)
