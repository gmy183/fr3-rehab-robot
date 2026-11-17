from __future__ import annotations

import argparse
import json
import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)

from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    validate_target,
)


ACTION_NAME = "/action_server/ptp_motion"

MAX_VELOCITY = 0.020

MAX_PLAN_DELTA_RAD = 0.040
MAX_START_DRIFT_RAD = 0.005
MAX_FINAL_ERROR_RAD = 0.010

MAX_START_VELOCITY_RAD_S = 0.010


class EffectTestError(RuntimeError):
    pass


class Node(rclpy.node.Node):

    def __init__(self):
        super().__init__(
            "rehab_cartesian_ptp_effect_test"
        )

        self.state = None
        self.state_time = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            20,
        )

    def callback(
        self,
        msg,
    ):
        if not set(
            JOINT_NAMES
        ).issubset(
            msg.name
        ):
            return

        mapping_pos = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        mapping_vel = dict(
            zip(
                msg.name,
                msg.velocity,
            )
        )

        self.state = (
            tuple(
                float(
                    mapping_pos[name]
                )
                for name in JOINT_NAMES
            ),
            tuple(
                float(
                    mapping_vel.get(
                        name,
                        0.0,
                    )
                )
                for name in JOINT_NAMES
            ),
        )

        self.state_time = (
            time.monotonic()
        )


def fresh_state(
    node,
    timeout=3.0,
):
    deadline = (
        time.monotonic()
        + timeout
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
            node.state is not None
            and node.state_time is not None
            and (
                time.monotonic()
                - node.state_time
            ) <= 0.10
        ):
            return node.state

    raise EffectTestError(
        "fresh FR3 joint state unavailable"
    )


def max_error(
    a,
    b,
):
    return max(
        abs(
            float(x)
            - float(y)
        )
        for x, y in zip(
            a,
            b,
        )
    )


def execute_goal(
    node,
    client,
    target,
    *,
    label,
):
    target = validate_target(
        target
    )

    q_now, velocity = fresh_state(
        node
    )

    max_velocity_now = max(
        abs(v)
        for v in velocity
    )

    if (
        max_velocity_now
        > MAX_START_VELOCITY_RAD_S
    ):
        raise EffectTestError(
            f"{label}: robot not settled: "
            f"max dq={max_velocity_now:.6f} rad/s"
        )

    print(
        f"[{label}] current q =",
        [
            round(v, 6)
            for v in q_now
        ],
    )

    print(
        f"[{label}] target q  =",
        [
            round(v, 6)
            for v in target
        ],
    )

    goal = client.build_goal(
        target,
        maximum_velocity_rad_s=(
            MAX_VELOCITY
        ),
        goal_tolerance=0.001,
    )

    send_future = (
        client.send_goal_async(
            goal,
            execute=True,
        )
    )

    rclpy.spin_until_future_complete(
        node,
        send_future,
        timeout_sec=5.0,
    )

    if not send_future.done():
        raise EffectTestError(
            f"{label}: goal acceptance timeout"
        )

    handle = send_future.result()

    if (
        handle is None
        or not handle.accepted
    ):
        raise EffectTestError(
            f"{label}: PTP goal rejected"
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
        timeout_sec=20.0,
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

        raise EffectTestError(
            f"{label}: PTP result timeout"
        )

    wrapped = (
        result_future.result()
    )

    if (
        wrapped is None
        or wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        status = (
            None
            if wrapped is None
            else wrapped.status
        )

        raise EffectTestError(
            f"{label}: PTP failed: "
            f"status={status}"
        )

    q_final, _ = fresh_state(
        node
    )

    error = max_error(
        q_final,
        target,
    )

    print(
        f"[{label}] final max error = "
        f"{error:.6f} rad"
    )

    if (
        error
        > MAX_FINAL_ERROR_RAD
    ):
        raise EffectTestError(
            f"{label}: final error too large"
        )

    return q_final


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
        plan.get("mode")
        != "PLAN_ONLY"
        or plan.get("executed")
        is not False
    ):
        raise EffectTestError(
            "invalid plan-only result"
        )

    if (
        float(
            plan.get(
                "fraction",
                0.0,
            )
        )
        < 0.995
    ):
        raise EffectTestError(
            "Cartesian plan fraction too low"
        )

    trajectory = (
        plan.get(
            "trajectory",
            []
        )
    )

    if not trajectory:
        raise EffectTestError(
            "planned trajectory empty"
        )

    q_start_map = (
        plan[
            "start_joint_position"
        ]
    )

    q_start = tuple(
        float(
            q_start_map[name]
        )
        for name in JOINT_NAMES
    )

    q_target_map = (
        trajectory[-1][
            "positions"
        ]
    )

    q_target = tuple(
        float(
            q_target_map[name]
        )
        for name in JOINT_NAMES
    )

    validate_target(
        q_start
    )

    validate_target(
        q_target
    )

    planned_delta = max_error(
        q_start,
        q_target,
    )

    print(
        "planned max joint delta = "
        f"{planned_delta:.6f} rad"
    )

    if (
        planned_delta
        > MAX_PLAN_DELTA_RAD
    ):
        raise EffectTestError(
            "5 mm Cartesian endpoint requires "
            "unexpectedly large joint motion"
        )

    rclpy.init()

    node = Node()

    try:
        q_live, velocity = fresh_state(
            node
        )

        drift = max_error(
            q_live,
            q_start,
        )

        print(
            "live/start drift = "
            f"{drift:.6f} rad"
        )

        if (
            drift
            > MAX_START_DRIFT_RAD
        ):
            raise EffectTestError(
                "robot moved after planning; "
                "re-plan required"
            )

        max_v = max(
            abs(v)
            for v in velocity
        )

        if (
            max_v
            > MAX_START_VELOCITY_RAD_S
        ):
            raise EffectTestError(
                "robot is not stationary"
            )

        client = FR3PTPClient(
            node,
            ACTION_NAME,
        )

        if not client.wait_ready(
            timeout_sec=3.0
        ):
            raise EffectTestError(
                "PTP action server unavailable"
            )

        print()
        print(
            "================================"
        )

        print(
            "REAL MOTION READY"
        )

        print(
            "Motion:"
        )

        print(
            "  current pose"
            " -> planned +Z 5 mm endpoint"
        )

        print(
            "  hold 1 s"
        )

        print(
            "  -> original position"
        )

        print(
            "PTP max velocity = "
            "0.020 rad/s"
        )

        print(
            "XHand = no command"
        )

        print(
            "Camera = not used"
        )

        print(
            "================================"
        )

        confirm = input(
            "输入 EXECUTE-5MM 才执行："
        ).strip()

        if (
            confirm
            != "EXECUTE-5MM"
        ):
            print(
                "[CANCEL] no motion sent"
            )

            return 2

        print()
        print(
            "[EXECUTE] outbound"
        )

        execute_goal(
            node,
            client,
            q_target,
            label="OUTBOUND",
        )

        print(
            "[PASS] outbound complete"
        )

        time.sleep(
            1.0
        )

        print()
        print(
            "[EXECUTE] planned return"
        )

        execute_goal(
            node,
            client,
            q_start,
            label="RETURN",
        )

        q_final, _ = fresh_state(
            node
        )

        return_error = max_error(
            q_final,
            q_start,
        )

        print()
        print(
            "return max error = "
            f"{return_error:.6f} rad"
        )

        if (
            return_error
            > MAX_FINAL_ERROR_RAD
        ):
            raise EffectTestError(
                "return verification failed"
            )

        print()
        print(
            "[PASS] 5 mm MOTION EFFECT TEST COMPLETE"
        )

        print(
            "[PASS] returned to captured start"
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
            "[SAFE] no automatic recovery "
            "motion will be sent"
        )

        raise SystemExit(2)
