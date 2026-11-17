#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)
from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    PTP_ACTION_TYPE,
    load_joint_limits,
    relative_target,
    validate_target,
)


WS = Path.home() / "rehab_robot_ws"

SEQUENCE = (
    WS
    / "config"
    / "empty_grasp_sequence.yaml"
)

# 当前 FR3 官方 Bringup 已在实机上确认的 Action。
PTP_ACTION_NAME = "/action_server/ptp_motion"

MAX_PREFLIGHT_VELOCITY = 0.020


class PreflightError(RuntimeError):
    pass


class FR3PreflightNode(Node):

    def __init__(self):
        super().__init__(
            "rehab_empty_grasp_preflight"
        )

        self.last_msg = None
        self.arrivals = []

        self.sub = self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._on_joint_state,
            100,
        )

    def _on_joint_state(self, msg):
        self.last_msg = msg

        self.arrivals.append(
            time.monotonic()
        )

        if len(self.arrivals) > 500:
            self.arrivals = (
                self.arrivals[-500:]
            )


def ordered_state(msg):
    names = list(msg.name)

    if len(names) != len(msg.position):
        raise PreflightError(
            "FR3 JointState "
            "name/position length mismatch"
        )

    lookup = {
        name: index
        for index, name
        in enumerate(names)
    }

    missing = [
        name
        for name in JOINT_NAMES
        if name not in lookup
    ]

    if missing:
        raise PreflightError(
            "missing FR3 joints: "
            + ", ".join(missing)
        )

    q = tuple(
        float(
            msg.position[
                lookup[name]
            ]
        )
        for name in JOINT_NAMES
    )

    if (
        msg.velocity
        and len(msg.velocity)
        == len(names)
    ):
        dq = tuple(
            float(
                msg.velocity[
                    lookup[name]
                ]
            )
            for name in JOINT_NAMES
        )
    else:
        dq = tuple(
            0.0
            for _ in JOINT_NAMES
        )

    if not all(
        math.isfinite(v)
        for v in q + dq
    ):
        raise PreflightError(
            "non-finite FR3 state"
        )

    return q, dq


def wait_for_joint_state(node):
    deadline = (
        time.monotonic()
        + 10.0
    )

    while (
        time.monotonic()
        < deadline
        and len(node.arrivals) < 100
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

    if node.last_msg is None:
        raise PreflightError(
            "/franka/joint_states "
            "not received"
        )


def check_ptp_action(node):
    print()
    print(
        "========== FR3 PTP Action =========="
    )

    # 当前实机已经确认官方 PTP Action 路径：
    # /action_server/ptp_motion
    #
    # Preflight 只使用 ActionClient 等待服务端。
    # 此操作不会创建或发送运动目标。

    client = FR3PTPClient(
        node,
        PTP_ACTION_NAME,
    )

    print(
        f"expected action = "
        f"{PTP_ACTION_NAME}"
    )

    print(
        f"expected type   = "
        f"{PTP_ACTION_TYPE}"
    )

    ready = client.wait_ready(
        timeout_sec=10.0
    )

    if not ready:
        raise PreflightError(
            "PTPMotion Action server "
            "not ready at "
            f"{PTP_ACTION_NAME}"
        )

    print(
        f"[PASS] {PTP_ACTION_NAME}"
    )

    print(
        "[PASS] ActionClient server ready"
    )

    print(
        "[PASS] NO PTP GOAL SENT"
    )

    return client


def projected_targets(q, fr3):
    d1 = float(
        fr3[
            "pregrasp_delta_joint1_rad"
        ]
    )

    d2 = float(
        fr3[
            "grasp_delta_joint1_rad"
        ]
    )

    d3 = float(
        fr3[
            "rehab_stage_1"
        ][
            "delta_joint1_rad"
        ]
    )

    q1 = relative_target(
        q,
        joint_index=0,
        delta_rad=d1,
    )

    q2 = relative_target(
        q1,
        joint_index=0,
        delta_rad=d2,
    )

    q3 = relative_target(
        q2,
        joint_index=0,
        delta_rad=d3,
    )

    return q1, q2, q3


def main():
    data = yaml.safe_load(
        SEQUENCE.read_text(
            encoding="utf-8"
        )
    )

    fr3 = data["fr3"]

    rclpy.init()

    node = FR3PreflightNode()

    try:
        # ====================================================
        # 1. 真实 FR3 JointState
        # ====================================================

        wait_for_joint_state(node)

        q, dq = ordered_state(
            node.last_msg
        )

        q = validate_target(q)

        max_velocity = max(
            abs(v)
            for v in dq
        )

        print()
        print(
            "========== FR3 live state =========="
        )

        for name, pos, vel in zip(
            JOINT_NAMES,
            q,
            dq,
        ):
            print(
                f"{name:10s} "
                f"q={pos:+.6f} "
                f"dq={vel:+.6f}"
            )

        print(
            "max |dq| = "
            f"{max_velocity:.6f} rad/s"
        )

        if (
            max_velocity
            > MAX_PREFLIGHT_VELOCITY
        ):
            raise PreflightError(
                "FR3 is moving too fast: "
                f"{max_velocity:.6f} rad/s"
            )

        if len(node.arrivals) >= 20:
            span = (
                node.arrivals[-1]
                - node.arrivals[0]
            )

            if span > 0:
                rate = (
                    len(node.arrivals) - 1
                ) / span

                print(
                    "observed receive rate "
                    f"≈ {rate:.1f} Hz"
                )

                if rate < 100.0:
                    raise PreflightError(
                        "FR3 state receive "
                        f"rate too low: "
                        f"{rate:.1f} Hz"
                    )

        # ====================================================
        # 2. PTP Action
        # ====================================================

        client = check_ptp_action(
            node
        )

        # 保留引用，防止对象在本函数作用域内提前析构。
        assert client is not None

        # ====================================================
        # 3. 真实 q_live → 三个未来相对目标
        #    这里只计算和限位检查，不发送。
        # ====================================================

        q1, q2, q3 = projected_targets(
            q,
            fr3,
        )

        print()
        print(
            "========== LIVE projected targets =========="
        )

        print(
            "以下全部只是计算结果；"
            "没有发送 PTP goal"
        )

        print(
            "q_live =",
            [
                round(v, 6)
                for v in q
            ],
        )

        print(
            "q1 pregrasp =",
            [
                round(v, 6)
                for v in q1
            ],
        )

        print(
            "q2 grasp =",
            [
                round(v, 6)
                for v in q2
            ],
        )

        print(
            "q3 rehab =",
            [
                round(v, 6)
                for v in q3
            ],
        )

        # ====================================================
        # 4. 最终投影姿态安全限位余量
        # ====================================================

        limits = load_joint_limits()

        margins = []

        for name, value in zip(
            JOINT_NAMES,
            q3,
        ):
            item = limits[name]

            lower = float(
                item["safe_lower_rad"]
            )

            upper = float(
                item["safe_upper_rad"]
            )

            margin = min(
                value - lower,
                upper - value,
            )

            margins.append(
                (
                    margin,
                    name,
                )
            )

        min_margin, min_name = min(
            margins
        )

        print()
        print(
            "========== FR3 safe-limit margin =========="
        )

        print(
            "minimum projected margin = "
            f"{min_margin:.6f} rad "
            f"({min_name})"
        )

        if min_margin <= 0.0:
            raise PreflightError(
                "projected FR3 target "
                "has no safe-limit margin"
            )

        print()
        print(
            "[PASS] FR3 LIVE PREFLIGHT"
        )

        print(
            "[PASS] live q used"
        )

        print(
            "[PASS] PTP server ready"
        )

        print(
            "[PASS] projected targets "
            "inside software limits"
        )

        print(
            "[PASS] NO PTP GOAL SENT"
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
            "\n[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}",
            file=sys.stderr,
        )

        raise SystemExit(2)
