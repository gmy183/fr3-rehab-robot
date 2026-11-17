#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
import yaml

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    TransformListener,
)


BASE_FRAME = "fr3_link0"
FLANGE_FRAME = "fr3_link8"


class CalibrationError(RuntimeError):
    pass


def quaternion_to_matrix(
    x,
    y,
    z,
    w,
):
    norm = math.sqrt(
        x*x + y*y + z*z + w*w
    )

    if norm < 1e-12:
        raise CalibrationError(
            "invalid zero quaternion"
        )

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1 - 2*(y*y + z*z),
                2*(x*y - z*w),
                2*(x*z + y*w),
            ],
            [
                2*(x*y + z*w),
                1 - 2*(x*x + z*z),
                2*(y*z - x*w),
            ],
            [
                2*(x*z - y*w),
                2*(y*z + x*w),
                1 - 2*(x*x + y*y),
            ],
        ],
        dtype=float,
    )


def rotation_distance_deg(
    r1,
    r2,
):
    relative = (
        r1.T @ r2
    )

    value = (
        np.trace(relative) - 1.0
    ) / 2.0

    value = float(
        np.clip(
            value,
            -1.0,
            1.0,
        )
    )

    return math.degrees(
        math.acos(value)
    )


class CaptureNode(Node):

    def __init__(self):
        super().__init__(
            "xhand_rehab_tcp_pivot_calibration"
        )

        self.buffer = Buffer()

        self.listener = TransformListener(
            self.buffer,
            self,
            spin_thread=True,
        )

    def capture(self):
        try:
            tf = self.buffer.lookup_transform(
                BASE_FRAME,
                FLANGE_FRAME,
                Time(),
                timeout=Duration(
                    seconds=2.0
                ),
            )

        except Exception as exc:
            raise CalibrationError(
                f"TF unavailable: "
                f"{BASE_FRAME} -> "
                f"{FLANGE_FRAME}: {exc}"
            ) from exc

        tr = tf.transform.translation
        qr = tf.transform.rotation

        t = np.array(
            [
                float(tr.x),
                float(tr.y),
                float(tr.z),
            ],
            dtype=float,
        )

        r = quaternion_to_matrix(
            float(qr.x),
            float(qr.y),
            float(qr.z),
            float(qr.w),
        )

        return r, t


def solve_pivot(
    samples,
):
    if len(samples) < 4:
        raise CalibrationError(
            "at least 4 poses required"
        )

    blocks = []
    rhs = []

    identity = np.eye(3)

    for r, t in samples:

        blocks.append(
            np.hstack(
                (
                    r,
                    -identity,
                )
            )
        )

        rhs.append(
            -t
        )

    a = np.vstack(
        blocks
    )

    b = np.concatenate(
        rhs
    )

    solution, _, rank, singular = (
        np.linalg.lstsq(
            a,
            b,
            rcond=None,
        )
    )

    if rank < 6:
        raise CalibrationError(
            f"pivot system rank deficient: "
            f"rank={rank}/6"
        )

    tcp_flange = (
        solution[:3]
    )

    pivot_base = (
        solution[3:]
    )

    residual_vectors = []

    residual_norms = []

    for r, t in samples:

        predicted = (
            r @ tcp_flange
            + t
        )

        residual = (
            predicted
            - pivot_base
        )

        residual_vectors.append(
            residual
        )

        residual_norms.append(
            float(
                np.linalg.norm(
                    residual
                )
            )
        )

    rms = math.sqrt(
        sum(
            value * value
            for value in residual_norms
        )
        / len(residual_norms)
    )

    maximum = max(
        residual_norms
    )

    return {
        "tcp_flange": tcp_flange,
        "pivot_base": pivot_base,
        "residual_vectors": (
            residual_vectors
        ),
        "residual_norms": (
            residual_norms
        ),
        "rms": rms,
        "max": maximum,
        "rank": int(rank),
        "singular_values": (
            singular
        ),
    }


def orientation_spread(
    samples,
):
    maximum = 0.0

    for i in range(
        len(samples)
    ):
        for j in range(
            i + 1,
            len(samples)
        ):
            angle = (
                rotation_distance_deg(
                    samples[i][0],
                    samples[j][0],
                )
            )

            maximum = max(
                maximum,
                angle,
            )

    return maximum


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--samples",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--output",
        required=True,
    )

    args = parser.parse_args()

    if args.samples < 4:
        raise CalibrationError(
            "--samples must be >= 4"
        )

    print()
    print("=" * 64)
    print("XHAND REHAB TCP PIVOT CALIBRATION")
    print("=" * 64)
    print()
    print("定义：")
    print(
        "  parent = fr3_link8"
    )
    print(
        "  TCP    = 校准尖端 / "
        "未来康复抓持功能点"
    )
    print()
    print("要求：")
    print(
        "  1. 尖端始终保持在同一固定凹点"
    )
    print(
        "  2. 每次明显改变 FR3 姿态"
    )
    print(
        "  3. 不要只平移，要改变末端方向"
    )
    print(
        "  4. 本程序不会发送任何机器人命令"
    )
    print()
    print(
        "输入 q 后回车可随时取消。"
    )
    print()

    rclpy.init()

    node = CaptureNode()

    samples = []

    try:
        time.sleep(
            1.0
        )

        index = 0

        while index < args.samples:

            text = input(
                f"[{index + 1}/{args.samples}] "
                "将尖端保持在固定点，"
                "改变姿态后按 Enter："
            ).strip()

            if text.lower() in (
                "q",
                "quit",
                "exit",
            ):
                print(
                    "[CANCEL] calibration cancelled"
                )

                return 2

            try:
                r, t = node.capture()

            except CalibrationError as exc:
                print(
                    "[RETRY]",
                    exc,
                )

                continue

            samples.append(
                (
                    r,
                    t,
                )
            )

            index += 1

            print(
                "  flange xyz = "
                f"[{t[0]:+.6f}, "
                f"{t[1]:+.6f}, "
                f"{t[2]:+.6f}] m"
            )

        result = solve_pivot(
            samples
        )

        spread_deg = (
            orientation_spread(
                samples
            )
        )

        tcp = result[
            "tcp_flange"
        ]

        pivot = result[
            "pivot_base"
        ]

        rms_mm = (
            result[
                "rms"
            ] * 1000.0
        )

        max_mm = (
            result[
                "max"
            ] * 1000.0
        )

        print()
        print("=" * 64)
        print("CALIBRATION RESULT")
        print("=" * 64)

        print(
            "fr3_link8 -> xhand_rehab_tcp"
        )

        print(
            "translation_m = "
            f"[{tcp[0]:+.9f}, "
            f"{tcp[1]:+.9f}, "
            f"{tcp[2]:+.9f}]"
        )

        print()
        print(
            "fixed pivot in fr3_link0 = "
            f"[{pivot[0]:+.9f}, "
            f"{pivot[1]:+.9f}, "
            f"{pivot[2]:+.9f}] m"
        )

        print()
        print(
            f"RMS residual = "
            f"{rms_mm:.3f} mm"
        )

        print(
            f"MAX residual = "
            f"{max_mm:.3f} mm"
        )

        print(
            f"orientation spread = "
            f"{spread_deg:.2f} deg"
        )

        print(
            f"matrix rank = "
            f"{result['rank']}/6"
        )

        output = {
            "xhand_rehab_tcp_candidate": {
                "parent": (
                    FLANGE_FRAME
                ),

                "child": (
                    "xhand_rehab_tcp"
                ),

                "translation_m": [
                    float(v)
                    for v in tcp
                ],

                # Pivot calibration determines the TCP point.
                # Functional treatment axes will later come
                # from rehab_device_frame / ankle_frame.
                "rotation_definition": {
                    "mode": (
                        "inherit_fr3_link8_axes"
                    ),
                    "rpy_rad": [
                        0.0,
                        0.0,
                        0.0,
                    ],
                    "measured": False,
                },

                "method": (
                    "manual_pivot_calibration"
                ),

                "sample_count": (
                    len(samples)
                ),

                "rms_residual_mm": (
                    float(rms_mm)
                ),

                "max_residual_mm": (
                    float(max_mm)
                ),

                "orientation_spread_deg": (
                    float(spread_deg)
                ),

                "matrix_rank": (
                    int(
                        result[
                            "rank"
                        ]
                    )
                ),

                "pivot_point_in_fr3_link0_m": [
                    float(v)
                    for v in pivot
                ],

                # Deliberately false.
                # We inspect the result before promoting it.
                "verified": False,
            }
        }

        path = Path(
            args.output
        )

        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        path.write_text(
            yaml.safe_dump(
                output,
                allow_unicode=True,
                sort_keys=False,
            ),
            encoding="utf-8",
        )

        print()
        print(
            "candidate_file =",
            path,
        )

        print()
        print(
            "[SAFE] candidate only"
        )

        print(
            "[SAFE] verified remains false"
        )

        print(
            "[SAFE] Cartesian execution "
            "gate was NOT changed"
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

        raise SystemExit(2)
