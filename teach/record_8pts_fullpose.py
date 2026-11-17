from __future__ import annotations

import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
import yaml

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


FR3_JOINT_NAMES = tuple(
    f"fr3_joint{i}"
    for i in range(1, 8)
)

FRAME = "fr3_link0"
TOOL = "fr3_link8"

STATE_TOPIC = "/franka/joint_states"

OUT = (
    Path.home()
    / "rehab_robot_ws"
    / "teach"
    / "teach_path_8pts_v2_fullpose.yaml"
)

# Real-state freshness.
STATE_FRESHNESS_S = 0.25

# Robot must be approximately stationary when a point is saved.
STATIONARY_WINDOW_S = 0.30
STATIONARY_MAX_SPREAD_RAD = 0.010
MIN_STATIONARY_SAMPLES = 5

# Prevent saving the same point repeatedly.
# Reject only when BOTH Cartesian and joint changes are very small.
MIN_POINT_DISTANCE_M = 0.005
MIN_POINT_JOINT_DELTA_RAD = 0.020


QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TeachRecorder(Node):
    def __init__(self):
        super().__init__(
            "fr3_8point_fullpose_recorder_v21"
        )

        self._lock = threading.Lock()

        self._q = None
        self._rx_time = None
        self._sample_count = 0

        self._history = deque(
            maxlen=2000
        )

        self.create_subscription(
            JointState,
            STATE_TOPIC,
            self._joint_cb,
            QOS,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
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

        q = tuple(
            float(table[name])
            for name in FR3_JOINT_NAMES
        )

        now = time.monotonic()

        with self._lock:
            self._q = q
            self._rx_time = now
            self._sample_count += 1

            self._history.append(
                (
                    now,
                    q,
                )
            )

    def snapshot_state(self):
        with self._lock:
            if (
                self._q is None
                or self._rx_time is None
            ):
                raise RuntimeError(
                    "no real FR3 joint-state sample"
                )

            age = (
                time.monotonic()
                - self._rx_time
            )

            if age > STATE_FRESHNESS_S:
                raise RuntimeError(
                    "stale FR3 state: "
                    f"age={age:.3f}s"
                )

            q = tuple(self._q)
            count = int(
                self._sample_count
            )

        return q, age, count

    def stationary_check(self):
        now = time.monotonic()

        cutoff = (
            now
            - STATIONARY_WINDOW_S
        )

        with self._lock:
            samples = [
                q
                for stamp, q
                in self._history
                if stamp >= cutoff
            ]

        if (
            len(samples)
            < MIN_STATIONARY_SAMPLES
        ):
            raise RuntimeError(
                "too few fresh samples: "
                f"{len(samples)}"
            )

        spreads = []

        for joint_index in range(7):
            values = [
                q[joint_index]
                for q in samples
            ]

            spreads.append(
                max(values)
                - min(values)
            )

        max_spread = max(spreads)

        if (
            max_spread
            > STATIONARY_MAX_SPREAD_RAD
        ):
            raise RuntimeError(
                "robot still moving: "
                f"max_spread="
                f"{max_spread:.6f} rad"
            )

        return (
            len(samples),
            max_spread,
        )


def normalize_quaternion(values):
    norm = math.sqrt(
        sum(
            value * value
            for value in values
        )
    )

    if norm < 1e-12:
        raise RuntimeError(
            "invalid zero quaternion"
        )

    return tuple(
        value / norm
        for value in values
    )


def lookup_tool_pose(node):
    tf = (
        node.tf_buffer
        .lookup_transform(
            FRAME,
            TOOL,
            Time(),
        )
    )

    t = tf.transform.translation
    r = tf.transform.rotation

    xyz = (
        float(t.x),
        float(t.y),
        float(t.z),
    )

    quat = normalize_quaternion(
        (
            float(r.x),
            float(r.y),
            float(r.z),
            float(r.w),
        )
    )

    return xyz, quat


def fmt(values, digits):
    return (
        "["
        + ", ".join(
            f"{value:.{digits}f}"
            for value in values
        )
        + "]"
    )


def point_difference(
    first,
    second,
):
    dxyz = math.dist(
        first["position_m"],
        second["position_m"],
    )

    dq = max(
        abs(a - b)
        for a, b in zip(
            first["joint_position_rad"],
            second["joint_position_rad"],
        )
    )

    return dxyz, dq


def check_new_point(
    candidate,
    saved_points,
):
    if not saved_points:
        return

    previous = saved_points[-1]

    dxyz, dq = point_difference(
        candidate,
        previous,
    )

    if (
        dxyz
        < MIN_POINT_DISTANCE_M
        and
        dq
        < MIN_POINT_JOINT_DELTA_RAD
    ):
        raise RuntimeError(
            "point unchanged from "
            f"{previous['id']}: "
            f"dxyz="
            f"{dxyz * 1000.0:.3f} mm, "
            f"max_dq={dq:.6f} rad"
        )

    # Also prevent accidentally repeating
    # any earlier point.
    for old_point in saved_points[:-1]:
        dxyz_old, dq_old = (
            point_difference(
                candidate,
                old_point,
            )
        )

        if (
            dxyz_old
            < MIN_POINT_DISTANCE_M
            and
            dq_old
            < MIN_POINT_JOINT_DELTA_RAD
        ):
            raise RuntimeError(
                "point duplicates "
                f"{old_point['id']}: "
                f"dxyz="
                f"{dxyz_old * 1000.0:.3f} mm, "
                f"max_dq="
                f"{dq_old:.6f} rad"
            )


def audit_points(points):
    if len(points) != 8:
        raise RuntimeError(
            f"expected 8 points, "
            f"got {len(points)}"
        )

    expected = tuple(
        f"P{i}"
        for i in range(1, 9)
    )

    actual = tuple(
        point["id"]
        for point in points
    )

    if actual != expected:
        raise RuntimeError(
            f"unexpected point order: "
            f"{actual}"
        )

    print()
    print("=" * 72)
    print(
        "V2.1 FULL-POSE "
        "TEACH AUDIT"
    )
    print("=" * 72)

    for point in points:
        quat = (
            point[
                "orientation_xyzw"
            ]
        )

        norm = math.sqrt(
            sum(
                value * value
                for value in quat
            )
        )

        if abs(norm - 1.0) > 1e-6:
            raise RuntimeError(
                f"{point['id']} "
                "quaternion invalid"
            )

        print(
            f"{point['id']}: "
            f"xyz="
            f"{fmt(point['position_m'], 6)}, "
            f"quat_norm={norm:.9f}"
        )

    print()
    print(
        "Adjacent point changes:"
    )

    for first, second in zip(
        points[:-1],
        points[1:],
    ):
        dxyz, dq = point_difference(
            first,
            second,
        )

        print(
            f"  {first['id']} "
            f"-> {second['id']}: "
            f"dxyz="
            f"{dxyz * 1000.0:8.3f} mm, "
            f"max_dq={dq:.6f} rad"
        )

        if (
            dxyz
            < MIN_POINT_DISTANCE_M
            and
            dq
            < MIN_POINT_JOINT_DELTA_RAD
        ):
            raise RuntimeError(
                f"{first['id']} -> "
                f"{second['id']} "
                "failed movement gate"
            )

    print()
    print(
        "[PASS] 8/8 full poses"
    )
    print(
        "[PASS] XYZ complete"
    )
    print(
        "[PASS] quaternion complete"
    )
    print(
        "[PASS] 7 joint angles complete"
    )
    print(
        "[PASS] movement audit"
    )


def main():
    OUT.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    rclpy.init()

    node = TeachRecorder()

    # Important:
    # keep ROS spinning continuously while
    # the operator is moving the robot and
    # while input() is waiting.
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )

    spin_thread.start()

    points = []

    try:
        print("=" * 72)
        print(
            "FR3 8-POINT FULL-POSE "
            "TEACH RECORDER V2.1"
        )
        print(
            "READ ONLY / "
            "NO MOTION COMMAND"
        )
        print("=" * 72)

        print(
            f"state topic = "
            f"{STATE_TOPIC}"
        )

        print(
            "expected publisher = "
            "joint_state_broadcaster"
        )

        print(
            f"frame = {FRAME}"
        )

        print(
            f"tool  = {TOOL}"
        )

        print(
            f"output = {OUT}"
        )

        print()
        print(
            "Record order: "
            "P1 -> P2 -> ... -> P8"
        )

        print()
        print(
            "At every point:"
        )

        print(
            "  1. manually move FR3"
        )

        print(
            "  2. stop and hold still"
        )

        print(
            "  3. press ENTER"
        )

        print(
            "  4. inspect values"
        )

        print(
            "  5. y = save"
        )

        print()
        print(
            "q = abort"
        )

        # Require real stream before teaching.
        deadline = (
            time.monotonic()
            + 5.0
        )

        while (
            time.monotonic()
            < deadline
        ):
            try:
                q, age, count = (
                    node.snapshot_state()
                )

                if count >= 5:
                    break

            except RuntimeError:
                pass

            time.sleep(0.05)

        else:
            raise RuntimeError(
                "no fresh stream from "
                f"{STATE_TOPIC}"
            )

        print()
        print(
            "[PASS] real FR3 "
            "state stream ready"
        )

        print(
            f"samples={count}"
        )

        print(
            f"state_age={age:.4f}s"
        )

        print(
            f"q={fmt(q, 6)}"
        )

        for index in range(1, 9):
            point_id = f"P{index}"

            while True:
                answer = input(
                    f"\n[{point_id}] "
                    "Move robot to point, "
                    "STOP, then press ENTER "
                    "(q=abort): "
                ).strip().lower()

                if answer == "q":
                    raise KeyboardInterrupt

                try:
                    q, age, count = (
                        node.snapshot_state()
                    )

                    (
                        stationary_samples,
                        max_spread,
                    ) = (
                        node.stationary_check()
                    )

                    xyz, quat = (
                        lookup_tool_pose(
                            node
                        )
                    )

                    candidate = {
                        "id": point_id,
                        "position_m": [
                            float(v)
                            for v in xyz
                        ],
                        "orientation_xyzw": [
                            float(v)
                            for v in quat
                        ],
                        "joint_position_rad": [
                            float(v)
                            for v in q
                        ],
                    }

                    check_new_point(
                        candidate,
                        points,
                    )

                except Exception as exc:
                    print(
                        f"[REJECT] {exc}"
                    )

                    print(
                        "[INFO] point not saved"
                    )

                    continue

                print()
                print(
                    f"  {point_id}"
                )

                print(
                    "  state_age_s       = "
                    f"{age:.4f}"
                )

                print(
                    "  stationary_samples= "
                    f"{stationary_samples}"
                )

                print(
                    "  max_spread_rad    = "
                    f"{max_spread:.6f}"
                )

                print(
                    "  position_m         = "
                    + fmt(
                        candidate[
                            "position_m"
                        ],
                        6,
                    )
                )

                print(
                    "  orientation_xyzw   = "
                    + fmt(
                        candidate[
                            "orientation_xyzw"
                        ],
                        9,
                    )
                )

                print(
                    "  joint_position_rad = "
                    + fmt(
                        candidate[
                            "joint_position_rad"
                        ],
                        9,
                    )
                )

                if points:
                    dxyz, dq = (
                        point_difference(
                            candidate,
                            points[-1],
                        )
                    )

                    print(
                        f"  from "
                        f"{points[-1]['id']}: "
                        f"dxyz="
                        f"{dxyz * 1000.0:.3f} mm, "
                        f"max_dq="
                        f"{dq:.6f} rad"
                    )

                confirm = input(
                    "  Save this point? "
                    "[y=save / "
                    "r=record again / "
                    "q=abort]: "
                ).strip().lower()

                if confirm == "q":
                    raise KeyboardInterrupt

                if confirm != "y":
                    print(
                        "[INFO] point not saved"
                    )

                    continue

                # Final freshness check
                # immediately before save.
                node.snapshot_state()

                points.append(
                    candidate
                )

                print(
                    f"[PASS] "
                    f"{point_id} saved"
                )

                break

        audit_points(points)

        if OUT.exists():
            stamp = (
                datetime.now()
                .strftime(
                    "%Y%m%d_%H%M%S"
                )
            )

            backup = (
                OUT.with_name(
                    OUT.name
                    + f".backup_{stamp}"
                )
            )

            OUT.replace(
                backup
            )

            print(
                "[INFO] previous file "
                f"backed up: {backup}"
            )

        data = {
            "teach_path": {
                "version": 2,
                "revision": "2.1",
                "name": (
                    "manual_8point_"
                    "fullpose_v2_1"
                ),
                "frame": FRAME,
                "tool": TOOL,
                "source": (
                    "manual_hand_guiding"
                ),
                "state_topic": (
                    STATE_TOPIC
                ),
                "state_publisher": (
                    "joint_state_broadcaster"
                ),
                "pose_mode": (
                    "recorded_per_point"
                ),
                "recorded_at": (
                    datetime.now()
                    .astimezone()
                    .isoformat(
                        timespec="seconds"
                    )
                ),
                "execution": {
                    "validated": False,
                    "full_path_enabled": False,
                    "collision_model_validated": False,
                },
                "points": points,
            }
        }

        OUT.write_text(
            yaml.safe_dump(
                data,
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )

        print()
        print("=" * 72)
        print(
            "[PASS] V2.1 "
            "TEACH PATH SAVED"
        )

        print(
            f"[PASS] {OUT}"
        )

        print(
            "[SAFE] validated=false"
        )

        print(
            "[SAFE] "
            "full_path_enabled=false"
        )

        print(
            "[SAFE] "
            "collision_model_validated=false"
        )

        print(
            "[SAFE] P15 real "
            "execution remains locked"
        )

        print("=" * 72)

    except KeyboardInterrupt:
        print()
        print(
            "[ABORT] teaching cancelled"
        )

        print(
            "[SAFE] incomplete path "
            "not written"
        )

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        spin_thread.join(
            timeout=1.0
        )


if __name__ == "__main__":
    main()
