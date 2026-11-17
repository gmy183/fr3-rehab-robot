from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

JOINT_LIMITS = {
    "fr3_joint1": (-2.850700, 2.850700),
    "fr3_joint2": (-1.786100, 1.786100),
    "fr3_joint3": (-2.850700, 2.850700),
    "fr3_joint4": (-3.027000, -0.166900),
    "fr3_joint5": (-2.826300, 2.826300),
    "fr3_joint6": (0.489800, 4.571600),
    "fr3_joint7": (-3.000800, 3.000800),
}

TEACH = {
    "P1": (
        0.361755,
        -0.060096,
        0.442920,
    ),
    "P2": (
        0.433758,
        -0.040105,
        0.529633,
    ),
    "P3": (
        0.507519,
        0.122428,
        0.401257,
    ),
}

# V1 candidate timing.
# This is deliberately conservative and is NOT executed in this script.
MAX_JOINT_SPEED = 0.010       # rad/s
MAX_JOINT_ACCEL = 0.050       # rad/s^2
MIN_SEGMENT_DT = 0.050        # s

CARTESIAN_MAX_STEP = 0.001    # 1 mm
MIN_FRACTION = 0.995

START_EDGE_TOLERANCE = 0.008  # 8 mm, read-only resume validation
START_Q_TOLERANCE = 0.020     # rad


class PlanError(RuntimeError):
    pass


def norm3(a, b):
    return math.sqrt(
        sum(
            (float(x) - float(y)) ** 2
            for x, y in zip(a, b)
        )
    )


def max_abs_delta(a, b):
    return max(
        abs(float(x) - float(y))
        for x, y in zip(a, b)
    )


def project_to_segment(point, start, end):
    v = tuple(
        float(e) - float(s)
        for s, e in zip(start, end)
    )

    w = tuple(
        float(p) - float(s)
        for p, s in zip(point, start)
    )

    vv = sum(x * x for x in v)

    if vv <= 1e-12:
        raise PlanError(
            "zero-length taught edge"
        )

    t = sum(
        x * y
        for x, y in zip(w, v)
    ) / vv

    projection = tuple(
        float(s) + t * x
        for s, x in zip(start, v)
    )

    cross = norm3(
        point,
        projection,
    )

    return t, projection, cross


class PlannerNode(Node):

    def __init__(self):
        super().__init__(
            "rehab_continuous_trajectory_plan"
        )

        self.latest_joint_state = None

        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._joint_cb,
            20,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.cartesian_client = (
            self.create_client(
                GetCartesianPath,
                "/compute_cartesian_path",
            )
        )

    def _joint_cb(self, msg):
        self.latest_joint_state = msg

    def read_joint_state(self, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            msg = self.latest_joint_state

            if msg is None:
                continue

            mapping = {
                name: float(position)
                for name, position
                in zip(msg.name, msg.position)
            }

            if not all(
                name in mapping
                for name in JOINT_NAMES
            ):
                continue

            return [
                mapping[name]
                for name in JOINT_NAMES
            ]

        raise PlanError(
            "fresh FR3 joint state unavailable"
        )

    def read_flange_pose(self, timeout_s=6.0):
        deadline = time.monotonic() + timeout_s
        attempt = 0
        last = None

        while time.monotonic() < deadline:

            attempt += 1

            try:
                tf = self.tf_buffer.lookup_transform(
                    "fr3_link0",
                    "fr3_link8",
                    Time(),
                    timeout=Duration(
                        seconds=0.5
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

                if attempt > 1:
                    print(
                        "[PASS] TF ready after "
                        f"{attempt} attempts"
                    )

                return xyz, quat

            except Exception as exc:
                last = exc

                if attempt == 1:
                    print(
                        "[WAIT] TF tree warming..."
                    )

                time.sleep(0.10)

        raise PlanError(
            f"TF unavailable: {last}"
        )


def pose_from_xyz_quat(xyz, quat):
    pose = Pose()

    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])

    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])

    return pose


def verify_joint_limits(points):
    minimum_margin = float("inf")
    minimum_joint = None

    for point_index, q in enumerate(points):

        if len(q) != 7:
            raise PlanError(
                f"point {point_index}: "
                "expected 7 joints"
            )

        for name, value in zip(
            JOINT_NAMES,
            q,
        ):
            lower, upper = (
                JOINT_LIMITS[name]
            )

            if not (
                lower <= value <= upper
            ):
                raise PlanError(
                    f"{name} outside limit "
                    f"at point {point_index}: "
                    f"{value:.6f}"
                )

            margin = min(
                value - lower,
                upper - value,
            )

            if margin < minimum_margin:
                minimum_margin = margin
                minimum_joint = name

    return minimum_margin, minimum_joint


def remove_near_duplicates(points):
    if not points:
        return []

    output = [points[0]]

    for q in points[1:]:

        if max_abs_delta(
            q,
            output[-1],
        ) > 1e-7:
            output.append(q)

    return output


def build_timing(points):
    """
    Conservative candidate timing.

    V1 does NOT execute this trajectory.

    Segment times are selected from joint displacement and
    MAX_JOINT_SPEED.

    Waypoint velocities use a sign-aware central slope:
      - zero at endpoints
      - zero at a direction reversal
      - average adjacent slopes otherwise

    Then global time scaling is repeated until candidate
    velocity/acceleration bounds are satisfied.
    """

    if len(points) < 2:
        raise PlanError(
            "trajectory has fewer than 2 points"
        )

    dts = []

    for a, b in zip(
        points[:-1],
        points[1:],
    ):
        dq = max_abs_delta(
            a,
            b,
        )

        dt = max(
            MIN_SEGMENT_DT,
            dq / MAX_JOINT_SPEED,
        )

        dts.append(dt)

    def calculate(dts_local):
        times = [0.0]

        for dt in dts_local:
            times.append(
                times[-1] + dt
            )

        slopes = []

        for i in range(
            len(points) - 1
        ):
            dt = dts_local[i]

            slopes.append([
                (
                    float(points[i + 1][j])
                    - float(points[i][j])
                ) / dt
                for j in range(7)
            ])

        velocities = [
            [0.0] * 7
            for _ in points
        ]

        for i in range(
            1,
            len(points) - 1,
        ):
            for j in range(7):

                left = slopes[i - 1][j]
                right = slopes[i][j]

                if (
                    left == 0.0
                    or right == 0.0
                    or left * right <= 0.0
                ):
                    velocities[i][j] = 0.0

                else:
                    value = 0.5 * (
                        left + right
                    )

                    limit = min(
                        abs(left),
                        abs(right),
                    )

                    velocities[i][j] = (
                        math.copysign(
                            min(
                                abs(value),
                                limit,
                            ),
                            value,
                        )
                    )

        max_velocity = max(
            abs(v)
            for row in velocities
            for v in row
        )

        max_acceleration = 0.0

        for i in range(
            len(points) - 1
        ):
            dt = dts_local[i]

            for j in range(7):
                a = abs(
                    velocities[i + 1][j]
                    - velocities[i][j]
                ) / dt

                max_acceleration = max(
                    max_acceleration,
                    a,
                )

        return (
            times,
            velocities,
            max_velocity,
            max_acceleration,
        )

    for _ in range(20):

        (
            times,
            velocities,
            max_velocity,
            max_acceleration,
        ) = calculate(dts)

        velocity_scale = (
            max_velocity
            / MAX_JOINT_SPEED
            if MAX_JOINT_SPEED > 0.0
            else 1.0
        )

        accel_scale = math.sqrt(
            max_acceleration
            / MAX_JOINT_ACCEL
        ) if (
            MAX_JOINT_ACCEL > 0.0
            and max_acceleration
            > MAX_JOINT_ACCEL
        ) else 1.0

        scale = max(
            1.0,
            velocity_scale,
            accel_scale,
        )

        if scale <= 1.000001:
            break

        scale *= 1.05

        dts = [
            dt * scale
            for dt in dts
        ]

    (
        times,
        velocities,
        max_velocity,
        max_acceleration,
    ) = calculate(dts)

    return {
        "times": times,
        "velocities": velocities,
        "max_velocity": max_velocity,
        "max_acceleration": max_acceleration,
        "duration": times[-1],
        "segment_dt_min": min(dts),
        "segment_dt_max": max(dts),
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
        required=True,
    )

    args = parser.parse_args()

    rclpy.init()

    node = PlannerNode()

    try:
        print("=" * 68)
        print(
            "FR3 CONTINUOUS TRAJECTORY "
            "PLAN-ONLY V1"
        )
        print("=" * 68)

        q_start = (
            node.read_joint_state()
        )

        xyz_start, quat = (
            node.read_flange_pose()
        )

        print(
            "current xyz =",
            [
                round(v, 6)
                for v in xyz_start
            ],
        )

        print(
            "q_start =",
            [
                round(v, 6)
                for v in q_start
            ],
        )

        t, projection, cross = (
            project_to_segment(
                xyz_start,
                TEACH["P3"],
                TEACH["P2"],
            )
        )

        print()
        print(
            "resume taught edge = P3 -> P2"
        )

        print(
            f"edge progress = "
            f"{t * 100.0:.1f}%"
        )

        print(
            f"cross-track error = "
            f"{cross * 1000.0:.3f} mm"
        )

        if not (
            0.0 <= t <= 1.0
        ):
            raise PlanError(
                "current position lies outside "
                "P3->P2"
            )

        if cross > START_EDGE_TOLERANCE:
            raise PlanError(
                "current position is too far "
                "from P3->P2"
            )

        if not node.cartesian_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise PlanError(
                "/compute_cartesian_path "
                "unavailable"
            )

        request = (
            GetCartesianPath.Request()
        )

        request.header.frame_id = (
            "fr3_link0"
        )

        request.group_name = "fr3_arm"
        request.link_name = "fr3_link8"

        request.max_step = (
            CARTESIAN_MAX_STEP
        )

        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True

        start_state = RobotState()

        start_state.joint_state.name = (
            JOINT_NAMES
        )

        start_state.joint_state.position = (
            q_start
        )

        request.start_state = (
            start_state
        )

        request.waypoints = [
            pose_from_xyz_quat(
                TEACH["P2"],
                quat,
            ),
            pose_from_xyz_quat(
                TEACH["P1"],
                quat,
            ),
        ]

        print()
        print(
            "waypoints = "
            "CURRENT -> P2 -> P1"
        )

        print(
            "orientation = "
            "hold task-start fr3_link8 quaternion"
        )

        print(
            f"Cartesian max_step = "
            f"{CARTESIAN_MAX_STEP * 1000:.1f} mm"
        )

        print(
            "avoid_collisions = true"
        )

        future = (
            node.cartesian_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=30.0,
        )

        if not future.done():
            raise PlanError(
                "GetCartesianPath timeout"
            )

        response = future.result()

        if response is None:
            raise PlanError(
                "GetCartesianPath response "
                "missing"
            )

        fraction = float(
            response.fraction
        )

        print()
        print(
            f"fraction = {fraction:.6f}"
        )

        if fraction < MIN_FRACTION:
            raise PlanError(
                "Cartesian fraction below "
                f"{MIN_FRACTION}"
            )

        trajectory = (
            response.solution
            .joint_trajectory
        )

        returned_names = list(
            trajectory.joint_names
        )

        if returned_names != JOINT_NAMES:
            raise PlanError(
                "unexpected joint order: "
                f"{returned_names}"
            )

        raw_points = [
            [
                float(v)
                for v in point.positions
            ]
            for point in trajectory.points
        ]

        print(
            f"raw trajectory points = "
            f"{len(raw_points)}"
        )

        if len(raw_points) < 2:
            raise PlanError(
                "insufficient trajectory points"
            )

        points = remove_near_duplicates(
            raw_points
        )

        print(
            f"unique trajectory points = "
            f"{len(points)}"
        )

        start_q_error = max_abs_delta(
            points[0],
            q_start,
        )

        print(
            f"trajectory start q error = "
            f"{start_q_error:.6f} rad"
        )

        if start_q_error > START_Q_TOLERANCE:
            raise PlanError(
                "trajectory start does not "
                "match live robot"
            )

        maximum_step = max(
            max_abs_delta(a, b)
            for a, b in zip(
                points[:-1],
                points[1:],
            )
        )

        print(
            f"max adjacent joint step = "
            f"{maximum_step:.6f} rad"
        )

        margin, margin_joint = (
            verify_joint_limits(
                points
            )
        )

        print(
            f"minimum joint-limit margin = "
            f"{margin:.6f} rad "
            f"({margin_joint})"
        )

        timing = build_timing(
            points
        )

        print()
        print(
            "========== Candidate timing =========="
        )

        print(
            f"duration = "
            f"{timing['duration']:.3f} s"
        )

        print(
            f"max candidate velocity = "
            f"{timing['max_velocity']:.6f} rad/s"
        )

        print(
            f"max candidate acceleration = "
            f"{timing['max_acceleration']:.6f} rad/s^2"
        )

        print(
            f"segment dt range = "
            f"{timing['segment_dt_min']:.3f} .. "
            f"{timing['segment_dt_max']:.3f} s"
        )

        if (
            timing["max_velocity"]
            > MAX_JOINT_SPEED * 1.001
        ):
            raise PlanError(
                "candidate velocity limit "
                "exceeded"
            )

        if (
            timing["max_acceleration"]
            > MAX_JOINT_ACCEL * 1.001
        ):
            raise PlanError(
                "candidate acceleration limit "
                "exceeded"
            )

        output = {
            "schema": (
                "rehab_continuous_trajectory_v1"
            ),

            "execution_enabled": False,

            "frame": "fr3_link0",
            "tool": "fr3_link8",

            "route": [
                "CURRENT",
                "P2",
                "P1",
            ],

            "orientation_mode": (
                "hold_task_start_orientation"
            ),

            "start_xyz": list(
                xyz_start
            ),

            "start_quaternion_xyzw": list(
                quat
            ),

            "start_q": list(
                q_start
            ),

            "resume_validation": {
                "edge": "P3->P2",
                "progress": t,
                "cross_track_m": cross,
            },

            "cartesian": {
                "fraction": fraction,
                "max_step_m": (
                    CARTESIAN_MAX_STEP
                ),
                "avoid_collisions": True,
            },

            "joint_names": JOINT_NAMES,

            "trajectory_points": [
                {
                    "time_from_start_s":
                        timing["times"][i],

                    "positions":
                        points[i],

                    "velocities":
                        timing[
                            "velocities"
                        ][i],
                }
                for i in range(
                    len(points)
                )
            ],

            "audit": {
                "point_count":
                    len(points),

                "maximum_adjacent_joint_step_rad":
                    maximum_step,

                "minimum_joint_limit_margin_rad":
                    margin,

                "minimum_joint_limit_margin_joint":
                    margin_joint,

                "max_candidate_velocity_rad_s":
                    timing[
                        "max_velocity"
                    ],

                "max_candidate_acceleration_rad_s2":
                    timing[
                        "max_acceleration"
                    ],

                "duration_s":
                    timing["duration"],
            },

            "limits": {
                "max_joint_speed_rad_s":
                    MAX_JOINT_SPEED,

                "max_joint_acceleration_rad_s2":
                    MAX_JOINT_ACCEL,
            },
        }

        output_path = Path(
            args.output
        )

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        output_path.write_text(
            json.dumps(
                output,
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        print()
        print(
            f"saved = {output_path}"
        )

        print()
        print(
            "[PASS] CONTINUOUS "
            "TRAJECTORY PLAN-ONLY"
        )

        print(
            "[SAFE] execution_enabled=false"
        )

        print(
            "[SAFE] no controller activation"
        )

        print(
            "[SAFE] no FollowJointTrajectory goal"
        )

        print(
            "[SAFE] no PTP goal"
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
            "[SAFE] NO MOTION"
        )

        raise SystemExit(2)
