from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
)
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


# ------------------------------------------------------------
# Planning / execution-contract limits
# ------------------------------------------------------------

CARTESIAN_MAX_STEP = 0.001
MIN_FRACTION = 0.995

# Ask MoveIt for a slower trajectory before it performs
# time parameterization/resampling.
MOVEIT_VELOCITY_SCALING = 0.05
MOVEIT_ACCELERATION_SCALING = 0.05

# Final rehab V2 candidate limits.
# V2 DOES NOT execute them.
MAX_JOINT_SPEED = 0.010
MAX_JOINT_ACCEL = 0.050

# Existing joint-limit safety margin contract.
MIN_JOINT_LIMIT_MARGIN = 0.050

# Geometric gates.
START_EDGE_TOLERANCE = 0.008
START_FK_ERROR_GATE = 0.003
P2_ERROR_GATE = 0.003
P1_ERROR_GATE = 0.003

MAX_ROUTE_CROSS_TRACK = 0.002
MAX_ORIENTATION_ERROR_DEG = 0.50
MAX_REVERSE_PROGRESS = 0.002

# Dense FK audit resolution.
AUDIT_TARGET_CART_STEP = 0.00075
AUDIT_TARGET_JOINT_STEP = 0.0015
AUDIT_MAX_SAMPLE_SPACING = 0.002

# Dynamic extrema sampling independent of FK.
DYNAMIC_SAMPLES_PER_SEGMENT = 120

# Small safety margin on global time scaling.
TIME_SCALE_MARGIN = 1.03


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


def duration_to_s(duration):
    return (
        float(duration.sec)
        + 1e-9 * float(duration.nanosec)
    )


def project_to_segment(point, start, end, clamp=False):

    vx = [
        float(e) - float(s)
        for s, e in zip(start, end)
    ]

    wx = [
        float(p) - float(s)
        for p, s in zip(point, start)
    ]

    vv = sum(v * v for v in vx)

    if vv <= 1e-15:
        raise PlanError(
            "zero-length segment"
        )

    t = sum(
        w * v
        for w, v in zip(wx, vx)
    ) / vv

    if clamp:
        t = max(
            0.0,
            min(1.0, t),
        )

    projection = [
        float(s) + t * v
        for s, v in zip(start, vx)
    ]

    distance = norm3(
        point,
        projection,
    )

    return (
        t,
        tuple(projection),
        distance,
    )


def quat_angle_deg(a, b):

    na = math.sqrt(
        sum(float(v) ** 2 for v in a)
    )

    nb = math.sqrt(
        sum(float(v) ** 2 for v in b)
    )

    if na <= 0.0 or nb <= 0.0:
        raise PlanError(
            "invalid zero quaternion"
        )

    dot = sum(
        float(x) * float(y)
        for x, y in zip(a, b)
    ) / (na * nb)

    dot = abs(dot)

    dot = max(
        -1.0,
        min(1.0, dot),
    )

    return math.degrees(
        2.0 * math.acos(dot)
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


# ------------------------------------------------------------
# Quintic interpolation
# ------------------------------------------------------------

def quintic_coefficients(
    q0,
    v0,
    a0,
    q1,
    v1,
    a1,
    duration,
):
    """
    Fifth-order Hermite polynomial satisfying q/v/a
    boundary conditions at both ends.
    """

    T = float(duration)

    if T <= 0.0:
        raise PlanError(
            "non-positive quintic duration"
        )

    c0 = float(q0)
    c1 = float(v0)
    c2 = 0.5 * float(a0)

    c3 = (
        20.0 * (q1 - q0)
        - (8.0 * v1 + 12.0 * v0) * T
        - (3.0 * a0 - a1) * T * T
    ) / (2.0 * T ** 3)

    c4 = (
        30.0 * (q0 - q1)
        + (14.0 * v1 + 16.0 * v0) * T
        + (3.0 * a0 - 2.0 * a1) * T * T
    ) / (2.0 * T ** 4)

    c5 = (
        12.0 * (q1 - q0)
        - (6.0 * v1 + 6.0 * v0) * T
        - (a0 - a1) * T * T
    ) / (2.0 * T ** 5)

    return (
        c0,
        c1,
        c2,
        c3,
        c4,
        c5,
    )


def quintic_eval(coeff, tau):

    (
        c0,
        c1,
        c2,
        c3,
        c4,
        c5,
    ) = coeff

    t = float(tau)

    q = (
        c0
        + c1 * t
        + c2 * t ** 2
        + c3 * t ** 3
        + c4 * t ** 4
        + c5 * t ** 5
    )

    v = (
        c1
        + 2.0 * c2 * t
        + 3.0 * c3 * t ** 2
        + 4.0 * c4 * t ** 3
        + 5.0 * c5 * t ** 4
    )

    a = (
        2.0 * c2
        + 6.0 * c3 * t
        + 12.0 * c4 * t ** 2
        + 20.0 * c5 * t ** 3
    )

    return q, v, a


def build_segment_coefficients(
    p0,
    p1,
):

    T = (
        float(p1["time"])
        - float(p0["time"])
    )

    if T <= 1e-9:
        raise PlanError(
            "trajectory timestamps are "
            "not strictly increasing"
        )

    coeffs = []

    for j in range(7):

        coeffs.append(
            quintic_coefficients(
                p0["positions"][j],
                p0["velocities"][j],
                p0["accelerations"][j],
                p1["positions"][j],
                p1["velocities"][j],
                p1["accelerations"][j],
                T,
            )
        )

    return T, coeffs


def evaluate_segment(
    p0,
    p1,
    u,
):

    T, coeffs = (
        build_segment_coefficients(
            p0,
            p1,
        )
    )

    tau = max(
        0.0,
        min(1.0, float(u)),
    ) * T

    q = []
    v = []
    a = []

    for coeff in coeffs:

        qj, vj, aj = (
            quintic_eval(
                coeff,
                tau,
            )
        )

        q.append(qj)
        v.append(vj)
        a.append(aj)

    return q, v, a


def dynamic_extrema(points):

    vmax = 0.0
    amax = 0.0

    for p0, p1 in zip(
        points[:-1],
        points[1:],
    ):

        for k in range(
            DYNAMIC_SAMPLES_PER_SEGMENT + 1
        ):

            u = (
                k
                / DYNAMIC_SAMPLES_PER_SEGMENT
            )

            _, velocity, acceleration = (
                evaluate_segment(
                    p0,
                    p1,
                    u,
                )
            )

            vmax = max(
                vmax,
                max(abs(v) for v in velocity),
            )

            amax = max(
                amax,
                max(abs(a) for a in acceleration),
            )

    return vmax, amax


def scale_timing(points, scale):

    S = float(scale)

    if S < 1.0:
        raise PlanError(
            "time scale must be >= 1"
        )

    output = []

    for point in points:

        output.append({
            "time":
                float(point["time"]) * S,

            "positions":
                list(point["positions"]),

            "velocities": [
                float(v) / S
                for v in point["velocities"]
            ],

            "accelerations": [
                float(a) / (S * S)
                for a in point["accelerations"]
            ],
        })

    return output


def compute_time_scale(points):

    original_vmax, original_amax = (
        dynamic_extrema(points)
    )

    velocity_scale = (
        original_vmax / MAX_JOINT_SPEED
        if MAX_JOINT_SPEED > 0.0
        else 1.0
    )

    accel_scale = (
        math.sqrt(
            original_amax
            / MAX_JOINT_ACCEL
        )
        if (
            MAX_JOINT_ACCEL > 0.0
            and original_amax > 0.0
        )
        else 1.0
    )

    scale = max(
        1.0,
        velocity_scale,
        accel_scale,
    )

    scale *= TIME_SCALE_MARGIN

    for _ in range(5):

        scaled = scale_timing(
            points,
            scale,
        )

        vmax, amax = (
            dynamic_extrema(scaled)
        )

        vr = (
            vmax / MAX_JOINT_SPEED
            if MAX_JOINT_SPEED > 0.0
            else 1.0
        )

        ar = (
            math.sqrt(
                amax / MAX_JOINT_ACCEL
            )
            if (
                MAX_JOINT_ACCEL > 0.0
                and amax > MAX_JOINT_ACCEL
            )
            else 1.0
        )

        correction = max(
            1.0,
            vr,
            ar,
        )

        if correction <= 1.000001:
            return (
                scale,
                scaled,
                original_vmax,
                original_amax,
                vmax,
                amax,
            )

        scale *= (
            correction
            * TIME_SCALE_MARGIN
        )

    raise PlanError(
        "unable to satisfy dynamic limits "
        "after time scaling"
    )


# ------------------------------------------------------------
# ROS planning / FK
# ------------------------------------------------------------

class PlannerNode(Node):

    def __init__(self):

        super().__init__(
            "rehab_continuous_trajectory_plan_v2"
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

        self.fk_client = (
            self.create_client(
                GetPositionFK,
                "/compute_fk",
            )
        )

    def _joint_cb(self, msg):
        self.latest_joint_state = msg

    def read_joint_state(
        self,
        timeout_s=3.0,
    ):

        deadline = (
            time.monotonic()
            + timeout_s
        )

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
                in zip(
                    msg.name,
                    msg.position,
                )
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

    def read_flange_pose(
        self,
        timeout_s=6.0,
    ):

        deadline = (
            time.monotonic()
            + timeout_s
        )

        attempt = 0
        last = None

        while time.monotonic() < deadline:

            attempt += 1

            try:

                tf = (
                    self.tf_buffer
                    .lookup_transform(
                        "fr3_link0",
                        "fr3_link8",
                        Time(),
                        timeout=Duration(
                            seconds=0.5
                        ),
                    )
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
                        "[PASS] TF connected after "
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

    def fk(self, positions):

        request = (
            GetPositionFK.Request()
        )

        request.header.frame_id = (
            "fr3_link0"
        )

        request.fk_link_names = [
            "fr3_link8"
        ]

        state = RobotState()

        state.joint_state.name = list(
            JOINT_NAMES
        )

        state.joint_state.position = [
            float(v)
            for v in positions
        ]

        request.robot_state = state

        future = (
            self.fk_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done():
            raise PlanError(
                "FK request timeout"
            )

        response = future.result()

        if response is None:
            raise PlanError(
                "FK response missing"
            )

        if response.error_code.val != 1:
            raise PlanError(
                "FK error_code="
                f"{response.error_code.val}"
            )

        if not response.pose_stamped:
            raise PlanError(
                "FK returned no pose"
            )

        pose = (
            response.pose_stamped[0]
            .pose
        )

        xyz = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )

        quat = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

        return xyz, quat


# ------------------------------------------------------------
# MoveIt trajectory extraction
# ------------------------------------------------------------

def extract_moveit_trajectory(
    joint_trajectory,
):

    returned_names = list(
        joint_trajectory.joint_names
    )

    if set(returned_names) != set(
        JOINT_NAMES
    ):
        raise PlanError(
            "unexpected trajectory joints: "
            f"{returned_names}"
        )

    index = {
        name: returned_names.index(name)
        for name in JOINT_NAMES
    }

    raw_points = []

    for i, point in enumerate(
        joint_trajectory.points
    ):

        n = len(returned_names)

        if len(point.positions) != n:
            raise PlanError(
                f"point {i}: invalid positions"
            )

        if len(point.velocities) != n:
            raise PlanError(
                f"point {i}: MoveIt did not "
                "provide complete velocities"
            )

        if len(point.accelerations) != n:
            raise PlanError(
                f"point {i}: MoveIt did not "
                "provide complete accelerations"
            )

        raw_points.append({
            "time":
                duration_to_s(
                    point.time_from_start
                ),

            "positions": [
                float(
                    point.positions[
                        index[name]
                    ]
                )
                for name in JOINT_NAMES
            ],

            "velocities": [
                float(
                    point.velocities[
                        index[name]
                    ]
                )
                for name in JOINT_NAMES
            ],

            "accelerations": [
                float(
                    point.accelerations[
                        index[name]
                    ]
                )
                for name in JOINT_NAMES
            ],
        })

    if len(raw_points) < 2:
        raise PlanError(
            "MoveIt returned fewer than "
            "2 trajectory points"
        )

    t0 = raw_points[0]["time"]

    for point in raw_points:
        point["time"] -= t0

    raw_points[0]["time"] = 0.0

    for i in range(
        len(raw_points) - 1
    ):

        dt = (
            raw_points[i + 1]["time"]
            - raw_points[i]["time"]
        )

        if dt <= 1e-9:
            raise PlanError(
                f"non-increasing timestamp "
                f"at point {i}"
            )

    return raw_points


# ------------------------------------------------------------
# Dense geometry audit
# ------------------------------------------------------------

def route_metrics(
    xyz,
    start,
    p2,
    p1,
):

    d1 = norm3(start, p2)
    d2 = norm3(p2, p1)

    t1, _, e1 = project_to_segment(
        xyz,
        start,
        p2,
        clamp=True,
    )

    t2, _, e2 = project_to_segment(
        xyz,
        p2,
        p1,
        clamp=True,
    )

    if e1 <= e2:
        return (
            e1,
            t1 * d1,
            1,
        )

    return (
        e2,
        d1 + t2 * d2,
        2,
    )


def audit_dense_trajectory(
    node,
    points,
    start_xyz,
    start_quat,
):

    # First calculate FK for every trajectory knot.
    knot_xyz = []
    knot_quat = []

    print()
    print(
        "========== FK trajectory knots =========="
    )

    for i, point in enumerate(points):

        xyz, quat = node.fk(
            point["positions"]
        )

        knot_xyz.append(xyz)
        knot_quat.append(quat)

        print(
            f"KNOT "
            f"{i + 1:03d}/{len(points):03d} "
            f"xyz=["
            f"{xyz[0]:+.6f}, "
            f"{xyz[1]:+.6f}, "
            f"{xyz[2]:+.6f}]"
        )

    dense = []

    for segment_index, (
        p0,
        p1,
    ) in enumerate(
        zip(
            points[:-1],
            points[1:],
        )
    ):

        endpoint_cart_distance = norm3(
            knot_xyz[segment_index],
            knot_xyz[segment_index + 1],
        )

        endpoint_joint_distance = (
            max_abs_delta(
                p0["positions"],
                p1["positions"],
            )
        )

        subdivisions = max(
            8,
            int(
                math.ceil(
                    endpoint_cart_distance
                    / AUDIT_TARGET_CART_STEP
                )
            ),
            int(
                math.ceil(
                    endpoint_joint_distance
                    / AUDIT_TARGET_JOINT_STEP
                )
            ),
        )

        for k in range(subdivisions):

            u = (
                k / subdivisions
            )

            q, v, a = (
                evaluate_segment(
                    p0,
                    p1,
                    u,
                )
            )

            xyz, quat = node.fk(q)

            dense.append({
                "segment":
                    segment_index + 1,

                "u":
                    u,

                "q":
                    q,

                "v":
                    v,

                "a":
                    a,

                "xyz":
                    xyz,

                "quat":
                    quat,
            })

    # Append final knot exactly once.
    final = points[-1]

    xyz, quat = node.fk(
        final["positions"]
    )

    dense.append({
        "segment":
            len(points) - 1,

        "u":
            1.0,

        "q":
            list(final["positions"]),

        "v":
            list(final["velocities"]),

        "a":
            list(final["accelerations"]),

        "xyz":
            xyz,

        "quat":
            quat,
    })


    p2 = TEACH["P2"]
    p1 = TEACH["P1"]

    maximum_cross_track = 0.0
    maximum_orientation_error = 0.0
    minimum_joint_margin = float("inf")
    minimum_joint_margin_joint = None

    maximum_velocity = 0.0
    maximum_acceleration = 0.0

    nearest_p2_error = float("inf")
    nearest_p2_index = None

    progress_values = []

    for i, sample in enumerate(dense):

        xyz = sample["xyz"]
        q = sample["q"]
        v = sample["v"]
        a = sample["a"]

        cross_track, progress, _ = (
            route_metrics(
                xyz,
                start_xyz,
                p2,
                p1,
            )
        )

        progress_values.append(
            progress
        )

        maximum_cross_track = max(
            maximum_cross_track,
            cross_track,
        )

        orientation_error = (
            quat_angle_deg(
                sample["quat"],
                start_quat,
            )
        )

        maximum_orientation_error = max(
            maximum_orientation_error,
            orientation_error,
        )

        p2_error = norm3(
            xyz,
            p2,
        )

        if p2_error < nearest_p2_error:
            nearest_p2_error = p2_error
            nearest_p2_index = i

        maximum_velocity = max(
            maximum_velocity,
            max(abs(x) for x in v),
        )

        maximum_acceleration = max(
            maximum_acceleration,
            max(abs(x) for x in a),
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
                    f"{name} outside joint limit "
                    f"in dense spline audit: "
                    f"{value:.6f}"
                )

            margin = min(
                value - lower,
                upper - value,
            )

            if margin < minimum_joint_margin:
                minimum_joint_margin = margin
                minimum_joint_margin_joint = name

    cart_steps = [
        norm3(
            a["xyz"],
            b["xyz"],
        )
        for a, b in zip(
            dense[:-1],
            dense[1:],
        )
    ]

    dense_path_length = sum(
        cart_steps
    )

    maximum_sample_spacing = max(
        cart_steps
    )

    maximum_reverse_progress = 0.0

    for before, after in zip(
        progress_values[:-1],
        progress_values[1:],
    ):

        reverse = before - after

        if reverse > maximum_reverse_progress:
            maximum_reverse_progress = reverse

    start_fk_error = norm3(
        dense[0]["xyz"],
        start_xyz,
    )

    final_p1_error = norm3(
        dense[-1]["xyz"],
        p1,
    )

    expected_route_length = (
        norm3(start_xyz, p2)
        + norm3(p2, p1)
    )

    knot_max_joint_step = max(
        max_abs_delta(
            a["positions"],
            b["positions"],
        )
        for a, b in zip(
            points[:-1],
            points[1:],
        )
    )

    return {
        "dense_sample_count":
            len(dense),

        "dense_path_length_m":
            dense_path_length,

        "expected_route_length_m":
            expected_route_length,

        "max_dense_fk_spacing_m":
            maximum_sample_spacing,

        "start_fk_error_m":
            start_fk_error,

        "nearest_p2_error_m":
            nearest_p2_error,

        "nearest_p2_dense_index":
            nearest_p2_index,

        "final_p1_error_m":
            final_p1_error,

        "max_route_cross_track_m":
            maximum_cross_track,

        "max_orientation_error_deg":
            maximum_orientation_error,

        "max_reverse_progress_m":
            maximum_reverse_progress,

        "minimum_joint_limit_margin_rad":
            minimum_joint_margin,

        "minimum_joint_limit_margin_joint":
            minimum_joint_margin_joint,

        "max_dense_velocity_rad_s":
            maximum_velocity,

        "max_dense_acceleration_rad_s2":
            maximum_acceleration,

        "max_adjacent_knot_joint_step_rad":
            knot_max_joint_step,
    }


def validate_audit(
    fraction,
    audit,
):

    failures = []

    if fraction < MIN_FRACTION:
        failures.append(
            "Cartesian fraction < 0.995"
        )

    if (
        audit["start_fk_error_m"]
        > START_FK_ERROR_GATE
    ):
        failures.append(
            "start FK error > 3 mm"
        )

    if (
        audit["nearest_p2_error_m"]
        > P2_ERROR_GATE
    ):
        failures.append(
            "trajectory does not pass P2 "
            "within 3 mm"
        )

    if (
        audit["final_p1_error_m"]
        > P1_ERROR_GATE
    ):
        failures.append(
            "final P1 error > 3 mm"
        )

    if (
        audit["max_route_cross_track_m"]
        > MAX_ROUTE_CROSS_TRACK
    ):
        failures.append(
            "dense spline route deviation > 2 mm"
        )

    if (
        audit["max_orientation_error_deg"]
        > MAX_ORIENTATION_ERROR_DEG
    ):
        failures.append(
            "orientation drift > 0.50 deg"
        )

    if (
        audit["max_reverse_progress_m"]
        > MAX_REVERSE_PROGRESS
    ):
        failures.append(
            "trajectory reverses > 2 mm"
        )

    if (
        audit[
            "minimum_joint_limit_margin_rad"
        ]
        < MIN_JOINT_LIMIT_MARGIN
    ):
        failures.append(
            "joint-limit margin < 0.050 rad"
        )

    if (
        audit["max_dense_velocity_rad_s"]
        > MAX_JOINT_SPEED * 1.001
    ):
        failures.append(
            "dense spline velocity > 0.010 rad/s"
        )

    if (
        audit[
            "max_dense_acceleration_rad_s2"
        ]
        > MAX_JOINT_ACCEL * 1.001
    ):
        failures.append(
            "dense spline acceleration "
            "> 0.050 rad/s^2"
        )

    if (
        audit["max_dense_fk_spacing_m"]
        > AUDIT_MAX_SAMPLE_SPACING
    ):
        failures.append(
            "FK audit sampling spacing > 2 mm"
        )

    return failures


# ------------------------------------------------------------
# Self-test
# ------------------------------------------------------------

def self_test():

    q0 = 0.10
    v0 = 0.03
    a0 = -0.02

    q1 = 0.42
    v1 = -0.01
    a1 = 0.015

    T = 2.3

    c = quintic_coefficients(
        q0,
        v0,
        a0,
        q1,
        v1,
        a1,
        T,
    )

    qs, vs, accs = quintic_eval(
        c,
        0.0,
    )

    qe, ve, acce = quintic_eval(
        c,
        T,
    )

    values = [
        (qs, q0),
        (vs, v0),
        (accs, a0),
        (qe, q1),
        (ve, v1),
        (acce, a1),
    ]

    for actual, expected in values:

        if abs(actual - expected) > 1e-9:
            raise PlanError(
                "quintic boundary self-test failed"
            )

    # Verify uniform time scaling leaves geometry unchanged.
    S = 4.0

    c_scaled = quintic_coefficients(
        q0,
        v0 / S,
        a0 / (S * S),
        q1,
        v1 / S,
        a1 / (S * S),
        T * S,
    )

    for k in range(21):

        u = k / 20.0

        q_original, _, _ = (
            quintic_eval(
                c,
                u * T,
            )
        )

        q_scaled, _, _ = (
            quintic_eval(
                c_scaled,
                u * T * S,
            )
        )

        if (
            abs(q_original - q_scaled)
            > 1e-9
        ):
            raise PlanError(
                "time-scale geometry "
                "invariance self-test failed"
            )

    print(
        "[PASS] quintic boundary self-test"
    )

    print(
        "[PASS] uniform time-scale "
        "geometry invariance self-test"
    )

    return 0


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output",
    )

    parser.add_argument(
        "--self-test",
        action="store_true",
    )

    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if not args.output:
        raise PlanError(
            "--output is required"
        )

    rclpy.init()

    node = PlannerNode()

    try:

        print("=" * 72)
        print(
            "FR3 CONTINUOUS TRAJECTORY "
            "PLAN-ONLY V2"
        )
        print(
            "MOVEIT TIMING + QUINTIC "
            "DENSE FK AUDIT"
        )
        print("=" * 72)

        q_start = (
            node.read_joint_state()
        )

        xyz_start, quat_start = (
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

        edge_t, _, edge_cross = (
            project_to_segment(
                xyz_start,
                TEACH["P3"],
                TEACH["P2"],
                clamp=False,
            )
        )

        print()
        print(
            "resume taught edge = P3 -> P2"
        )

        print(
            f"edge progress = "
            f"{edge_t * 100.0:.1f}%"
        )

        print(
            f"cross-track error = "
            f"{edge_cross * 1000.0:.3f} mm"
        )

        if not (
            0.0 <= edge_t <= 1.0
        ):
            raise PlanError(
                "current position lies outside "
                "P3->P2"
            )

        if (
            edge_cross
            > START_EDGE_TOLERANCE
        ):
            raise PlanError(
                "current position too far "
                "from P3->P2"
            )

        if not (
            node.cartesian_client
            .wait_for_service(
                timeout_sec=5.0
            )
        ):
            raise PlanError(
                "/compute_cartesian_path unavailable"
            )

        if not (
            node.fk_client
            .wait_for_service(
                timeout_sec=5.0
            )
        ):
            raise PlanError(
                "/compute_fk unavailable"
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

        moveit_velocity_scaling_applied = None
        moveit_acceleration_scaling_applied = None

        if hasattr(
            request,
            "max_velocity_scaling_factor",
        ):
            request.max_velocity_scaling_factor = (
                MOVEIT_VELOCITY_SCALING
            )

            moveit_velocity_scaling_applied = (
                MOVEIT_VELOCITY_SCALING
            )

        if hasattr(
            request,
            "max_acceleration_scaling_factor",
        ):
            request.max_acceleration_scaling_factor = (
                MOVEIT_ACCELERATION_SCALING
            )

            moveit_acceleration_scaling_applied = (
                MOVEIT_ACCELERATION_SCALING
            )

        start_state = RobotState()

        start_state.joint_state.name = list(
            JOINT_NAMES
        )

        start_state.joint_state.position = list(
            q_start
        )

        request.start_state = (
            start_state
        )

        request.waypoints = [
            pose_from_xyz_quat(
                TEACH["P2"],
                quat_start,
            ),
            pose_from_xyz_quat(
                TEACH["P1"],
                quat_start,
            ),
        ]

        print()
        print(
            "waypoints = CURRENT -> P2 -> P1"
        )

        print(
            "orientation = hold task-start "
            "fr3_link8 quaternion"
        )

        print(
            f"Cartesian max_step = "
            f"{CARTESIAN_MAX_STEP * 1000:.1f} mm"
        )

        print(
            "avoid_collisions = true"
        )

        print(
            "MoveIt velocity scaling field = "
            f"{moveit_velocity_scaling_applied}"
        )

        print(
            "MoveIt acceleration scaling field = "
            f"{moveit_acceleration_scaling_applied}"
        )

        future = (
            node.cartesian_client
            .call_async(request)
        )

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=40.0,
        )

        if not future.done():
            raise PlanError(
                "GetCartesianPath timeout"
            )

        response = future.result()

        if response is None:
            raise PlanError(
                "GetCartesianPath response missing"
            )

        if hasattr(
            response,
            "error_code",
        ):
            if response.error_code.val != 1:
                raise PlanError(
                    "GetCartesianPath "
                    "error_code="
                    f"{response.error_code.val}"
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

        moveit_points = (
            extract_moveit_trajectory(
                response.solution
                .joint_trajectory
            )
        )

        print(
            f"MoveIt trajectory points = "
            f"{len(moveit_points)}"
        )

        print(
            f"MoveIt duration = "
            f"{moveit_points[-1]['time']:.6f} s"
        )

        start_q_error = max_abs_delta(
            moveit_points[0]["positions"],
            q_start,
        )

        print(
            f"trajectory start q error = "
            f"{start_q_error:.9f} rad"
        )

        if start_q_error > 0.020:
            raise PlanError(
                "MoveIt trajectory start does "
                "not match live robot"
            )

        (
            time_scale,
            scaled_points,
            original_vmax,
            original_amax,
            scaled_vmax,
            scaled_amax,
        ) = compute_time_scale(
            moveit_points
        )

        print()
        print(
            "========== Uniform time scaling =========="
        )

        print(
            f"original spline vmax = "
            f"{original_vmax:.6f} rad/s"
        )

        print(
            f"original spline amax = "
            f"{original_amax:.6f} rad/s^2"
        )

        print(
            f"time scale factor = "
            f"{time_scale:.6f}"
        )

        print(
            f"scaled duration = "
            f"{scaled_points[-1]['time']:.3f} s"
        )

        print(
            f"scaled spline vmax = "
            f"{scaled_vmax:.6f} rad/s"
        )

        print(
            f"scaled spline amax = "
            f"{scaled_amax:.6f} rad/s^2"
        )

        print()
        print(
            "========== Dense quintic FK audit =========="
        )

        audit = audit_dense_trajectory(
            node,
            scaled_points,
            xyz_start,
            quat_start,
        )

        print()
        print(
            "============================================"
        )

        print(
            "DENSE QUINTIC TRAJECTORY SUMMARY"
        )

        print(
            "============================================"
        )

        print(
            f"dense samples               = "
            f"{audit['dense_sample_count']}"
        )

        print(
            f"expected route length       = "
            f"{audit['expected_route_length_m'] * 1000.0:.3f} mm"
        )

        print(
            f"dense spline path length    = "
            f"{audit['dense_path_length_m'] * 1000.0:.3f} mm"
        )

        print(
            f"max FK sample spacing       = "
            f"{audit['max_dense_fk_spacing_m'] * 1000.0:.3f} mm"
        )

        print(
            f"start FK error              = "
            f"{audit['start_fk_error_m'] * 1000.0:.3f} mm"
        )

        print(
            f"nearest P2 error            = "
            f"{audit['nearest_p2_error_m'] * 1000.0:.3f} mm"
        )

        print(
            f"final P1 error              = "
            f"{audit['final_p1_error_m'] * 1000.0:.3f} mm"
        )

        print(
            f"max route cross-track       = "
            f"{audit['max_route_cross_track_m'] * 1000.0:.3f} mm"
        )

        print(
            f"max orientation error       = "
            f"{audit['max_orientation_error_deg']:.6f} deg"
        )

        print(
            f"max reverse progress        = "
            f"{audit['max_reverse_progress_m'] * 1000.0:.3f} mm"
        )

        print(
            f"minimum joint-limit margin  = "
            f"{audit['minimum_joint_limit_margin_rad']:.6f} rad "
            f"({audit['minimum_joint_limit_margin_joint']})"
        )

        print(
            f"max dense velocity          = "
            f"{audit['max_dense_velocity_rad_s']:.6f} rad/s"
        )

        print(
            f"max dense acceleration      = "
            f"{audit['max_dense_acceleration_rad_s2']:.6f} rad/s^2"
        )

        print(
            f"max adjacent trajectory knot q step = "
            f"{audit['max_adjacent_knot_joint_step_rad']:.6f} rad "
            f"(informational only)"
        )

        failures = validate_audit(
            fraction,
            audit,
        )

        execution_eligible = (
            len(failures) == 0
        )

        output = {
            "schema":
                "rehab_continuous_trajectory_v2",

            "execution_enabled":
                False,

            "execution_eligible":
                execution_eligible,

            "controller_contract": {
                "controller":
                    "fr3_arm_controller",

                "action":
                    "/fr3_arm_controller/"
                    "follow_joint_trajectory",

                "command_interfaces": [
                    "effort"
                ],

                "state_interfaces": [
                    "position",
                    "velocity",
                ],

                "interpolation_contract":
                    "position_velocity_acceleration_"
                    "quintic",
            },

            "frame":
                "fr3_link0",

            "tool":
                "fr3_link8",

            "route": [
                "CURRENT",
                "P2",
                "P1",
            ],

            "orientation_mode":
                "hold_task_start_orientation",

            "start_xyz":
                list(xyz_start),

            "start_quaternion_xyzw":
                list(quat_start),

            "start_q":
                list(q_start),

            "resume_validation": {
                "edge":
                    "P3->P2",

                "progress":
                    edge_t,

                "cross_track_m":
                    edge_cross,
            },

            "cartesian_request": {
                "fraction":
                    fraction,

                "max_step_m":
                    CARTESIAN_MAX_STEP,

                "avoid_collisions":
                    True,

                "moveit_velocity_scaling":
                    moveit_velocity_scaling_applied,

                "moveit_acceleration_scaling":
                    moveit_acceleration_scaling_applied,
            },

            "moveit_original": {
                "point_count":
                    len(moveit_points),

                "duration_s":
                    moveit_points[-1]["time"],

                "dense_quintic_vmax_rad_s":
                    original_vmax,

                "dense_quintic_amax_rad_s2":
                    original_amax,
            },

            "uniform_time_scaling": {
                "factor":
                    time_scale,

                "duration_s":
                    scaled_points[-1]["time"],

                "target_max_velocity_rad_s":
                    MAX_JOINT_SPEED,

                "target_max_acceleration_rad_s2":
                    MAX_JOINT_ACCEL,
            },

            "joint_names":
                JOINT_NAMES,

            "trajectory_points": [
                {
                    "time_from_start_s":
                        point["time"],

                    "positions":
                        point["positions"],

                    "velocities":
                        point["velocities"],

                    "accelerations":
                        point["accelerations"],
                }
                for point in scaled_points
            ],

            "audit":
                audit,

            "blockers":
                failures,
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
            "============================================"
        )

        if failures:

            print(
                "[BLOCK] V2 TRAJECTORY IS NOT "
                "ELIGIBLE FOR REAL EXECUTION"
            )

            for failure in failures:
                print(
                    f"  - {failure}"
                )

            print(
                "[SAFE] execution_enabled=false"
            )

            print(
                "[SAFE] execution_eligible=false"
            )

            print(
                "============================================"
            )

            return 2

        print(
            "[PASS] V2 DENSE QUINTIC AUDIT"
        )

        print(
            "[PASS] execution_eligible=true"
        )

        print(
            "[SAFE] execution_enabled=false"
        )

        print(
            "[SAFE] NO REAL MOTION SENT"
        )

        print(
            "============================================"
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
