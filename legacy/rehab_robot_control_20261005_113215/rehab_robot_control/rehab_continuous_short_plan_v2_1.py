from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    JOINT_NAMES,
    JOINT_LIMITS,
    TEACH,
    PlanError,
    PlannerNode,
    pose_from_xyz_quat,
    extract_moveit_trajectory,
    dynamic_extrema,
    scale_timing,
    evaluate_segment,
    norm3,
    max_abs_delta,
    quat_angle_deg,
    project_to_segment,
)


TEST_DISTANCE_M = 0.005

CART_STEP_M = 0.0005

MOVEIT_VEL_SCALE = 0.05
MOVEIT_ACC_SCALE = 0.05

MAX_VELOCITY = 0.005
MAX_ACCELERATION = 0.025

TIME_MARGIN = 1.05

MAX_CROSS_TRACK_M = 0.0005
MAX_TARGET_ERROR_M = 0.0005
MAX_START_ERROR_M = 0.001

MAX_ORIENTATION_ERROR_DEG = 0.10

MIN_JOINT_LIMIT_MARGIN = 0.050

MAX_AUDIT_SPACING_M = 0.00075

ENDPOINT_VELOCITY_GATE = 0.0002


def make_time_scaled(points):

    original_v, original_a = (
        dynamic_extrema(points)
    )

    scale = max(
        1.0,
        (
            original_v / MAX_VELOCITY
            if MAX_VELOCITY > 0
            else 1.0
        ),
        (
            math.sqrt(
                original_a / MAX_ACCELERATION
            )
            if (
                MAX_ACCELERATION > 0
                and original_a > 0
            )
            else 1.0
        ),
    )

    scale *= TIME_MARGIN

    for _ in range(6):

        scaled = scale_timing(
            points,
            scale,
        )

        vmax, amax = (
            dynamic_extrema(scaled)
        )

        correction = max(
            1.0,
            vmax / MAX_VELOCITY,
            (
                math.sqrt(
                    amax / MAX_ACCELERATION
                )
                if amax > MAX_ACCELERATION
                else 1.0
            ),
        )

        if correction <= 1.000001:
            return (
                scale,
                scaled,
                original_v,
                original_a,
                vmax,
                amax,
            )

        scale *= correction * TIME_MARGIN

    raise PlanError(
        "cannot satisfy short-test "
        "dynamic limits"
    )


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
            "FR3 CONTINUOUS SHORT PLAN V2.1"
        )
        print(
            "CURRENT -> 5 mm TOWARD P2"
        )
        print("=" * 68)

        q_start = (
            node.read_joint_state()
        )

        xyz_start, quat_start = (
            node.read_flange_pose()
        )

        p2 = TEACH["P2"]

        vector = [
            p2[i] - xyz_start[i]
            for i in range(3)
        ]

        distance_to_p2 = math.sqrt(
            sum(v * v for v in vector)
        )

        if distance_to_p2 < 0.020:
            raise PlanError(
                "current position too close "
                "to P2 for 5 mm validation"
            )

        direction = [
            v / distance_to_p2
            for v in vector
        ]

        target_xyz = tuple(
            xyz_start[i]
            + TEST_DISTANCE_M * direction[i]
            for i in range(3)
        )

        print(
            "current xyz =",
            [
                round(v, 6)
                for v in xyz_start
            ],
        )

        print(
            "target xyz  =",
            [
                round(v, 6)
                for v in target_xyz
            ],
        )

        print(
            f"commanded translation = "
            f"{TEST_DISTANCE_M * 1000:.3f} mm"
        )

        if not (
            node.cartesian_client
            .wait_for_service(
                timeout_sec=5.0
            )
        ):
            raise PlanError(
                "/compute_cartesian_path "
                "unavailable"
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

        req = GetCartesianPath.Request()

        req.header.frame_id = "fr3_link0"
        req.group_name = "fr3_arm"
        req.link_name = "fr3_link8"

        req.max_step = CART_STEP_M

        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0

        req.avoid_collisions = True

        if hasattr(
            req,
            "max_velocity_scaling_factor",
        ):
            req.max_velocity_scaling_factor = (
                MOVEIT_VEL_SCALE
            )

        if hasattr(
            req,
            "max_acceleration_scaling_factor",
        ):
            req.max_acceleration_scaling_factor = (
                MOVEIT_ACC_SCALE
            )

        start_state = RobotState()

        start_state.joint_state.name = list(
            JOINT_NAMES
        )

        start_state.joint_state.position = list(
            q_start
        )

        req.start_state = start_state

        req.waypoints = [
            pose_from_xyz_quat(
                target_xyz,
                quat_start,
            )
        ]

        future = (
            node.cartesian_client
            .call_async(req)
        )

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=20.0,
        )

        if not future.done():
            raise PlanError(
                "GetCartesianPath timeout"
            )

        response = future.result()

        if response is None:
            raise PlanError(
                "missing Cartesian response"
            )

        fraction = float(
            response.fraction
        )

        print(
            f"fraction = {fraction:.6f}"
        )

        if fraction < 0.999:
            raise PlanError(
                "short-test Cartesian "
                "fraction < 0.999"
            )

        points = (
            extract_moveit_trajectory(
                response.solution
                .joint_trajectory
            )
        )

        print(
            f"MoveIt points = "
            f"{len(points)}"
        )

        (
            time_scale,
            scaled,
            original_v,
            original_a,
            scaled_v,
            scaled_a,
        ) = make_time_scaled(
            points
        )

        print(
            f"time scale = "
            f"{time_scale:.6f}"
        )

        print(
            f"duration = "
            f"{scaled[-1]['time']:.3f} s"
        )

        print(
            f"vmax = "
            f"{scaled_v:.6f} rad/s"
        )

        print(
            f"amax = "
            f"{scaled_a:.6f} rad/s^2"
        )

        start_velocity = max(
            abs(v)
            for v in scaled[0][
                "velocities"
            ]
        )

        end_velocity = max(
            abs(v)
            for v in scaled[-1][
                "velocities"
            ]
        )

        print(
            f"start knot velocity = "
            f"{start_velocity:.9f} rad/s"
        )

        print(
            f"end knot velocity = "
            f"{end_velocity:.9f} rad/s"
        )

        if (
            start_velocity
            > ENDPOINT_VELOCITY_GATE
        ):
            raise PlanError(
                "start trajectory velocity "
                "not approximately zero"
            )

        if (
            end_velocity
            > ENDPOINT_VELOCITY_GATE
        ):
            raise PlanError(
                "final trajectory velocity "
                "not approximately zero"
            )

        knot_xyz = []

        for point in scaled:

            xyz, _ = node.fk(
                point["positions"]
            )

            knot_xyz.append(xyz)

        dense = []

        for segment_index, (
            p0,
            p1,
        ) in enumerate(
            zip(
                scaled[:-1],
                scaled[1:],
            )
        ):

            cart_delta = norm3(
                knot_xyz[segment_index],
                knot_xyz[segment_index + 1],
            )

            joint_delta = max_abs_delta(
                p0["positions"],
                p1["positions"],
            )

            divisions = max(
                10,
                int(
                    math.ceil(
                        cart_delta
                        / 0.00025
                    )
                ),
                int(
                    math.ceil(
                        joint_delta
                        / 0.00075
                    )
                ),
            )

            for k in range(divisions):

                u = k / divisions

                q, v, a = (
                    evaluate_segment(
                        p0,
                        p1,
                        u,
                    )
                )

                xyz, quat = node.fk(q)

                dense.append({
                    "q": q,
                    "v": v,
                    "a": a,
                    "xyz": xyz,
                    "quat": quat,
                })

        final = scaled[-1]

        xyz, quat = node.fk(
            final["positions"]
        )

        dense.append({
            "q": final["positions"],
            "v": final["velocities"],
            "a": final["accelerations"],
            "xyz": xyz,
            "quat": quat,
        })

        max_cross = 0.0
        max_orientation = 0.0
        minimum_margin = float("inf")
        minimum_margin_joint = None

        progress = []

        vmax_dense = 0.0
        amax_dense = 0.0

        for sample in dense:

            raw_t, _, cross = (
                project_to_segment(
                    sample["xyz"],
                    xyz_start,
                    target_xyz,
                    clamp=False,
                )
            )

            progress.append(
                raw_t * TEST_DISTANCE_M
            )

            max_cross = max(
                max_cross,
                cross,
            )

            max_orientation = max(
                max_orientation,
                quat_angle_deg(
                    sample["quat"],
                    quat_start,
                ),
            )

            vmax_dense = max(
                vmax_dense,
                max(
                    abs(x)
                    for x in sample["v"]
                ),
            )

            amax_dense = max(
                amax_dense,
                max(
                    abs(x)
                    for x in sample["a"]
                ),
            )

            for name, value in zip(
                JOINT_NAMES,
                sample["q"],
            ):

                lower, upper = (
                    JOINT_LIMITS[name]
                )

                if not (
                    lower <= value <= upper
                ):
                    raise PlanError(
                        f"{name} outside limit"
                    )

                margin = min(
                    value - lower,
                    upper - value,
                )

                if margin < minimum_margin:
                    minimum_margin = margin
                    minimum_margin_joint = name

        spacings = [
            norm3(
                a["xyz"],
                b["xyz"],
            )
            for a, b in zip(
                dense[:-1],
                dense[1:],
            )
        ]

        max_spacing = max(spacings)

        reverse = 0.0

        for before, after in zip(
            progress[:-1],
            progress[1:],
        ):

            reverse = max(
                reverse,
                before - after,
            )

        start_error = norm3(
            dense[0]["xyz"],
            xyz_start,
        )

        target_error = norm3(
            dense[-1]["xyz"],
            target_xyz,
        )

        actual_path_length = sum(
            spacings
        )

        print()
        print(
            "========== SHORT DENSE AUDIT =========="
        )

        print(
            f"dense samples = "
            f"{len(dense)}"
        )

        print(
            f"path length = "
            f"{actual_path_length * 1000:.3f} mm"
        )

        print(
            f"max FK spacing = "
            f"{max_spacing * 1000:.3f} mm"
        )

        print(
            f"start FK error = "
            f"{start_error * 1000:.3f} mm"
        )

        print(
            f"target error = "
            f"{target_error * 1000:.3f} mm"
        )

        print(
            f"max cross-track = "
            f"{max_cross * 1000:.3f} mm"
        )

        print(
            f"max orientation error = "
            f"{max_orientation:.6f} deg"
        )

        print(
            f"max reverse progress = "
            f"{reverse * 1000:.3f} mm"
        )

        print(
            f"minimum joint margin = "
            f"{minimum_margin:.6f} rad "
            f"({minimum_margin_joint})"
        )

        print(
            f"dense vmax = "
            f"{vmax_dense:.6f} rad/s"
        )

        print(
            f"dense amax = "
            f"{amax_dense:.6f} rad/s^2"
        )

        failures = []

        if start_error > MAX_START_ERROR_M:
            failures.append(
                "start FK error > 1 mm"
            )

        if target_error > MAX_TARGET_ERROR_M:
            failures.append(
                "target FK error > 0.5 mm"
            )

        if max_cross > MAX_CROSS_TRACK_M:
            failures.append(
                "cross-track > 0.5 mm"
            )

        if (
            max_orientation
            > MAX_ORIENTATION_ERROR_DEG
        ):
            failures.append(
                "orientation error > 0.1 deg"
            )

        if reverse > 0.0002:
            failures.append(
                "reverse progress > 0.2 mm"
            )

        if (
            minimum_margin
            < MIN_JOINT_LIMIT_MARGIN
        ):
            failures.append(
                "joint margin < 0.05 rad"
            )

        if vmax_dense > MAX_VELOCITY * 1.001:
            failures.append(
                "velocity > 0.005 rad/s"
            )

        if (
            amax_dense
            > MAX_ACCELERATION * 1.001
        ):
            failures.append(
                "acceleration > 0.025 rad/s^2"
            )

        if (
            max_spacing
            > MAX_AUDIT_SPACING_M
        ):
            failures.append(
                "audit spacing > 0.75 mm"
            )

        execution_eligible = (
            len(failures) == 0
        )

        output = {
            "schema":
                "rehab_continuous_short_v2_1",

            "execution_enabled":
                False,

            "execution_eligible":
                execution_eligible,

            "frame":
                "fr3_link0",

            "tool":
                "fr3_link8",

            "test_distance_m":
                TEST_DISTANCE_M,

            "start_xyz":
                list(xyz_start),

            "target_xyz":
                list(target_xyz),

            "start_quaternion_xyzw":
                list(quat_start),

            "start_q":
                list(q_start),

            "joint_names":
                JOINT_NAMES,

            "fraction":
                fraction,

            "time_scale_factor":
                time_scale,

            "duration_s":
                scaled[-1]["time"],

            "limits": {
                "max_velocity_rad_s":
                    MAX_VELOCITY,

                "max_acceleration_rad_s2":
                    MAX_ACCELERATION,
            },

            "audit": {
                "dense_samples":
                    len(dense),

                "path_length_m":
                    actual_path_length,

                "max_fk_spacing_m":
                    max_spacing,

                "start_fk_error_m":
                    start_error,

                "target_error_m":
                    target_error,

                "max_cross_track_m":
                    max_cross,

                "max_orientation_error_deg":
                    max_orientation,

                "max_reverse_progress_m":
                    reverse,

                "minimum_joint_margin_rad":
                    minimum_margin,

                "minimum_joint_margin_joint":
                    minimum_margin_joint,

                "max_velocity_rad_s":
                    vmax_dense,

                "max_acceleration_rad_s2":
                    amax_dense,

                "start_velocity_rad_s":
                    start_velocity,

                "end_velocity_rad_s":
                    end_velocity,
            },

            "trajectory_points": [
                {
                    "time_from_start_s":
                        p["time"],

                    "positions":
                        p["positions"],

                    "velocities":
                        p["velocities"],

                    "accelerations":
                        p["accelerations"],
                }
                for p in scaled
            ],

            "blockers":
                failures,
        }

        path = Path(args.output)

        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        path.write_text(
            json.dumps(
                output,
                indent=2,
            ),
            encoding="utf-8",
        )

        print()
        print(
            f"saved = {path}"
        )

        if failures:

            print()
            print(
                "[BLOCK] SHORT TRAJECTORY "
                "NOT ELIGIBLE"
            )

            for failure in failures:
                print(
                    f"  - {failure}"
                )

            return 2

        print()
        print(
            "[PASS] SHORT TRAJECTORY "
            "EXECUTION ELIGIBLE"
        )

        print(
            "[SAFE] NO REAL MOTION SENT"
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
            "[SAFE] NO MOTION"
        )

        raise SystemExit(2)
