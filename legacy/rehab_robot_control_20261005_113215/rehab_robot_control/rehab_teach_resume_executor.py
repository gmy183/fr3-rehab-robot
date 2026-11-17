from __future__ import annotations

import argparse
import math
from argparse import Namespace

import rclpy

from rehab_robot_control.rehab_teach_path_executor import (
    ExecutorNode,
    TeachPathError,
    distance,
    flange_pose,
    load_config,
    load_teach_profile,
    ordered_route_ids,
    route_geometry,
    run_real,
    wait_idle,
)

from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


def project_to_segment(
    point,
    start,
    end,
):
    vx = (
        float(end[0])
        - float(start[0])
    )

    vy = (
        float(end[1])
        - float(start[1])
    )

    vz = (
        float(end[2])
        - float(start[2])
    )

    wx = (
        float(point[0])
        - float(start[0])
    )

    wy = (
        float(point[1])
        - float(start[1])
    )

    wz = (
        float(point[2])
        - float(start[2])
    )

    vv = (
        vx * vx
        + vy * vy
        + vz * vz
    )

    if vv <= 0.0:
        raise TeachPathError(
            "resume edge has zero length"
        )

    t = (
        wx * vx
        + wy * vy
        + wz * vz
    ) / vv

    projection = (
        float(start[0]) + t * vx,
        float(start[1]) + t * vy,
        float(start[2]) + t * vz,
    )

    cross_track = distance(
        point,
        projection,
    )

    return (
        float(t),
        projection,
        cross_track,
    )


def capture_current(
    profile,
    config,
):
    topology = (
        discover_fr3_moveit_topology()
    )

    if (
        profile["frame"]
        != topology.base_link
        or profile["tool"]
        != topology.tip_link
    ):
        raise TeachPathError(
            "teach topology mismatch"
        )

    state_cfg = config["state"]

    rclpy.init()

    node = ExecutorNode()

    try:
        wait_idle(
            node,
            idle_mode=int(
                state_cfg[
                    "idle_robot_mode"
                ]
            ),
            move_mode=int(
                state_cfg[
                    "move_robot_mode"
                ]
            ),
            fresh_timeout_s=float(
                state_cfg[
                    "fresh_state_timeout_s"
                ]
            ),
            idle_timeout_s=float(
                state_cfg[
                    "idle_timeout_s"
                ]
            ),
        )

        xyz, quat = flange_pose(
            node,
            topology.base_link,
            topology.tip_link,
        )

        return xyz, quat

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def prepare_resume(
    profile,
    config,
    edge_from,
    edge_to,
    final_to,
):
    ids = profile["point_ids"]

    if edge_from not in ids:
        raise TeachPathError(
            f"unknown edge-from: {edge_from}"
        )

    if edge_to not in ids:
        raise TeachPathError(
            f"unknown edge-to: {edge_to}"
        )

    if final_to not in ids:
        raise TeachPathError(
            f"unknown final-to: {final_to}"
        )

    i0 = ids.index(edge_from)
    i1 = ids.index(edge_to)

    if abs(i1 - i0) != 1:
        raise TeachPathError(
            "resume edge points must be adjacent"
        )

    direction = (
        1 if i1 > i0 else -1
    )

    final_index = ids.index(
        final_to
    )

    if (
        final_index != i1
        and (
            1
            if final_index > i1
            else -1
        ) != direction
    ):
        raise TeachPathError(
            "final target is not in the "
            "resume travel direction"
        )

    current_xyz, _ = (
        capture_current(
            profile,
            config,
        )
    )

    edge_start = (
        profile["points"][
            edge_from
        ]
    )

    edge_target = (
        profile["points"][
            edge_to
        ]
    )

    (
        t,
        projection,
        cross_track,
    ) = project_to_segment(
        current_xyz,
        edge_start,
        edge_target,
    )

    tolerance = float(
        config["motion"][
            "max_start_point_error_m"
        ]
    )

    print("=" * 68)
    print("FR3 MID-EDGE RESUME CHECK")
    print("=" * 68)

    print(
        f"resume edge = "
        f"{edge_from} -> {edge_to}"
    )

    print(
        f"final target = {final_to}"
    )

    print(
        "current xyz =",
        [
            round(v, 6)
            for v in current_xyz
        ],
    )

    print(
        "edge projection =",
        [
            round(v, 6)
            for v in projection
        ],
    )

    print(
        f"edge progress = "
        f"{t*100.0:.1f}%"
    )

    print(
        f"cross-track error = "
        f"{cross_track*1000:.3f} mm"
    )

    if not (
        0.0 <= t <= 1.0
    ):
        raise TeachPathError(
            "current pose is outside "
            "the selected resume edge"
        )

    if cross_track > tolerance:
        raise TeachPathError(
            "current pose is too far "
            "from the selected resume edge"
        )

    tail = ordered_route_ids(
        ids,
        edge_to,
        final_to,
    )

    resume_ids = (
        ["RESUME"]
        + tail
    )

    resume_points = dict(
        profile["points"]
    )

    resume_points[
        "RESUME"
    ] = current_xyz

    resume_profile = dict(
        profile
    )

    resume_profile["name"] = (
        profile["name"]
        + "_mid_edge_resume"
    )

    resume_profile[
        "point_ids"
    ] = resume_ids

    resume_profile[
        "points"
    ] = resume_points

    max_segment = float(
        config["motion"][
            "max_cartesian_segment_m"
        ]
    )

    geometry = route_geometry(
        resume_profile,
        resume_ids,
        max_segment,
    )

    print(
        "resume route =",
        " -> ".join(
            resume_ids
        ),
    )

    print(
        "remaining polyline = "
        f"{geometry[0]:.3f} m"
    )

    print(
        "estimated subsegments =",
        geometry[1],
    )

    print(
        "[PASS] current pose validated "
        "on selected taught edge"
    )

    return (
        resume_profile,
        resume_ids,
        geometry,
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--teach-file",
        required=True,
    )

    parser.add_argument(
        "--config",
        required=True,
    )

    parser.add_argument(
        "--edge-from",
        required=True,
    )

    parser.add_argument(
        "--edge-to",
        required=True,
    )

    parser.add_argument(
        "--final-to",
        required=True,
    )

    parser.add_argument(
        "--result",
        default="",
    )

    parser.add_argument(
        "--preview-only",
        action="store_true",
    )

    args = parser.parse_args()

    profile = load_teach_profile(
        args.teach_file
    )

    config = load_config(
        args.config
    )

    (
        resume_profile,
        resume_ids,
        geometry,
    ) = prepare_resume(
        profile,
        config,
        args.edge_from,
        args.edge_to,
        args.final_to,
    )

    if args.preview_only:

        print()
        print(
            "[PASS] RESUME PREVIEW ONLY"
        )

        print(
            "[SAFE] no motion command sent"
        )

        return 0

    run_args = Namespace(
        teach_file=args.teach_file,
        config=args.config,
        from_point="RESUME",
        to_point=args.final_to,
        result=args.result,
    )

    return run_real(
        run_args,
        resume_profile,
        config,
        resume_ids,
        geometry,
    )


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
            "[SAFE] no automatic retry"
        )

        print(
            "[SAFE] no automatic return"
        )

        raise SystemExit(2)
