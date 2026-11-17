from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from franka_msgs.msg import FrankaRobotState

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)

from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    validate_target,
)

from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


PTP_ACTION = "/action_server/ptp_motion"
CARTESIAN_SERVICE = "/compute_cartesian_path"


class TeachPathError(RuntimeError):
    pass


def distance(a, b):
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


def ordered_route_ids(
    point_ids,
    from_point,
    to_point,
):
    if from_point not in point_ids:
        raise TeachPathError(
            f"unknown from-point: {from_point}"
        )

    if to_point not in point_ids:
        raise TeachPathError(
            f"unknown to-point: {to_point}"
        )

    i0 = point_ids.index(from_point)
    i1 = point_ids.index(to_point)

    if i0 == i1:
        raise TeachPathError(
            "from-point and to-point are identical"
        )

    step = 1 if i1 > i0 else -1

    # Do not use list slicing here.
    # Reverse slicing with stop=-1 fails when
    # the requested destination is index 0.
    indices = range(
        i0,
        i1 + step,
        step,
    )

    route = [
        point_ids[index]
        for index in indices
    ]

    if (
        not route
        or route[0] != from_point
        or route[-1] != to_point
    ):
        raise TeachPathError(
            "internal route construction failed"
        )

    return route


def subdivide_line(
    start_xyz,
    target_xyz,
    max_segment_m,
):
    length = distance(
        start_xyz,
        target_xyz,
    )

    count = max(
        1,
        math.ceil(
            length / max_segment_m
        ),
    )

    targets = []

    for index in range(
        1,
        count + 1,
    ):
        alpha = index / count

        xyz = tuple(
            float(a)
            + alpha * (
                float(b) - float(a)
            )
            for a, b in zip(
                start_xyz,
                target_xyz,
            )
        )

        targets.append(xyz)

    return length, targets


def load_yaml(path):
    with open(
        path,
        "r",
        encoding="utf-8",
    ) as handle:
        return yaml.safe_load(handle)


def load_config(path):
    data = load_yaml(path)

    if int(data.get("version", 0)) != 1:
        raise TeachPathError(
            "unsupported executor config version"
        )

    return data


def load_teach_profile(path):
    data = load_yaml(path)

    if "teach_path" not in data:
        raise TeachPathError(
            "teach_path root missing"
        )

    profile = data["teach_path"]

    raw_points = profile.get(
        "points",
        [],
    )

    if len(raw_points) < 2:
        raise TeachPathError(
            "at least two teach points required"
        )

    point_ids = []
    points = {}

    for item in raw_points:
        point_id = str(item["id"])

        if point_id in points:
            raise TeachPathError(
                f"duplicate teach point: {point_id}"
            )

        xyz = tuple(
            float(v)
            for v in item["position_m"]
        )

        if len(xyz) != 3:
            raise TeachPathError(
                f"invalid xyz for {point_id}"
            )

        point_ids.append(point_id)
        points[point_id] = xyz

    return {
        "name": profile.get(
            "name",
            "unnamed_teach_path",
        ),
        "frame": str(
            profile["frame"]
        ),
        "tool": str(
            profile["tool"]
        ),
        "point_ids": point_ids,
        "points": points,
    }


def route_geometry(
    profile,
    route_ids,
    max_segment_m,
):
    total = 0.0
    estimated_segments = 0
    edges = []

    for a, b in zip(
        route_ids[:-1],
        route_ids[1:],
    ):
        pa = profile["points"][a]
        pb = profile["points"][b]

        length = distance(pa, pb)

        count = max(
            1,
            math.ceil(
                length / max_segment_m
            ),
        )

        total += length
        estimated_segments += count

        edges.append(
            {
                "from": a,
                "to": b,
                "length_m": length,
                "estimated_segments": count,
            }
        )

    return (
        total,
        estimated_segments,
        edges,
    )


class ExecutorNode(Node):

    def __init__(self):
        super().__init__(
            "rehab_teach_path_executor"
        )

        self.q = None
        self.q_time = None

        self.robot_mode = None
        self.mode_time = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_cb,
            20,
        )

        self.create_subscription(
            FrankaRobotState,
            "/franka_robot_state_broadcaster/robot_state",
            self._robot_state_cb,
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
                CARTESIAN_SERVICE,
            )
        )

    def _joint_cb(self, msg):
        if not set(
            JOINT_NAMES
        ).issubset(
            msg.name
        ):
            return

        mapping = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        self.q = tuple(
            float(mapping[name])
            for name in JOINT_NAMES
        )

        self.q_time = time.monotonic()

    def _robot_state_cb(self, msg):
        self.robot_mode = int(
            msg.robot_mode
        )

        self.mode_time = time.monotonic()


def fresh_state(
    node,
    timeout_s,
):
    deadline = (
        time.monotonic()
        + timeout_s
    )

    while (
        time.monotonic()
        < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        now = time.monotonic()

        if (
            node.q is not None
            and node.robot_mode is not None
            and node.q_time is not None
            and node.mode_time is not None
            and now - node.q_time <= 0.10
            and now - node.mode_time <= 0.10
        ):
            return (
                validate_target(node.q),
                int(node.robot_mode),
            )

    raise TeachPathError(
        "fresh FR3 state unavailable"
    )


def wait_idle(
    node,
    *,
    idle_mode,
    move_mode,
    fresh_timeout_s,
    idle_timeout_s,
):
    deadline = (
        time.monotonic()
        + idle_timeout_s
    )

    last_mode = None

    while (
        time.monotonic()
        < deadline
    ):
        q, mode = fresh_state(
            node,
            fresh_timeout_s,
        )

        last_mode = mode

        if mode == idle_mode:
            return q

        if mode == move_mode:
            print(
                "[WAIT] robot_mode=MOVE, "
                "waiting for IDLE..."
            )
        else:
            print(
                f"[WAIT] robot_mode={mode}, "
                "waiting for IDLE..."
            )

        time.sleep(0.10)

    raise TeachPathError(
        "FR3 did not return to IDLE; "
        f"last robot_mode={last_mode}"
    )


def flange_pose(
    node,
    base,
    tip,
    timeout_s=6.0,
):
    """
    Read the latest base -> tip transform.

    A newly-created tf2 Buffer can temporarily contain an
    incomplete tree even though the robot TF publishers are
    healthy.  Do not fail on the first transient lookup error.

    This function is read-only and never sends a motion command.
    """

    deadline = (
        time.monotonic()
        + float(timeout_s)
    )

    last_error = None
    attempt = 0

    while (
        time.monotonic()
        < deadline
    ):
        attempt += 1

        try:
            transform = (
                node.tf_buffer.lookup_transform(
                    base,
                    tip,
                    Time(),
                    timeout=Duration(
                        seconds=0.5
                    ),
                )
            )

            t = (
                transform
                .transform
                .translation
            )

            q = (
                transform
                .transform
                .rotation
            )

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

            last_error = exc

            if attempt == 1:
                print(
                    "[WAIT] TF tree not ready; "
                    "waiting for "
                    f"{base} -> {tip}..."
                )

            # Allow subscriptions / tf listener
            # to populate before the next lookup.
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

            time.sleep(
                0.10
            )

    raise TeachPathError(
        "TF unavailable after "
        f"{float(timeout_s):.1f}s: "
        f"{last_error}"
    )

def plan_segment(
    node,
    *,
    topology,
    config,
    q_start,
    target_xyz,
    fixed_quat,
):
    motion = config["motion"]
    planning = config["planning"]

    current_xyz, _ = flange_pose(
        node,
        topology.base_link,
        topology.tip_link,
    )

    segment_distance = distance(
        current_xyz,
        target_xyz,
    )

    max_segment = float(
        motion[
            "max_cartesian_segment_m"
        ]
    )

    slack = float(
        motion[
            "segment_distance_slack_m"
        ]
    )

    print(
        "    actual distance = "
        f"{segment_distance * 1000:.2f} mm"
    )

    if (
        segment_distance
        > max_segment + slack
    ):
        raise TeachPathError(
            "live Cartesian segment exceeds "
            "configured safety limit"
        )

    if not (
        node.cartesian_client
        .wait_for_service(
            timeout_sec=5.0
        )
    ):
        raise TeachPathError(
            "GetCartesianPath unavailable"
        )

    request = GetCartesianPath.Request()

    request.header.frame_id = (
        topology.base_link
    )

    request.group_name = (
        topology.planning_group
    )

    request.link_name = (
        topology.tip_link
    )

    request.start_state.joint_state.name = (
        list(JOINT_NAMES)
    )

    request.start_state.joint_state.position = (
        list(q_start)
    )

    request.start_state.is_diff = False

    waypoint = Pose()

    waypoint.position.x = (
        float(target_xyz[0])
    )

    waypoint.position.y = (
        float(target_xyz[1])
    )

    waypoint.position.z = (
        float(target_xyz[2])
    )

    waypoint.orientation.x = (
        float(fixed_quat[0])
    )

    waypoint.orientation.y = (
        float(fixed_quat[1])
    )

    waypoint.orientation.z = (
        float(fixed_quat[2])
    )

    waypoint.orientation.w = (
        float(fixed_quat[3])
    )

    request.waypoints = [waypoint]

    request.max_step = float(
        planning[
            "cartesian_max_step_m"
        ]
    )

    request.jump_threshold = 0.0

    request.avoid_collisions = bool(
        planning[
            "avoid_collisions"
        ]
    )

    future = (
        node.cartesian_client.call_async(
            request
        )
    )

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=20.0,
    )

    if not future.done():
        raise TeachPathError(
            "Cartesian planning timeout"
        )

    response = future.result()

    if response is None:
        raise TeachPathError(
            "Cartesian planner returned None"
        )

    if (
        response.error_code.val
        != MoveItErrorCodes.SUCCESS
    ):
        raise TeachPathError(
            "MoveIt Cartesian planning failed: "
            f"{response.error_code.val}"
        )

    fraction = float(
        response.fraction
    )

    print(
        f"    fraction = {fraction:.6f}"
    )

    if (
        fraction
        < float(
            planning[
                "min_cartesian_fraction"
            ]
        )
    ):
        raise TeachPathError(
            "Cartesian planning fraction "
            "below configured minimum"
        )

    trajectory = (
        response.solution.joint_trajectory
    )

    if not trajectory.points:
        raise TeachPathError(
            "empty planned trajectory"
        )

    name_to_index = {
        name: index
        for index, name
        in enumerate(
            trajectory.joint_names
        )
    }

    if not set(
        JOINT_NAMES
    ).issubset(
        name_to_index
    ):
        raise TeachPathError(
            "trajectory joint mapping mismatch"
        )

    last = trajectory.points[-1]

    q_target = tuple(
        float(
            last.positions[
                name_to_index[name]
            ]
        )
        for name in JOINT_NAMES
    )

    q_target = validate_target(
        q_target
    )

    joint_delta = max_abs_delta(
        q_start,
        q_target,
    )

    print(
        "    max joint delta = "
        f"{joint_delta:.6f} rad"
    )

    if (
        joint_delta
        > float(
            motion[
                "max_segment_joint_delta_rad"
            ]
        )
    ):
        raise TeachPathError(
            "planned segment joint displacement "
            "too large"
        )

    return {
        "q_target": q_target,
        "fraction": fraction,
        "segment_distance_m": (
            segment_distance
        ),
        "trajectory_points": len(
            trajectory.points
        ),
    }


def execute_ptp_segment(
    node,
    *,
    client,
    config,
    q_target,
    label,
):
    motion = config["motion"]
    state_cfg = config["state"]

    idle_mode = int(
        state_cfg[
            "idle_robot_mode"
        ]
    )

    move_mode = int(
        state_cfg[
            "move_robot_mode"
        ]
    )

    fresh_timeout = float(
        state_cfg[
            "fresh_state_timeout_s"
        ]
    )

    idle_timeout = float(
        state_cfg[
            "idle_timeout_s"
        ]
    )

    settle_s = float(
        state_cfg.get(
            "inter_goal_settle_s",
            0.30,
        )
    )

    start_timeout = float(
        state_cfg.get(
            "goal_start_timeout_s",
            3.0,
        )
    )

    result_timeout = float(
        state_cfg.get(
            "result_timeout_s",
            30.0,
        )
    )

    min_start_motion = float(
        state_cfg.get(
            "min_start_joint_motion_rad",
            0.0005,
        )
    )

    # -------------------------------------------------
    # 1. Previous motion must have completely returned
    #    to IDLE.
    # -------------------------------------------------

    q_idle = wait_idle(
        node,
        idle_mode=idle_mode,
        move_mode=move_mode,
        fresh_timeout_s=fresh_timeout,
        idle_timeout_s=idle_timeout,
    )

    # PTP TARGET_REACHED / robot_mode IDLE can arrive
    # slightly before all server-side goal bookkeeping
    # has completely settled.
    if settle_s > 0.0:

        print(
            f"    [{label}] inter-goal settle "
            f"{settle_s:.2f} s"
        )

        time.sleep(
            settle_s
        )

    q_before, mode_before = fresh_state(
        node,
        fresh_timeout,
    )

    if mode_before != idle_mode:
        raise TeachPathError(
            f"{label}: FR3 left IDLE "
            "during inter-goal settle"
        )

    # -------------------------------------------------
    # 2. Send exactly one goal.
    # -------------------------------------------------

    goal = client.build_goal(
        q_target,
        maximum_velocity_rad_s=float(
            motion[
                "ptp_max_velocity_rad_s"
            ]
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
        raise TeachPathError(
            f"{label}: PTP acceptance timeout"
        )

    handle = send_future.result()

    if (
        handle is None
        or not handle.accepted
    ):
        raise TeachPathError(
            f"{label}: PTP rejected"
        )

    print(
        f"    [{label}] PTP accepted"
    )

    result_future = (
        handle.get_result_async()
    )

    accepted_at = time.monotonic()

    start_deadline = (
        accepted_at
        + start_timeout
    )

    result_deadline = (
        accepted_at
        + result_timeout
    )

    observed_start = False

    # -------------------------------------------------
    # 3. NEW:
    #    Acceptance alone is NOT sufficient.
    #
    #    We require either:
    #       robot_mode == MOVE
    #    or measurable real joint displacement.
    #
    #    If neither occurs, cancel and STOP.
    #    Never automatically resend the target.
    # -------------------------------------------------

    while not result_future.done():

        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        now = time.monotonic()

        if node.robot_mode == move_mode:
            observed_start = True

        if node.q is not None:

            try:
                live_motion = max_abs_delta(
                    node.q,
                    q_before,
                )

                if (
                    live_motion
                    >= min_start_motion
                ):
                    observed_start = True

            except Exception:
                pass

        if (
            not observed_start
            and now >= start_deadline
        ):

            print(
                f"    [{label}] "
                "ACCEPTED BUT NOT STARTED"
            )

            print(
                f"    [{label}] "
                "cancelling accepted goal; "
                "NO RETRY"
            )

            try:
                cancel_future = (
                    handle.cancel_goal_async()
                )

                rclpy.spin_until_future_complete(
                    node,
                    cancel_future,
                    timeout_sec=2.0,
                )

            except Exception as exc:
                print(
                    f"    [{label}] "
                    "cancel exception: "
                    f"{type(exc).__name__}: "
                    f"{exc}"
                )

            raise TeachPathError(
                f"{label}: PTP goal accepted "
                f"but motion did not start "
                f"within {start_timeout:.1f}s"
            )

        if now >= result_deadline:

            print(
                f"    [{label}] "
                "PTP RESULT TIMEOUT"
            )

            try:
                cancel_future = (
                    handle.cancel_goal_async()
                )

                rclpy.spin_until_future_complete(
                    node,
                    cancel_future,
                    timeout_sec=2.0,
                )

            except Exception:
                pass

            raise TeachPathError(
                f"{label}: PTP result timeout "
                f"after {result_timeout:.1f}s"
            )

    # -------------------------------------------------
    # 4. Result received.
    # -------------------------------------------------

    wrapped = result_future.result()

    if wrapped is None:
        raise TeachPathError(
            f"{label}: PTP result missing"
        )

    result = wrapped.result

    target_status = getattr(
        getattr(
            result,
            "target_status",
            None,
        ),
        "status",
        None,
    )

    error_message = getattr(
        result,
        "error_message",
        "",
    )

    print(
        f"    [{label}] ROS status = "
        f"{wrapped.status}"
    )

    print(
        f"    [{label}] PTP status = "
        f"{target_status}"
    )

    if error_message:
        print(
            f"    [{label}] message = "
            f"{error_message}"
        )

    if (
        wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        raise TeachPathError(
            f"{label}: ROS action failed"
        )

    if target_status != 2:
        raise TeachPathError(
            f"{label}: PTP did not report "
            "TARGET_REACHED"
        )

    print(
        f"    [{label}] waiting "
        "MOVE -> IDLE..."
    )

    q_final = wait_idle(
        node,
        idle_mode=idle_mode,
        move_mode=move_mode,
        fresh_timeout_s=fresh_timeout,
        idle_timeout_s=idle_timeout,
    )

    joint_error = max_abs_delta(
        q_final,
        q_target,
    )

    print(
        f"    [{label}] joint error = "
        f"{joint_error:.6f} rad"
    )

    if (
        joint_error
        > float(
            motion[
                "max_final_joint_error_rad"
            ]
        )
    ):
        raise TeachPathError(
            f"{label}: final joint error "
            "too large"
        )

    return {
        "ros_status": int(
            wrapped.status
        ),
        "ptp_status": (
            None
            if target_status is None
            else int(target_status)
        ),
        "joint_error_rad": (
            joint_error
        ),
        "observed_motion_start": (
            observed_start
        ),
    }

def preview(
    profile,
    config,
    from_point,
    to_point,
):
    route_ids = ordered_route_ids(
        profile["point_ids"],
        from_point,
        to_point,
    )

    max_segment = float(
        config["motion"][
            "max_cartesian_segment_m"
        ]
    )

    (
        total_length,
        estimated_segments,
        edges,
    ) = route_geometry(
        profile,
        route_ids,
        max_segment,
    )

    print("=" * 68)
    print("FR3 FINAL TEACH PATH EXECUTOR")
    print("=" * 68)

    print(
        "profile =",
        profile["name"],
    )

    print(
        "route   =",
        " -> ".join(route_ids),
    )

    print(
        "frame   =",
        profile["frame"],
    )

    print(
        "tool    =",
        profile["tool"],
    )

    print(
        "orientation = hold task-start "
        "fr3_link8 orientation"
    )

    print(
        "polyline length = "
        f"{total_length:.3f} m"
    )

    print(
        "estimated subsegments =",
        estimated_segments,
    )

    print()

    for edge in edges:
        print(
            f"  {edge['from']} -> "
            f"{edge['to']}: "
            f"{edge['length_m']*1000:.1f} mm, "
            f"~{edge['estimated_segments']} segments"
        )

    return (
        route_ids,
        total_length,
        estimated_segments,
        edges,
    )


def write_result(
    path,
    summary,
):
    if not path:
        return

    target = Path(path)

    target.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    target.write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def run_real(
    args,
    profile,
    config,
    route_ids,
    geometry,
):
    topology = (
        discover_fr3_moveit_topology()
    )

    if (
        profile["frame"]
        != topology.base_link
    ):
        raise TeachPathError(
            "teach base frame does not match "
            "MoveIt topology"
        )

    if (
        profile["tool"]
        != topology.tip_link
    ):
        raise TeachPathError(
            "teach tool frame does not match "
            "MoveIt topology"
        )

    state_cfg = config["state"]
    motion = config["motion"]

    summary = {
        "version": 1,
        "started_at": (
            datetime.now()
            .astimezone()
            .isoformat()
        ),
        "teach_file": str(
            Path(args.teach_file)
            .expanduser()
        ),
        "config_file": str(
            Path(args.config)
            .expanduser()
        ),
        "from_point": args.from_point,
        "to_point": args.to_point,
        "route": route_ids,
        "planned_polyline_length_m": (
            geometry[0]
        ),
        "estimated_subsegments": (
            geometry[1]
        ),
        "result": "RUNNING",
        "edges": [],
    }

    rclpy.init()

    node = ExecutorNode()

    try:
        q_start = wait_idle(
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

        live_xyz, task_quat = (
            flange_pose(
                node,
                topology.base_link,
                topology.tip_link,
            )
        )

        first_teach_xyz = (
            profile["points"][
                route_ids[0]
            ]
        )

        start_error = distance(
            live_xyz,
            first_teach_xyz,
        )

        print()
        print(
            "current xyz =",
            [
                round(v, 6)
                for v in live_xyz
            ],
        )

        print(
            f"{route_ids[0]} xyz =",
            [
                round(v, 6)
                for v in first_teach_xyz
            ],
        )

        print(
            "start error = "
            f"{start_error*1000:.3f} mm"
        )

        if (
            start_error
            > float(
                motion[
                    "max_start_point_error_m"
                ]
            )
        ):
            raise TeachPathError(
                "robot is not sufficiently "
                f"close to {route_ids[0]}"
            )

        if not (
            node.cartesian_client
            .wait_for_service(
                timeout_sec=5.0
            )
        ):
            raise TeachPathError(
                "GetCartesianPath unavailable"
            )

        ptp_client = FR3PTPClient(
            node,
            PTP_ACTION,
        )

        if not ptp_client.wait_ready(
            timeout_sec=3.0
        ):
            raise TeachPathError(
                "PTP action server unavailable"
            )

        print()
        print(
            "================================================"
        )

        print(
            "REAL EXECUTION READY"
        )

        print(
            "================================================"
        )

        print(
            "PTP speed = "
            f"{float(motion['ptp_max_velocity_rad_s']):.3f} "
            "rad/s"
        )

        print(
            "max Cartesian segment = "
            f"{float(motion['max_cartesian_segment_m'])*1000:.1f} "
            "mm"
        )

        print(
            "orientation = fixed task-start orientation"
        )

        print(
            "fault = stop immediately"
        )

        print(
            "automatic return = DISABLED"
        )

        print(
            "automatic recovery = DISABLED"
        )

        print(
            "XHand = NO COMMAND"
        )

        print(
            "Camera = NOT USED"
        )

        confirm = (
            f"EXECUTE-{args.from_point}-"
            f"{args.to_point}"
        )

        print()

        answer = input(
            f"输入 {confirm} 才执行："
        ).strip()

        if answer != confirm:
            summary["result"] = "CANCELLED"
            write_result(
                args.result,
                summary,
            )

            print(
                "[CANCEL] no motion sent"
            )

            return 2

        total_segment_counter = 0

        for edge_index, (
            edge_start_id,
            edge_target_id,
        ) in enumerate(
            zip(
                route_ids[:-1],
                route_ids[1:],
            ),
            start=1,
        ):
            print()
            print(
                "=" * 68
            )

            print(
                f"EDGE {edge_index}/"
                f"{len(route_ids)-1}: "
                f"{edge_start_id} -> "
                f"{edge_target_id}"
            )

            print(
                "=" * 68
            )

            q_live = wait_idle(
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

            live_edge_start, _ = (
                flange_pose(
                    node,
                    topology.base_link,
                    topology.tip_link,
                )
            )

            taught_edge_start = (
                profile["points"][
                    edge_start_id
                ]
            )

            edge_start_error = (
                distance(
                    live_edge_start,
                    taught_edge_start,
                )
            )

            print(
                "edge start error = "
                f"{edge_start_error*1000:.3f} mm"
            )

            if (
                edge_start_error
                > float(
                    motion[
                        "max_start_point_error_m"
                    ]
                )
            ):
                raise TeachPathError(
                    f"robot is not close enough "
                    f"to {edge_start_id}"
                )

            taught_target = (
                profile["points"][
                    edge_target_id
                ]
            )

            (
                live_edge_length,
                targets,
            ) = subdivide_line(
                live_edge_start,
                taught_target,
                float(
                    motion[
                        "max_cartesian_segment_m"
                    ]
                ),
            )

            print(
                "live edge length = "
                f"{live_edge_length*1000:.1f} mm"
            )

            print(
                "subsegments =",
                len(targets),
            )

            edge_summary = {
                "from": edge_start_id,
                "to": edge_target_id,
                "live_edge_length_m": (
                    live_edge_length
                ),
                "segments": [],
            }

            for local_index, target_xyz in enumerate(
                targets,
                start=1,
            ):
                total_segment_counter += 1

                print()
                print(
                    f"  --- SEGMENT "
                    f"{local_index}/{len(targets)} "
                    f"---"
                )

                q_current = wait_idle(
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

                current_xyz, _ = (
                    flange_pose(
                        node,
                        topology.base_link,
                        topology.tip_link,
                    )
                )

                print(
                    "    current =",
                    [
                        round(v, 6)
                        for v in current_xyz
                    ],
                )

                print(
                    "    target  =",
                    [
                        round(v, 6)
                        for v in target_xyz
                    ],
                )

                plan = plan_segment(
                    node,
                    topology=topology,
                    config=config,
                    q_start=q_current,
                    target_xyz=target_xyz,
                    fixed_quat=task_quat,
                )

                execution = (
                    execute_ptp_segment(
                        node,
                        client=ptp_client,
                        config=config,
                        q_target=plan[
                            "q_target"
                        ],
                        label=(
                            f"E{edge_index}"
                            f"S{local_index}"
                        ),
                    )
                )

                achieved_xyz, _ = (
                    flange_pose(
                        node,
                        topology.base_link,
                        topology.tip_link,
                    )
                )

                xyz_error = distance(
                    achieved_xyz,
                    target_xyz,
                )

                print(
                    "    Cartesian error = "
                    f"{xyz_error*1000:.3f} mm"
                )

                if (
                    xyz_error
                    > float(
                        motion[
                            "max_segment_final_error_m"
                        ]
                    )
                ):
                    raise TeachPathError(
                        "segment Cartesian final "
                        "error too large"
                    )

                edge_summary[
                    "segments"
                ].append(
                    {
                        "index": local_index,
                        "target_xyz_m": (
                            list(target_xyz)
                        ),
                        "planned_distance_m": (
                            plan[
                                "segment_distance_m"
                            ]
                        ),
                        "fraction": (
                            plan["fraction"]
                        ),
                        "joint_error_rad": (
                            execution[
                                "joint_error_rad"
                            ]
                        ),
                        "cartesian_error_m": (
                            xyz_error
                        ),
                    }
                )

            final_edge_xyz, _ = (
                flange_pose(
                    node,
                    topology.base_link,
                    topology.tip_link,
                )
            )

            waypoint_error = (
                distance(
                    final_edge_xyz,
                    taught_target,
                )
            )

            print()
            print(
                f"[EDGE PASS] "
                f"{edge_start_id} -> "
                f"{edge_target_id}"
            )

            print(
                "waypoint error = "
                f"{waypoint_error*1000:.3f} mm"
            )

            if (
                waypoint_error
                > float(
                    motion[
                        "max_waypoint_final_error_m"
                    ]
                )
            ):
                raise TeachPathError(
                    f"{edge_target_id} final "
                    "error too large"
                )

            edge_summary[
                "waypoint_final_error_m"
            ] = waypoint_error

            summary[
                "edges"
            ].append(
                edge_summary
            )

        final_xyz, _ = flange_pose(
            node,
            topology.base_link,
            topology.tip_link,
        )

        final_target = (
            profile["points"][
                route_ids[-1]
            ]
        )

        final_error = distance(
            final_xyz,
            final_target,
        )

        summary[
            "completed_segments"
        ] = total_segment_counter

        summary[
            "final_cartesian_error_m"
        ] = final_error

        summary["result"] = "PASS"

        summary["finished_at"] = (
            datetime.now()
            .astimezone()
            .isoformat()
        )

        write_result(
            args.result,
            summary,
        )

        print()
        print(
            "=" * 68
        )

        print(
            "[PASS] COMPLETE TEACH PATH EXECUTION"
        )

        print(
            "route =",
            " -> ".join(route_ids),
        )

        print(
            "completed segments =",
            total_segment_counter,
        )

        print(
            "final Cartesian error = "
            f"{final_error*1000:.3f} mm"
        )

        print(
            f"[SAFE] robot remains at "
            f"{route_ids[-1]}"
        )

        print(
            "=" * 68
        )

        return 0

    except Exception as exc:
        summary["result"] = "FAULT"

        summary["fault"] = (
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        summary["finished_at"] = (
            datetime.now()
            .astimezone()
            .isoformat()
        )

        write_result(
            args.result,
            summary,
        )

        raise

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Final segmented FR3 teach-path "
            "executor"
        )
    )

    parser.add_argument(
        "--teach-file",
        required=True,
    )

    parser.add_argument(
        "--config",
        required=True,
    )

    parser.add_argument(
        "--from-point",
        required=True,
    )

    parser.add_argument(
        "--to-point",
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
        route_ids,
        total_length,
        estimated_segments,
        edges,
    ) = preview(
        profile,
        config,
        args.from_point,
        args.to_point,
    )

    if args.preview_only:
        print()
        print(
            "[PASS] PREVIEW ONLY"
        )

        print(
            "[SAFE] no ROS motion command sent"
        )

        return 0

    try:
        return run_real(
            args,
            profile,
            config,
            route_ids,
            (
                total_length,
                estimated_segments,
                edges,
            ),
        )

    except Exception as exc:
        print()

        print(
            "[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] task stopped"
        )

        print(
            "[SAFE] no automatic return"
        )

        print(
            "[SAFE] no automatic error recovery"
        )

        return 2


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
