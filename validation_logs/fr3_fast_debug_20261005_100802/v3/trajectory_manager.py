from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
import json
import math
import os
import signal
import subprocess
import time

from .config import V3Config
from .teach_path import TeachPath, distance, project_to_segment


JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
JOINT_LIMITS = {
    "fr3_joint1": (-2.850700, 2.850700),
    "fr3_joint2": (-1.786100, 1.786100),
    "fr3_joint3": (-2.850700, 2.850700),
    "fr3_joint4": (-3.027000, -0.166900),
    "fr3_joint5": (-2.826300, 2.826300),
    "fr3_joint6": (0.489800, 4.571600),
    "fr3_joint7": (-3.000800, 3.000800),
}


class TrajectoryError(RuntimeError):
    pass


def max_abs_delta(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def quat_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na = math.sqrt(sum(float(x) ** 2 for x in a))
    nb = math.sqrt(sum(float(x) ** 2 for x in b))
    if na <= 1e-15 or nb <= 1e-15:
        raise TrajectoryError("invalid quaternion")
    dot = sum(float(x) * float(y) for x, y in zip(a, b)) / (na * nb)
    dot = abs(max(-1.0, min(1.0, dot)))
    return math.degrees(2.0 * math.acos(dot))


def duration_to_s(duration: Any) -> float:
    return float(duration.sec) + 1e-9 * float(duration.nanosec)


def quintic_coefficients(q0: float, v0: float, a0: float, q1: float, v1: float, a1: float, T: float):
    if T <= 0.0:
        raise TrajectoryError("non-positive quintic duration")
    c0 = float(q0)
    c1 = float(v0)
    c2 = 0.5 * float(a0)
    c3 = (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T * T) / (2.0 * T ** 3)
    c4 = (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T * T) / (2.0 * T ** 4)
    c5 = (12.0 * (q1 - q0) - (6.0 * v1 + 6.0 * v0) * T - (a0 - a1) * T * T) / (2.0 * T ** 5)
    return c0, c1, c2, c3, c4, c5


def quintic_eval(coeff: Sequence[float], tau: float):
    c0, c1, c2, c3, c4, c5 = coeff
    t = float(tau)
    q = c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
    v = c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
    a = 2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3
    return q, v, a


def evaluate_segment(p0: Dict[str, Any], p1: Dict[str, Any], u: float):
    T = float(p1["time"]) - float(p0["time"])
    if T <= 1e-9:
        raise TrajectoryError("non-increasing trajectory timestamps")
    tau = max(0.0, min(1.0, float(u))) * T
    q, v, a = [], [], []
    for j in range(7):
        c = quintic_coefficients(
            p0["positions"][j], p0["velocities"][j], p0["accelerations"][j],
            p1["positions"][j], p1["velocities"][j], p1["accelerations"][j], T,
        )
        qj, vj, aj = quintic_eval(c, tau)
        q.append(qj); v.append(vj); a.append(aj)
    return q, v, a


def dynamic_extrema(points: List[Dict[str, Any]], samples_per_segment: int = 120):
    vmax = 0.0
    amax = 0.0
    for p0, p1 in zip(points[:-1], points[1:]):
        for k in range(samples_per_segment + 1):
            _, v, a = evaluate_segment(p0, p1, k / samples_per_segment)
            vmax = max(vmax, max(abs(x) for x in v))
            amax = max(amax, max(abs(x) for x in a))
    return vmax, amax


def scale_timing(points: List[Dict[str, Any]], scale: float):
    S = float(scale)
    return [
        {
            "time": float(p["time"]) * S,
            "positions": list(p["positions"]),
            "velocities": [float(v) / S for v in p["velocities"]],
            "accelerations": [float(a) / (S*S) for a in p["accelerations"]],
        }
        for p in points
    ]


def extract_moveit_trajectory(jt: Any) -> List[Dict[str, Any]]:
    returned = list(jt.joint_names)
    if set(returned) != set(JOINT_NAMES):
        raise TrajectoryError(f"unexpected MoveIt joint set: {returned}")
    idx = {name: returned.index(name) for name in JOINT_NAMES}
    out: List[Dict[str, Any]] = []
    for i, p in enumerate(jt.points):
        n = len(returned)
        if len(p.positions) != n or len(p.velocities) != n or len(p.accelerations) != n:
            raise TrajectoryError(f"MoveIt point {i} missing position/velocity/acceleration")
        out.append({
            "time": duration_to_s(p.time_from_start),
            "positions": [float(p.positions[idx[nm]]) for nm in JOINT_NAMES],
            "velocities": [float(p.velocities[idx[nm]]) for nm in JOINT_NAMES],
            "accelerations": [float(p.accelerations[idx[nm]]) for nm in JOINT_NAMES],
        })
    if len(out) < 2:
        raise TrajectoryError("MoveIt returned fewer than 2 trajectory points")
    t0 = out[0]["time"]
    for p in out:
        p["time"] -= t0
    out[0]["time"] = 0.0
    for a, b in zip(out[:-1], out[1:]):
        if b["time"] - a["time"] <= 1e-9:
            raise TrajectoryError("MoveIt trajectory timestamps are not strictly increasing")
    return out


def uniform_time_scale(points: List[Dict[str, Any]], vmax_limit: float, amax_limit: float, margin: float):
    orig_v, orig_a = dynamic_extrema(points)
    scale = max(
        1.0,
        orig_v / vmax_limit if vmax_limit > 0 else 1.0,
        math.sqrt(orig_a / amax_limit) if amax_limit > 0 and orig_a > 0 else 1.0,
    ) * margin
    for _ in range(6):
        scaled = scale_timing(points, scale)
        vmax, amax = dynamic_extrema(scaled)
        correction = max(
            1.0,
            vmax / vmax_limit if vmax_limit > 0 else 1.0,
            math.sqrt(amax / amax_limit) if amax_limit > 0 and amax > amax_limit else 1.0,
        )
        if correction <= 1.000001:
            return scale, scaled, orig_v, orig_a, vmax, amax
        scale *= correction * margin
    raise TrajectoryError("unable to satisfy trajectory dynamic limits")


def nearest_polyline_distance_and_progress(xyz: Sequence[float], polyline: Sequence[Sequence[float]]):
    cumulative = 0.0
    best = None
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        _, tc, _, cross = project_to_segment(xyz, a, b)
        seg_len = distance(a, b)
        progress = cumulative + tc * seg_len
        item = (cross, progress, i)
        if best is None or item[0] < best[0]:
            best = item
        cumulative += seg_len
    if best is None:
        return distance(xyz, polyline[0]), 0.0, 0
    return best


@dataclass
class LiveState:
    q: List[float]
    xyz: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]


class PlanningServer:
    def __init__(self, config: V3Config, log_path: Path):
        self.config = config
        self.log_path = Path(log_path)
        self.proc: Optional[subprocess.Popen] = None
        self.log_file = None

    def __enter__(self):
        server = self.config.moveit_server
        if not Path(server).is_file():
            raise TrajectoryError(f"planning-only server missing: {server}")
        rc = subprocess.run(
            ["pgrep", "-f", "^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False,
        )
        if rc.returncode == 0 and rc.stdout.strip():
            raise TrajectoryError("existing move_group detected; refusing to stack planning servers")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open("w", encoding="utf-8")
        self.proc = subprocess.Popen(
            ["python3", server, "--robot-ip", self.config.robot_ip],
            stdout=self.log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        deadline = time.monotonic() + 25.0
        cart = str(self.config.hardware["moveit"]["cartesian_service"])
        fk = str(self.config.hardware["moveit"]["fk_service"])
        while time.monotonic() < deadline:
            p = subprocess.run(
                ["ros2", "service", "list", "-t"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False,
            )
            text = p.stdout
            if f"{cart} [moveit_msgs/srv/GetCartesianPath]" in text and f"{fk} [moveit_msgs/srv/GetPositionFK]" in text:
                return self
            if self.proc.poll() is not None:
                break
            time.sleep(0.4)
        tail = ""
        try:
            self.log_file.flush()
            tail = "\n".join(self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-80:])
        except Exception:
            pass
        self.__exit__(None, None, None)
        raise TrajectoryError(f"MoveIt planning services unavailable\n{tail}")

    def __exit__(self, exc_type, exc, tb):
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(self.proc.pid, signal.SIGTERM)
                    self.proc.wait(timeout=2.0)
                except Exception:
                    pass
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
        # Targeted cleanup only for planning-only move_group children.
        subprocess.run(
            ["pkill", "-TERM", "-f", "^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
        time.sleep(0.5)
        return False


class RosMoveItPlanner:
    def __init__(self, config: V3Config, teach_path: TeachPath):
        self.config = config
        self.teach_path = teach_path
        # Lazy ROS imports keep pure-core tests runnable without ROS.
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener
        from moveit_msgs.srv import GetCartesianPath, GetPositionFK

        self.rclpy = rclpy
        self.GetCartesianPath = GetCartesianPath
        self.GetPositionFK = GetPositionFK
        if not rclpy.ok():
            rclpy.init()
            self._owns_context = True
        else:
            self._owns_context = False
        self.node = Node("rehab_v3_moveit_planner")
        self.latest_joint_state = None
        self.node.create_subscription(JointState, str(config.hardware["fr3"]["joint_state_topic"]), self._joint_cb, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.cart_client = self.node.create_client(GetCartesianPath, str(config.hardware["moveit"]["cartesian_service"]))
        self.fk_client = self.node.create_client(GetPositionFK, str(config.hardware["moveit"]["fk_service"]))

    def close(self):
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and self.rclpy.ok():
                self.rclpy.shutdown()

    def _joint_cb(self, msg):
        self.latest_joint_state = msg

    def _spin_until(self, predicate, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def capture_live_state(self) -> LiveState:
        # V3.1.3: every capture must consume a NEW JointState sample.
        #
        # Previously latest_joint_state was cached. If MoveIt startup or any
        # other blocking operation took several seconds, q could remain from
        # before that operation while TF was read at the current time. That
        # produced an internally inconsistent LiveState (stale q + fresh TF).
        # For the FR3 effort controller this was large enough to look like a
        # 7 mm FK start error even though the robot itself was healthy.
        self.latest_joint_state = None
        ok = self._spin_until(lambda: self.latest_joint_state is not None, 3.0)
        if not ok:
            raise TrajectoryError("fresh FR3 joint state unavailable")

        msg = self.latest_joint_state
        mapping = {n: float(q) for n, q in zip(msg.name, msg.position)}
        if not all(n in mapping for n in JOINT_NAMES):
            raise TrajectoryError("joint state does not contain all FR3 joints")
        q = [mapping[n] for n in JOINT_NAMES]

        # Keep spinning while TF catches up to the just-consumed joint sample.
        # We use the newest TF, but q is now guaranteed to be newly sampled in
        # this same capture call instead of being an arbitrarily old cache.
        from rclpy.time import Time
        from rclpy.duration import Duration
        deadline = time.monotonic() + 6.0
        last = None
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                tf = self.tf_buffer.lookup_transform(
                    str(self.config.task["base_frame"]),
                    str(self.config.task["tool_frame"]),
                    Time(),
                    timeout=Duration(seconds=0.25),
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                return LiveState(
                    q=q,
                    xyz=(float(t.x), float(t.y), float(t.z)),
                    quat=(float(r.x), float(r.y), float(r.z), float(r.w)),
                )
            except Exception as exc:
                last = exc
                time.sleep(0.02)
        raise TrajectoryError(f"TF unavailable: {last}")

    def fk(self, q: Sequence[float]):
        # V3.1.5: wait for this planner node's FK client discovery.
        #
        # PlanningServer.__enter__ previously verified /compute_fk through the
        # ROS graph CLI, but this RosMoveItPlanner node and its already-created
        # client can discover the newly-started move_group a little later.
        # Calling call_async() immediately in that window can leave the request
        # unanswered until the local 5 s future timeout even though the service
        # is visible globally.  This is a discovery/handshake race, not a robot
        # or kinematics failure.
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            raise TrajectoryError("FK service unavailable to planner client after 10 s")

        from moveit_msgs.msg import RobotState

        req = self.GetPositionFK.Request()
        req.header.frame_id = str(self.config.task["base_frame"])
        req.fk_link_names = [str(self.config.task["tool_frame"])]
        st = RobotState()
        st.joint_state.name = list(JOINT_NAMES)
        st.joint_state.position = [float(x) for x in q]
        req.robot_state = st

        # Two bounded, read-only attempts are allowed.  This never sends a
        # robot command and never retries a trajectory.  The second attempt is
        # only for a transient first RPC during MoveIt startup/warm-up.
        for attempt in (1, 2):
            fut = self.fk_client.call_async(req)
            self.rclpy.spin_until_future_complete(
                self.node,
                fut,
                timeout_sec=8.0,
            )

            if fut.done() and fut.result() is not None:
                res = fut.result()
                if res.error_code.val != 1 or not res.pose_stamped:
                    raise TrajectoryError(f"FK error_code={res.error_code.val}")
                p = res.pose_stamped[0].pose
                return (
                    (float(p.position.x), float(p.position.y), float(p.position.z)),
                    (
                        float(p.orientation.x),
                        float(p.orientation.y),
                        float(p.orientation.z),
                        float(p.orientation.w),
                    ),
                )

            try:
                fut.cancel()
            except Exception:
                pass

            if attempt == 1:
                print("[WAIT] first FK RPC timed out; one read-only retry")
                if not self.fk_client.wait_for_service(timeout_sec=3.0):
                    raise TrajectoryError("FK service disappeared after first RPC timeout")

        raise TrajectoryError("FK service response timeout after 2 read-only attempts")

    def _pose(self, xyz: Sequence[float], quat: Sequence[float]):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        return p

    def plan_stage(
        self,
        stage_name: str,
        start_q: Sequence[float],
        start_xyz: Sequence[float],
        quat: Sequence[float],
        waypoint_names: Sequence[str],
    ) -> Dict[str, Any]:
        if not waypoint_names:
            raise TrajectoryError(f"{stage_name}: empty waypoint list")
        if not self.cart_client.wait_for_service(timeout_sec=5.0):
            raise TrajectoryError("GetCartesianPath service unavailable")
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            raise TrajectoryError("FK service unavailable")

        from moveit_msgs.msg import RobotState
        req = self.GetCartesianPath.Request()
        req.header.frame_id = str(self.config.task["base_frame"])
        req.group_name = "fr3_arm"
        req.link_name = str(self.config.task["tool_frame"])
        ps = self.config.safety["planning"]
        req.max_step = float(ps["cartesian_max_step_m"])
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = bool(ps["avoid_collisions"])
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = float(ps["moveit_velocity_scaling"])
        if hasattr(req, "max_acceleration_scaling_factor"):
            req.max_acceleration_scaling_factor = float(ps["moveit_acceleration_scaling"])
        st = RobotState()
        st.joint_state.name = list(JOINT_NAMES)
        st.joint_state.position = [float(x) for x in start_q]
        req.start_state = st
        req.waypoints = [self._pose(self.teach_path.points[n], quat) for n in waypoint_names]

        fut = self.cart_client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60.0)
        if not fut.done() or fut.result() is None:
            raise TrajectoryError(f"{stage_name}: GetCartesianPath timeout")
        res = fut.result()
        if hasattr(res, "error_code") and res.error_code.val != 1:
            raise TrajectoryError(f"{stage_name}: MoveIt error_code={res.error_code.val}")
        fraction = float(res.fraction)
        if fraction < float(ps["minimum_fraction"]):
            raise TrajectoryError(f"{stage_name}: fraction={fraction:.6f} below threshold")

        raw = extract_moveit_trajectory(res.solution.joint_trajectory)
        if max_abs_delta(raw[0]["positions"], start_q) > float(self.config.safety["trajectory"]["maximum_start_joint_error_rad"]):
            raise TrajectoryError(f"{stage_name}: trajectory start does not match provided start state")

        ts = self.config.safety["trajectory"]
        scale, scaled, orig_v, orig_a, vmax, amax = uniform_time_scale(
            raw,
            float(ts["max_joint_velocity_rad_s"]),
            float(ts["max_joint_acceleration_rad_s2"]),
            float(ts["time_scale_margin"]),
        )
        audit = self._audit_dense(stage_name, scaled, start_xyz, quat, waypoint_names)
        blockers = []
        if audit["max_cross_track_m"] > float(ts["maximum_cartesian_cross_track_m"]):
            blockers.append("cross-track limit exceeded")
        if audit["max_orientation_error_deg"] > float(ts["maximum_orientation_error_deg"]):
            blockers.append("orientation error limit exceeded")
        if audit["minimum_joint_limit_margin_rad"] < float(ts["minimum_joint_limit_margin_rad"]):
            blockers.append("joint-limit margin too small")
        if audit["max_fk_spacing_m"] > float(ts["dense_fk_max_spacing_m"]):
            blockers.append("dense FK audit spacing too large")
        if audit["max_velocity_rad_s"] > float(ts["max_joint_velocity_rad_s"]) * 1.001:
            blockers.append("velocity limit exceeded")
        if audit["max_acceleration_rad_s2"] > float(ts["max_joint_acceleration_rad_s2"]) * 1.001:
            blockers.append("acceleration limit exceeded")
        max_wp = max(audit["waypoint_errors_m"].values()) if audit["waypoint_errors_m"] else 0.0
        if max_wp > float(ts["maximum_waypoint_error_m"]):
            blockers.append("waypoint error limit exceeded")
        if audit["final_error_m"] > float(ts["maximum_final_error_m"]):
            blockers.append("final Cartesian error limit exceeded")
        if blockers:
            raise TrajectoryError(f"{stage_name}: dense audit failed: {blockers}")

        return {
            "schema": "rehab_v3_stage_trajectory",
            "stage": stage_name,
            "execution_enabled": False,
            "execution_eligible": True,
            "frame": str(self.config.task["base_frame"]),
            "tool": str(self.config.task["tool_frame"]),
            "orientation_mode": str(self.config.task["orientation_mode"]),
            "start_xyz": list(map(float, start_xyz)),
            "start_q": list(map(float, start_q)),
            "start_quaternion_xyzw": list(map(float, quat)),
            "waypoint_names": list(waypoint_names),
            "waypoints_xyz": [list(self.teach_path.points[n]) for n in waypoint_names],
            "fraction": fraction,
            "moveit_point_count": len(raw),
            "moveit_duration_s": raw[-1]["time"],
            "uniform_time_scale": scale,
            "duration_s": scaled[-1]["time"],
            "original_spline_vmax_rad_s": orig_v,
            "original_spline_amax_rad_s2": orig_a,
            "scaled_spline_vmax_rad_s": vmax,
            "scaled_spline_amax_rad_s2": amax,
            "joint_names": list(JOINT_NAMES),
            "trajectory_points": [
                {
                    "time_from_start_s": p["time"],
                    "positions": p["positions"],
                    "velocities": p["velocities"],
                    "accelerations": p["accelerations"],
                }
                for p in scaled
            ],
            "audit": audit,
            "end_q": list(scaled[-1]["positions"]),
            "end_xyz": list(self.teach_path.points[waypoint_names[-1]]),
        }

    def _audit_dense(self, stage_name: str, points: List[Dict[str, Any]], start_xyz: Sequence[float], quat: Sequence[float], waypoint_names: Sequence[str]):
        ts = self.config.safety["trajectory"]
        target_spacing = float(ts["dense_fk_target_spacing_m"])
        polyline = [tuple(map(float, start_xyz))] + [self.teach_path.points[n] for n in waypoint_names]
        knot_xyz = [self.fk(p["positions"])[0] for p in points]
        dense = []
        for i, (p0, p1) in enumerate(zip(points[:-1], points[1:])):
            cart = distance(knot_xyz[i], knot_xyz[i + 1])
            qstep = max_abs_delta(p0["positions"], p1["positions"])
            divisions = max(8, int(math.ceil(cart / target_spacing)), int(math.ceil(qstep / 0.0015)))
            for k in range(divisions):
                q, v, a = evaluate_segment(p0, p1, k / divisions)
                xyz, qq = self.fk(q)
                dense.append((q, v, a, xyz, qq))
        q = points[-1]["positions"]
        xyz, qq = self.fk(q)
        dense.append((q, points[-1]["velocities"], points[-1]["accelerations"], xyz, qq))

        max_cross = 0.0
        max_ori = 0.0
        min_margin = float("inf")
        min_joint = None
        max_v = 0.0
        max_a = 0.0
        progresses = []
        for q, v, a, xyz, qq in dense:
            cross, progress, _ = nearest_polyline_distance_and_progress(xyz, polyline)
            progresses.append(progress)
            max_cross = max(max_cross, cross)
            max_ori = max(max_ori, quat_angle_deg(qq, quat))
            max_v = max(max_v, max(abs(x) for x in v))
            max_a = max(max_a, max(abs(x) for x in a))
            for name, value in zip(JOINT_NAMES, q):
                lo, hi = JOINT_LIMITS[name]
                if not (lo <= value <= hi):
                    raise TrajectoryError(f"{stage_name}: {name} outside limit during dense audit")
                margin = min(value - lo, hi - value)
                if margin < min_margin:
                    min_margin = margin; min_joint = name

        spacings = [distance(a[3], b[3]) for a, b in zip(dense[:-1], dense[1:])]
        max_spacing = max(spacings) if spacings else 0.0
        reverse = max((a - b for a, b in zip(progresses[:-1], progresses[1:])), default=0.0)
        waypoint_errors = {
            name: min(distance(sample[3], self.teach_path.points[name]) for sample in dense)
            for name in waypoint_names
        }
        final_error = distance(dense[-1][3], self.teach_path.points[waypoint_names[-1]])
        return {
            "dense_sample_count": len(dense),
            "max_fk_spacing_m": max_spacing,
            "max_cross_track_m": max_cross,
            "max_orientation_error_deg": max_ori,
            "max_reverse_progress_m": max(0.0, reverse),
            "minimum_joint_limit_margin_rad": min_margin,
            "minimum_joint_limit_margin_joint": min_joint,
            "max_velocity_rad_s": max_v,
            "max_acceleration_rad_s2": max_a,
            "waypoint_errors_m": waypoint_errors,
            "final_error_m": final_error,
            "dense_path_length_m": sum(spacings),
        }
