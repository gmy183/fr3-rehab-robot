from __future__ import annotations

import json
import math
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .config import V3Config, expand_path
from .safety_supervisor import SafetySupervisor
from .teach_path import TeachPath, distance
from .trajectory_manager import (
    PlanningServer,
    RosMoveItPlanner,
    JOINT_NAMES,
    JOINT_LIMITS,
    extract_moveit_trajectory,
    evaluate_segment,
    max_abs_delta,
    quat_angle_deg,
)


class FR3TestError(RuntimeError):
    pass


QUINTIC_SMOOTHSTEP_VMAX = 1.875
QUINTIC_SMOOTHSTEP_AMAX = 5.773502691896258


def rest_to_rest_segment_duration(
    q0: Sequence[float],
    q1: Sequence[float],
    vmax_limit: float,
    amax_limit: float,
    *,
    minimum_s: float = 0.20,
    margin: float = 1.08,
) -> float:
    """Conservative duration for a zero-v/zero-a quintic segment.

    All joints use the same normalized smoothstep, so each segment is a
    straight line in joint space with exact rest-to-rest boundary conditions.
    """
    dq = max_abs_delta(q0, q1)
    if dq <= 1e-12:
        return float(minimum_s)
    if vmax_limit <= 0.0 or amax_limit <= 0.0:
        raise FR3TestError("non-positive short-test dynamic limit")
    t_v = QUINTIC_SMOOTHSTEP_VMAX * dq / float(vmax_limit)
    t_a = math.sqrt(QUINTIC_SMOOTHSTEP_AMAX * dq / float(amax_limit))
    return max(float(minimum_s), t_v, t_a) * float(margin)


class FR3TestRunner:
    """Constrained FR3-only real-motion validation for V3.1.

    This mode intentionally bypasses the *production* REAL gate without changing it.
    It executes exactly one short Cartesian trajectory after the effort JTC is
    activated and allowed to settle. XHand is never commanded.

    No automatic retry, return motion, ErrorRecovery, or PTP is permitted.
    """

    TEST_DISTANCE_M = 0.005
    TEST_MAX_JOINT_VELOCITY = 0.005
    TEST_MAX_JOINT_ACCELERATION = 0.025
    SETTLE_INITIAL_S = 2.0
    SETTLE_SAMPLE_S = 1.5
    SETTLE_MAX_JOINT_RANGE_RAD = 0.0015
    ACTIVATION_MAX_CART_SHIFT_M = 0.020
    ACTIVATION_MAX_JOINT_SHIFT_RAD = 0.030
    START_REVALIDATE_RAD = 0.003
    ACTION_PATH_TOL_RAD = 0.050
    ACTION_GOAL_TOL_RAD = 0.020
    HARD_FINAL_JOINT_ERROR_RAD = 0.030
    HARD_FINAL_CART_ERROR_M = 0.012
    QUALITY_FINAL_JOINT_ERROR_RAD = 0.005
    QUALITY_FINAL_CART_ERROR_M = 0.003

    CONFIRM = "EXECUTE-FR3-V3.1-5MM"

    def __init__(self, config: V3Config):
        self.config = config
        self.safety = SafetySupervisor(config)
        self.teach = TeachPath(config.task["teach_points"], config.task["route_order"])
        self.runtime_dir = config.root / "validation_logs" / "runtime"
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.result_path = self.runtime_dir / f"rehab_v3_1_fr3_test_{self.stamp}.json"
        self.plan_path = self.runtime_dir / f"rehab_v3_1_fr3_test_plan_{self.stamp}.json"
        self.moveit_log = self.runtime_dir / f"rehab_v3_1_fr3_test_moveit_{self.stamp}.log"
        self.loaded = False
        self.active = False
        self.result: Dict[str, Any] = {
            "schema": "rehab_v3_1_2_fr3_test",
            "mode": "fr3-test",
            "test_distance_m": self.TEST_DISTANCE_M,
            "xhand_commanded": False,
            "ptp_used": False,
            "error_recovery_used": False,
            "auto_retry": False,
            "auto_return": False,
            "production_real_gate_changed": False,
            "production_tracking_backend_validated": False,
        }

    @staticmethod
    def _run(args: Sequence[str], timeout: float = 8.0) -> tuple[int, str]:
        try:
            p = subprocess.run(
                list(args),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )
            return p.returncode, p.stdout
        except subprocess.TimeoutExpired as exc:
            out = exc.stdout or ""
            if isinstance(out, bytes):
                out = out.decode(errors="replace")
            return 124, out + "\n[TIMEOUT]"

    def _save(self) -> None:
        self.result_path.write_text(
            json.dumps(self.result, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def _controller_state(self) -> Optional[str]:
        rc, text = self._run(["ros2", "control", "list_controllers"], timeout=5.0)
        if rc != 0:
            return None
        for line in text.splitlines():
            parts = line.split()
            if parts and parts[0] == str(self.config.hardware["fr3"]["trajectory_controller"]):
                return parts[-1]
        return None

    def _register_load_configure(self) -> None:
        fr3 = self.config.hardware["fr3"]
        cm = str(fr3["controller_manager"])
        name = str(fr3["trajectory_controller"])
        ctype = str(fr3["controller_type"])
        params = expand_path(str(fr3["controller_param_file"]))

        rc, out = self._run(["ros2", "param", "set", cm, f"{name}.type", ctype])
        print(out.rstrip())
        if rc != 0 or "successful" not in out.lower():
            raise FR3TestError("failed to register trajectory controller type")

        rc, current = self._run(["ros2", "param", "get", cm, f"{name}.params_file"])
        if params not in current:
            rc, out = self._run(
                ["ros2", "param", "set", cm, f"{name}.params_file", f"['{params}']"]
            )
            print(out.rstrip())
            if rc != 0 or "successful" not in out.lower():
                raise FR3TestError("failed to register trajectory controller params_file")

        rc, out = self._run([
            "ros2", "service", "call", f"{cm}/load_controller",
            "controller_manager_msgs/srv/LoadController", f"{{name: '{name}'}}",
        ])
        print(out.rstrip())
        if rc != 0 or not ("ok=True" in out or "ok: true" in out):
            raise FR3TestError("load_controller failed")
        self.loaded = True

        rc, out = self._run([
            "ros2", "service", "call", f"{cm}/configure_controller",
            "controller_manager_msgs/srv/ConfigureController", f"{{name: '{name}'}}",
        ])
        print(out.rstrip())
        if rc != 0 or not ("ok=True" in out or "ok: true" in out):
            raise FR3TestError("configure_controller failed")

        if self._controller_state() != "inactive":
            raise FR3TestError("trajectory controller did not reach inactive state")

    def _activate(self) -> None:
        fr3 = self.config.hardware["fr3"]
        cm = str(fr3["controller_manager"])
        name = str(fr3["trajectory_controller"])
        rc, out = self._run([
            "ros2", "control", "switch_controllers",
            "--activate", name,
            "--strict",
            "-c", cm,
        ])
        print(out.rstrip())
        if rc != 0 or self._controller_state() != "active":
            raise FR3TestError("trajectory controller activation failed")
        self.active = True

    def _cleanup_controller(self) -> None:
        fr3 = self.config.hardware["fr3"]
        cm = str(fr3["controller_manager"])
        name = str(fr3["trajectory_controller"])
        if self.active:
            print("[SAFE] deactivating fr3_arm_controller")
            self._run([
                "ros2", "control", "switch_controllers",
                "--deactivate", name,
                "--strict",
                "-c", cm,
            ])
            self.active = False
        if self.loaded:
            print("[SAFE] unloading fr3_arm_controller")
            self._run(["ros2", "control", "unload_controller", "-c", cm, name])
            self.loaded = False

    def _settle(self, planner: RosMoveItPlanner, before) -> Any:
        print(f"[WAIT] effort controller equilibrium settling {self.SETTLE_INITIAL_S:.1f} s")
        time.sleep(self.SETTLE_INITIAL_S)
        samples: List[List[float]] = []
        deadline = time.monotonic() + self.SETTLE_SAMPLE_S
        while time.monotonic() < deadline:
            samples.append(planner.capture_live_state().q)
            time.sleep(0.04)
        after = planner.capture_live_state()
        if len(samples) < 8:
            raise FR3TestError("insufficient active-hold settling samples")

        ranges = [max(q[j] for q in samples) - min(q[j] for q in samples) for j in range(7)]
        max_range = max(ranges)
        joint_shift = max_abs_delta(before.q, after.q)
        cart_shift = distance(before.xyz, after.xyz)

        print(f"activation Cartesian shift = {cart_shift * 1000.0:.3f} mm")
        print(f"activation max joint shift = {joint_shift:.6f} rad")
        print(f"active-hold max joint range = {max_range:.6f} rad")

        self.result["activation"] = {
            "cartesian_shift_m": cart_shift,
            "max_joint_shift_rad": joint_shift,
            "max_hold_range_rad": max_range,
        }

        if cart_shift > self.ACTIVATION_MAX_CART_SHIFT_M:
            raise FR3TestError("activation Cartesian shift exceeded 20 mm hard gate")
        if joint_shift > self.ACTIVATION_MAX_JOINT_SHIFT_RAD:
            raise FR3TestError("activation joint shift exceeded 0.030 rad hard gate")
        if max_range > self.SETTLE_MAX_JOINT_RANGE_RAD:
            raise FR3TestError("effort controller did not settle")
        print("[PASS] ACTIVE-HOLD equilibrium stable")
        return after

    def _build_target(self, active_state) -> tuple[float, float, float]:
        p2 = self.teach.points["P2"]
        vector = [p2[i] - active_state.xyz[i] for i in range(3)]
        remaining = math.sqrt(sum(v * v for v in vector))
        if remaining < 0.020:
            raise FR3TestError("current pose is too close to P2 for constrained 5 mm test")
        direction = [v / remaining for v in vector]
        return tuple(active_state.xyz[i] + self.TEST_DISTANCE_M * direction[i] for i in range(3))

    @staticmethod
    def _duration_msg(seconds: float):
        from builtin_interfaces.msg import Duration
        sec = int(math.floor(seconds))
        ns = int(round((seconds - sec) * 1e9))
        if ns >= 1_000_000_000:
            sec += 1
            ns -= 1_000_000_000
        msg = Duration()
        msg.sec = sec
        msg.nanosec = ns
        return msg

    def _plan_short_rest_to_rest(
        self,
        planner: RosMoveItPlanner,
        start,
        target: Sequence[float],
    ) -> Dict[str, Any]:
        """Plan one 5-mm Cartesian move and retime its IK knots rest-to-rest.

        V3.1.2 deliberately does NOT reuse MoveIt's returned velocities and
        accelerations for this short hardware test.  The old V3.1 path fed
        those derivatives into quintic interpolation; on a 5-mm path that can
        produce a spline that departs from the Cartesian line/orientation even
        when the MoveIt knots themselves are valid.

        Here MoveIt supplies only the Cartesian IK knot positions.  Every
        adjacent knot is then connected with the exact same zero-v/zero-a
        quintic that is audited and later sent to FollowJointTrajectory.
        Production V3 planning remains unchanged.
        """
        if not planner.cart_client.wait_for_service(timeout_sec=5.0):
            raise FR3TestError("GetCartesianPath service unavailable")
        if not planner.fk_client.wait_for_service(timeout_sec=5.0):
            raise FR3TestError("FK service unavailable")

        # V3.1.3 coherence gate: the live q and live TF must describe the
        # same robot state BEFORE asking MoveIt to generate the 5-mm path.
        # This turns stale-state bugs into an explicit fail-closed condition.
        start_fk_xyz, start_fk_quat = planner.fk(start.q)
        live_sync_cart = distance(start_fk_xyz, start.xyz)
        live_sync_ori = quat_angle_deg(start_fk_quat, start.quat)
        print(f"live q/TF coherence error = {live_sync_cart * 1000.0:.3f} mm")
        print(f"live q/TF orientation error = {live_sync_ori:.6f} deg")
        if live_sync_cart > 0.0005:
            raise FR3TestError(
                f"live state incoherent before planning: "
                f"FK(q)-TF={live_sync_cart * 1000.0:.3f} mm > 0.500 mm"
            )
        if live_sync_ori > 0.10:
            raise FR3TestError(
                f"live state orientation incoherent before planning: "
                f"{live_sync_ori:.6f} deg > 0.100 deg"
            )

        from moveit_msgs.msg import RobotState

        req = planner.GetCartesianPath.Request()
        req.header.frame_id = str(self.config.task["base_frame"])
        req.group_name = "fr3_arm"
        req.link_name = str(self.config.task["tool_frame"])
        req.max_step = 0.0005
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = bool(self.config.safety["planning"]["avoid_collisions"])
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = 0.05
        if hasattr(req, "max_acceleration_scaling_factor"):
            req.max_acceleration_scaling_factor = 0.05

        st = RobotState()
        st.joint_state.name = list(JOINT_NAMES)
        st.joint_state.position = [float(x) for x in start.q]
        req.start_state = st
        req.waypoints = [planner._pose(target, start.quat)]

        fut = planner.cart_client.call_async(req)
        planner.rclpy.spin_until_future_complete(planner.node, fut, timeout_sec=30.0)
        if not fut.done() or fut.result() is None:
            raise FR3TestError("5-mm GetCartesianPath timeout")
        res = fut.result()
        if hasattr(res, "error_code") and int(res.error_code.val) != 1:
            raise FR3TestError(f"MoveIt error_code={res.error_code.val}")

        fraction = float(res.fraction)
        if fraction < 0.999:
            raise FR3TestError(f"5-mm Cartesian fraction={fraction:.6f} < 0.999")

        raw = extract_moveit_trajectory(res.solution.joint_trajectory)
        if max_abs_delta(raw[0]["positions"], start.q) > 0.003:
            raise FR3TestError("MoveIt 5-mm path start does not match active state")

        # Use only MoveIt's IK knot positions.  Remove numerical duplicates.
        knots: List[List[float]] = []
        for item in raw:
            q = [float(x) for x in item["positions"]]
            if not knots or max_abs_delta(knots[-1], q) > 1e-9:
                knots.append(q)
        if len(knots) < 2:
            raise FR3TestError("MoveIt 5-mm path has fewer than two unique IK knots")

        vmax_limit = self.TEST_MAX_JOINT_VELOCITY
        amax_limit = self.TEST_MAX_JOINT_ACCELERATION
        zero = [0.0] * 7
        points: List[Dict[str, Any]] = [
            {
                "time": 0.0,
                "positions": list(knots[0]),
                "velocities": list(zero),
                "accelerations": list(zero),
            }
        ]
        t = 0.0
        for q0, q1 in zip(knots[:-1], knots[1:]):
            dt = rest_to_rest_segment_duration(
                q0,
                q1,
                vmax_limit,
                amax_limit,
            )
            t += dt
            points.append(
                {
                    "time": t,
                    "positions": list(q1),
                    "velocities": list(zero),
                    "accelerations": list(zero),
                }
            )

        # Dense audit of the EXACT quintics that will be sent to the JTC.
        target_spacing = 0.00025
        dense = []
        knot_xyz = [planner.fk(p["positions"])[0] for p in points]
        for i, (p0, p1) in enumerate(zip(points[:-1], points[1:])):
            cart = distance(knot_xyz[i], knot_xyz[i + 1])
            qstep = max_abs_delta(p0["positions"], p1["positions"])
            divisions = max(
                12,
                int(math.ceil(cart / target_spacing)),
                int(math.ceil(qstep / 0.00075)),
            )
            for k in range(divisions):
                q, v, a = evaluate_segment(p0, p1, k / divisions)
                xyz, qq = planner.fk(q)
                dense.append((q, v, a, xyz, qq))
        q = points[-1]["positions"]
        xyz, qq = planner.fk(q)
        dense.append((q, zero, zero, xyz, qq))

        # Straight-line distance from point to the single start->target segment.
        sx, sy, sz = map(float, start.xyz)
        tx, ty, tz = map(float, target)
        vx, vy, vz = tx - sx, ty - sy, tz - sz
        vv = vx*vx + vy*vy + vz*vz
        if vv <= 1e-15:
            raise FR3TestError("zero-length 5-mm audit segment")

        def cross_track(xyz_: Sequence[float]) -> float:
            px, py, pz = map(float, xyz_)
            wx, wy, wz = px - sx, py - sy, pz - sz
            u = max(0.0, min(1.0, (wx*vx + wy*vy + wz*vz) / vv))
            nx, ny, nz = sx + u*vx, sy + u*vy, sz + u*vz
            return math.sqrt((px-nx)**2 + (py-ny)**2 + (pz-nz)**2)

        max_cross = 0.0
        max_ori = 0.0
        max_v = 0.0
        max_a = 0.0
        min_margin = float("inf")
        min_joint = None
        for q, v, a, xyz, qq in dense:
            max_cross = max(max_cross, cross_track(xyz))
            max_ori = max(max_ori, quat_angle_deg(qq, start.quat))
            max_v = max(max_v, max(abs(x) for x in v))
            max_a = max(max_a, max(abs(x) for x in a))
            for name, value in zip(JOINT_NAMES, q):
                lo, hi = JOINT_LIMITS[name]
                if not (lo <= value <= hi):
                    raise FR3TestError(f"{name} outside software joint limit")
                margin = min(value - lo, hi - value)
                if margin < min_margin:
                    min_margin = margin
                    min_joint = name

        spacings = [distance(a[3], b[3]) for a, b in zip(dense[:-1], dense[1:])]
        max_spacing = max(spacings) if spacings else 0.0
        start_fk_error = distance(dense[0][3], start.xyz)
        final_error = distance(dense[-1][3], target)
        dense_path_length = sum(spacings)

        audit = {
            "dense_sample_count": len(dense),
            "moveit_unique_knot_count": len(knots),
            "max_fk_spacing_m": max_spacing,
            "start_fk_error_m": start_fk_error,
            "final_error_m": final_error,
            "max_cross_track_m": max_cross,
            "max_orientation_error_deg": max_ori,
            "minimum_joint_limit_margin_rad": min_margin,
            "minimum_joint_limit_margin_joint": min_joint,
            "max_velocity_rad_s": max_v,
            "max_acceleration_rad_s2": max_a,
            "dense_path_length_m": dense_path_length,
        }

        print("[V3.1.2] short path retimed from MoveIt IK positions only")
        print(f"fraction = {fraction:.6f}")
        print(f"MoveIt points = {len(raw)}, unique IK knots = {len(knots)}")
        print(f"retimed duration = {points[-1]['time']:.3f} s")
        print(f"dense samples = {len(dense)}")
        print(f"dense path length = {dense_path_length * 1000.0:.3f} mm")
        print(f"max FK spacing = {max_spacing * 1000.0:.3f} mm")
        print(f"start FK error = {start_fk_error * 1000.0:.3f} mm")
        print(f"final target error = {final_error * 1000.0:.3f} mm")
        print(f"max cross-track = {max_cross * 1000.0:.3f} mm")
        print(f"max orientation error = {max_ori:.6f} deg")
        print(f"minimum joint margin = {min_margin:.6f} rad ({min_joint})")
        print(f"max dense velocity = {max_v:.6f} rad/s")
        print(f"max dense acceleration = {max_a:.6f} rad/s^2")

        blockers = []
        if max_cross > 0.0005:
            blockers.append(f"cross-track {max_cross*1000.0:.3f} mm > 0.500 mm")
        if max_ori > 0.10:
            blockers.append(f"orientation {max_ori:.6f} deg > 0.100 deg")
        if max_spacing > 0.00075:
            blockers.append(f"FK spacing {max_spacing*1000.0:.3f} mm > 0.750 mm")
        if start_fk_error > 0.0005:
            blockers.append(f"start FK error {start_fk_error*1000.0:.3f} mm > 0.500 mm")
        if final_error > 0.0005:
            blockers.append(f"final error {final_error*1000.0:.3f} mm > 0.500 mm")
        if min_margin < 0.05:
            blockers.append(f"joint margin {min_margin:.6f} rad < 0.050 rad")
        if max_v > vmax_limit * 1.001:
            blockers.append(f"velocity {max_v:.6f} > {vmax_limit:.6f} rad/s")
        if max_a > amax_limit * 1.001:
            blockers.append(f"acceleration {max_a:.6f} > {amax_limit:.6f} rad/s^2")
        if blockers:
            raise FR3TestError("V3.1.2 short-path dense audit failed: " + "; ".join(blockers))

        print("[PASS] V3.1.2 exact short-trajectory dense audit")

        return {
            "schema": "rehab_v3_1_2_short_rest_to_rest",
            "stage": "fr3_test_5mm",
            "execution_enabled": False,
            "execution_eligible": True,
            "frame": str(self.config.task["base_frame"]),
            "tool": str(self.config.task["tool_frame"]),
            "orientation_mode": "hold_active_start_quaternion",
            "start_xyz": list(map(float, start.xyz)),
            "start_q": list(map(float, start.q)),
            "start_quaternion_xyzw": list(map(float, start.quat)),
            "waypoint_names": ["FR3_TEST_TARGET"],
            "waypoints_xyz": [list(map(float, target))],
            "fraction": fraction,
            "moveit_point_count": len(raw),
            "moveit_unique_knot_count": len(knots),
            "moveit_duration_s": raw[-1]["time"],
            "duration_s": points[-1]["time"],
            "joint_names": list(JOINT_NAMES),
            "trajectory_points": [
                {
                    "time_from_start_s": p["time"],
                    "positions": p["positions"],
                    "velocities": p["velocities"],
                    "accelerations": p["accelerations"],
                }
                for p in points
            ],
            "audit": audit,
            "end_q": list(points[-1]["positions"]),
            "end_xyz": list(map(float, target)),
        }

    def _execute_goal(self, planner: RosMoveItPlanner, plan: Dict[str, Any]) -> Dict[str, Any]:
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectoryPoint

        live = planner.capture_live_state()
        start_error = max_abs_delta(live.q, plan["start_q"])
        print(f"pre-goal active start q error = {start_error:.6f} rad")
        if start_error > self.START_REVALIDATE_RAD:
            raise FR3TestError("active equilibrium moved after planning; no goal sent")

        action_name = str(self.config.hardware["fr3"]["trajectory_action"])
        client = ActionClient(planner.node, FollowJointTrajectory, action_name)
        if not client.wait_for_server(timeout_sec=5.0):
            raise FR3TestError("FollowJointTrajectory server unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(plan["joint_names"])
        for item in plan["trajectory_points"]:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in item["positions"]]
            pt.velocities = [float(x) for x in item["velocities"]]
            pt.accelerations = [float(x) for x in item["accelerations"]]
            pt.time_from_start = self._duration_msg(float(item["time_from_start_s"]))
            goal.trajectory.points.append(pt)

        # Test-mode tolerances are explicit so ROS action success is not unconstrained.
        for name in plan["joint_names"]:
            path_tol = JointTolerance()
            path_tol.name = name
            path_tol.position = self.ACTION_PATH_TOL_RAD
            goal.path_tolerance.append(path_tol)
            goal_tol = JointTolerance()
            goal_tol.name = name
            goal_tol.position = self.ACTION_GOAL_TOL_RAD
            goal.goal_tolerance.append(goal_tol)
        goal.goal_time_tolerance = self._duration_msg(3.0)

        send_future = client.send_goal_async(goal)
        planner.rclpy.spin_until_future_complete(planner.node, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            raise FR3TestError("trajectory goal acceptance timeout")
        handle = send_future.result()
        if not handle.accepted:
            raise FR3TestError("trajectory goal rejected")
        print("[EXEC] ONE constrained FollowJointTrajectory goal accepted")

        result_future = handle.get_result_async()
        timeout = float(plan["duration_s"]) + 12.0
        planner.rclpy.spin_until_future_complete(planner.node, result_future, timeout_sec=timeout)
        if not result_future.done():
            print("[FAULT] trajectory result timeout; cancel requested once")
            cancel = handle.cancel_goal_async()
            planner.rclpy.spin_until_future_complete(planner.node, cancel, timeout_sec=3.0)
            raise FR3TestError("trajectory result timeout")

        wrapped = result_future.result()
        if wrapped is None:
            raise FR3TestError("missing trajectory result")
        status = int(wrapped.status)
        code = int(wrapped.result.error_code)
        text = str(wrapped.result.error_string)
        print(f"ROS action status = {status}")
        print(f"trajectory error_code = {code}")
        print(f"trajectory error_string = {text!r}")

        return {
            "ros_status": status,
            "error_code": code,
            "error_string": text,
            "action_succeeded": status == GoalStatus.STATUS_SUCCEEDED and code == 0,
        }

    def run(self) -> int:
        planner: Optional[RosMoveItPlanner] = None
        hard_fault = False
        try:
            print("=" * 72)
            print("REHAB ROBOT V3.1.2 FR3-TEST REAL-MOTION MODE")
            print("FR3 ONLY / ONE 5 mm TRAJECTORY / XHAND DISABLED")
            print("production REAL gate remains CLOSED")
            print("=" * 72)

            baseline = self.safety.check_runtime(require_idle=True, require_no_motion_controller=True)
            if not baseline.ok:
                raise FR3TestError("baseline safety check failed: " + "; ".join(baseline.blockers))
            print("[PASS] baseline safety check")
            self.result["baseline"] = baseline.data

            # This is intentionally independent of production real_mode_gate().
            production_gate = self.safety.real_mode_gate()
            self.result["production_real_gate_ok"] = production_gate.ok
            self.result["production_real_gate_blockers"] = production_gate.blockers
            print("[PASS] production REAL gate left unchanged/closed")

            planner = RosMoveItPlanner(self.config, self.teach)
            before = planner.capture_live_state()
            self.result["preactivation_xyz"] = list(before.xyz)
            self.result["preactivation_q"] = list(before.q)

            print()
            print("REAL-MOTION SAFETY GATE")
            print("The effort controller activation itself previously shifted the arm ~7 mm.")
            print("Confirm Desk=EXEC, FCI=ON, joints unlocked, hand-guiding OFF,")
            print("workspace completely clear, NO human/ankle contact, XHand not commanded.")
            print(f"Type exactly: {self.CONFIRM}")
            if input().strip() != self.CONFIRM:
                raise FR3TestError("user confirmation not received")

            print("[STEP] load/configure official effort JointTrajectoryController")
            self._register_load_configure()
            print("[STEP] activate trajectory controller")
            self._activate()
            active_state = self._settle(planner, before)

            target = self._build_target(active_state)
            self.result["active_start_xyz"] = list(active_state.xyz)
            self.result["test_target_xyz"] = list(target)
            print(f"active start xyz = {[round(x, 6) for x in active_state.xyz]}")
            print(f"5 mm target xyz = {[round(x, 6) for x in target]}")

            print("[TEST PROFILE] V3.1.2 exact rest-to-rest short trajectory")
            print("[TEST PROFILE] MoveIt Cartesian IK step = 0.5 mm")
            print("[TEST PROFILE] dense FK target = 0.25 mm")
            print("[TEST PROFILE] audited trajectory == transmitted trajectory")

            with PlanningServer(self.config, self.moveit_log):
                # Re-capture AFTER MoveIt starts while JTC remains active.
                start = planner.capture_live_state()
                print(f"planning start xyz = {[round(x, 6) for x in start.xyz]}")
                print(f"planning target xyz = {[round(x, 6) for x in target]}")
                plan = self._plan_short_rest_to_rest(planner, start, target)

            self.plan_path.write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding="utf-8")
            self.result["plan_json"] = str(self.plan_path)
            self.result["moveit_log"] = str(self.moveit_log)
            self.result["planned_duration_s"] = plan["duration_s"]
            self.result["plan_audit"] = plan["audit"]
            print(f"[PASS] active-state 5 mm plan audited, duration={plan['duration_s']:.3f}s")

            action_result = self._execute_goal(planner, plan)
            self.result["action"] = action_result

            final = planner.capture_live_state()
            final_joint_error = max_abs_delta(final.q, plan["end_q"])
            final_cart_error = distance(final.xyz, target)
            quality_ok = (
                final_joint_error <= self.QUALITY_FINAL_JOINT_ERROR_RAD
                and final_cart_error <= self.QUALITY_FINAL_CART_ERROR_M
            )
            hard_envelope_ok = (
                final_joint_error <= self.HARD_FINAL_JOINT_ERROR_RAD
                and final_cart_error <= self.HARD_FINAL_CART_ERROR_M
            )
            self.result["final"] = {
                "xyz": list(final.xyz),
                "max_joint_error_rad": final_joint_error,
                "cartesian_target_error_m": final_cart_error,
                "tracking_quality_pass": quality_ok,
                "hard_test_envelope_pass": hard_envelope_ok,
            }
            print(f"final max joint error = {final_joint_error:.6f} rad")
            print(f"final Cartesian target error = {final_cart_error * 1000.0:.3f} mm")

            if not action_result["action_succeeded"]:
                raise FR3TestError("FollowJointTrajectory action did not succeed")
            if not hard_envelope_ok:
                raise FR3TestError("final tracking error exceeded FR3-test hard envelope")

            print("[PASS] REAL TRAJECTORY CHANNEL completed inside FR3-test hard envelope")
            if quality_ok:
                print("[PASS] tracking quality also meets production-quality threshold")
            else:
                print("[WARN] tracking quality does NOT meet production threshold")
                print("[SAFE] fr3.tracking_backend_validated remains false")

            self.result["completed"] = True
            self.result["tracking_backend_validated"] = False
            self._save()
            return 0

        except Exception as exc:
            hard_fault = True
            self.result["completed"] = False
            self.result["fault"] = f"{type(exc).__name__}: {exc}"
            print(f"[FAULT] {type(exc).__name__}: {exc}")
            print("[SAFE] no retry / no auto-return / no PTP / no ErrorRecovery")
            self._save()
            return 2

        finally:
            self._cleanup_controller()
            if planner is not None:
                try:
                    planner.close()
                except Exception:
                    pass
            post = self.safety.check_runtime(require_idle=True, require_no_motion_controller=True)
            self.result["post_cleanup"] = {
                "ok": post.ok,
                "blockers": post.blockers,
                "data": post.data,
            }
            self.result["hard_fault"] = hard_fault
            self._save()
            print(f"result_json={self.result_path}")
            print("[SAFE] production REAL/XHand gates were not changed")
