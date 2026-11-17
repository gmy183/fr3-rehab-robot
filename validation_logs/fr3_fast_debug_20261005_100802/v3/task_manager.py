from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List
import json

from .config import V3Config
from .contracts import RunMode, StepRecord, TaskResult, TaskState
from .fr3_motion_executor import FR3MotionExecutor
from .safety_supervisor import SafetySupervisor
from .state_machine import RehabStateMachine
from .teach_path import TeachPath
from .trajectory_manager import PlanningServer, RosMoveItPlanner
from .xhand_executor import XHandExecutor


class RehabTaskManager:
    def __init__(self, config: V3Config, mode: RunMode):
        self.config = config
        self.mode = mode
        self.sm = RehabStateMachine()
        self.steps: List[StepRecord] = []
        self.artifacts: Dict[str, str] = {}
        self.safety = SafetySupervisor(config)
        self.teach = TeachPath(config.task["teach_points"], config.task["route_order"])
        self.fr3 = FR3MotionExecutor(config)
        self.xhand = XHandExecutor(config, dry_run=(mode != RunMode.REAL))
        self.runtime_dir = config.root / "validation_logs" / "runtime"
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    def _record(self, state: TaskState, ok: bool, message: str, data: Dict[str, Any] | None = None):
        self.steps.append(StepRecord(state=state.value, ok=ok, message=message, data=data))
        print(f"[{'PASS' if ok else 'FAIL'}] {state.value}: {message}")

    def _save(self, result: TaskResult) -> Path:
        out = self.runtime_dir / f"rehab_v3_{self.mode.value.replace('-', '_')}_{self.stamp}.json"
        out.write_text(json.dumps(result.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        self.artifacts["task_result"] = str(out)
        # Rewrite once so artifacts contain the result itself.
        result.artifacts = dict(self.artifacts)
        out.write_text(json.dumps(result.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        return out

    def run(self) -> TaskResult:
        try:
            if self.mode == RunMode.CHECK:
                return self._run_check()
            if self.mode == RunMode.DRY_RUN:
                return self._run_dry_run()
            if self.mode == RunMode.REAL:
                return self._run_real()
            raise RuntimeError(f"unsupported mode: {self.mode}")
        except Exception as exc:
            if self.sm.state != TaskState.FAULT:
                self.sm.fault()
            self._record(TaskState.FAULT, False, f"{type(exc).__name__}: {exc}")
            result = TaskResult(
                mode=self.mode.value,
                ok=False,
                final_state=self.sm.state.value,
                fault_latched=self.sm.fault_latched,
                steps=self.steps,
                artifacts=self.artifacts,
            )
            path = self._save(result)
            print(f"[SAFE] FAULT latched; no retry / no auto-return / no auto-release")
            print(f"result_json={path}")
            return result

    def _run_check(self) -> TaskResult:
        report = self.safety.check_runtime(require_idle=True, require_no_motion_controller=True)
        if not report.ok:
            raise RuntimeError("runtime safety check failed: " + "; ".join(report.blockers))
        self._record(TaskState.READY, True, "configuration and ROS/FR3 baseline check passed", report.data)
        gate = self.safety.real_mode_gate()
        self._record(
            TaskState.READY,
            True,
            "production motion gates intentionally closed" if not gate.ok else "production motion gates open",
            {"real_gate_ok": gate.ok, "blockers": gate.blockers, **gate.data},
        )
        result = TaskResult(
            mode=self.mode.value,
            ok=True,
            final_state=self.sm.state.value,
            fault_latched=False,
            steps=self.steps,
            artifacts=self.artifacts,
        )
        path = self._save(result)
        print(f"result_json={path}")
        return result

    def _save_stage(self, name: str, plan: Dict[str, Any]) -> Path:
        p = self.runtime_dir / f"rehab_v3_{name}_{self.stamp}.json"
        p.write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding="utf-8")
        self.artifacts[f"{name}_plan"] = str(p)
        return p

    def _run_dry_run(self) -> TaskResult:
        baseline = self.safety.check_runtime(require_idle=True, require_no_motion_controller=True)
        if not baseline.ok:
            raise RuntimeError("dry-run preflight failed: " + "; ".join(baseline.blockers))
        self._record(TaskState.READY, True, "dry-run preflight passed", baseline.data)

        moveit_log = self.runtime_dir / f"rehab_v3_moveit_{self.stamp}.log"
        self.artifacts["moveit_log"] = str(moveit_log)
        with PlanningServer(self.config, moveit_log):
            planner = RosMoveItPlanner(self.config, self.teach)
            try:
                live = planner.capture_live_state()

                # APPROACH: current -> P1. If current is already on a taught edge,
                # route through the remaining taught point(s) toward P1 so that the
                # approach remains consistent with the taught geometry.
                approach_target = str(self.config.task["approach_target"])
                approach_names = self.teach.remaining_waypoints_from_current(live.xyz, approach_target)
                if not approach_names:
                    approach_names = [approach_target]
                self.sm.transition(TaskState.APPROACH)
                approach = planner.plan_stage("approach", live.q, live.xyz, live.quat, approach_names)
                self._save_stage("approach", approach)
                self._record(TaskState.APPROACH, True, f"planned/audited {approach['duration_s']:.3f}s continuous approach", approach["audit"])

                self.sm.transition(TaskState.GRASP)
                grasp_msg = self.xhand.grasp()
                self._record(TaskState.GRASP, True, grasp_msg)

                self.sm.transition(TaskState.REHAB_MOTION)
                rehab_start = str(self.config.task["rehab_start"])
                rehab_end = str(self.config.task["rehab_end"])
                rehab_route = self.teach.route(rehab_start, rehab_end)
                rehab_waypoints = rehab_route[1:] if rehab_route and rehab_route[0] == rehab_start else rehab_route
                rehab = planner.plan_stage(
                    "rehab_motion",
                    approach["end_q"],
                    self.teach.points[rehab_start],
                    live.quat,
                    rehab_waypoints,
                )
                self._save_stage("rehab_motion", rehab)
                self._record(TaskState.REHAB_MOTION, True, f"planned/audited {rehab['duration_s']:.3f}s P1-P8 continuous rehab path", rehab["audit"])

                self.sm.transition(TaskState.RETURN)
                return_target = str(self.config.task["return_target"])
                return_route = self.teach.route(rehab_end, return_target)
                return_waypoints = return_route[1:] if return_route and return_route[0] == rehab_end else return_route
                ret = planner.plan_stage(
                    "return",
                    rehab["end_q"],
                    self.teach.points[rehab_end],
                    live.quat,
                    return_waypoints,
                )
                self._save_stage("return", ret)
                self._record(TaskState.RETURN, True, f"planned/audited {ret['duration_s']:.3f}s continuous return", ret["audit"])
            finally:
                planner.close()

        self.sm.transition(TaskState.RELEASE)
        release_msg = self.xhand.release()
        self._record(TaskState.RELEASE, True, release_msg)
        self.sm.transition(TaskState.FINISH)
        self._record(TaskState.FINISH, True, "V3 dry-run state machine completed; no motion command was sent")

        result = TaskResult(
            mode=self.mode.value,
            ok=True,
            final_state=self.sm.state.value,
            fault_latched=False,
            steps=self.steps,
            artifacts=self.artifacts,
        )
        path = self._save(result)
        print(f"result_json={path}")
        return result

    def _run_real(self) -> TaskResult:
        # The real gate is evaluated before loading/activating any motion controller.
        baseline = self.safety.check_runtime(require_idle=True, require_no_motion_controller=True)
        if not baseline.ok:
            raise RuntimeError("real-mode baseline failed: " + "; ".join(baseline.blockers))
        gate = self.safety.real_mode_gate()
        if not gate.ok:
            raise RuntimeError("REAL mode hard-blocked: " + "; ".join(gate.blockers))
        # Even after gates are eventually opened, the executor refuses until a
        # validated backend is explicitly commissioned. This avoids accidentally
        # reusing the currently inaccurate effort-JTC path.
        raise RuntimeError("REAL mode backend not commissioned in V3 release")
