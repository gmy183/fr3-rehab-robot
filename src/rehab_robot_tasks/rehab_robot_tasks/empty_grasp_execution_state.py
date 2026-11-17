from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import math
import os
import tempfile
import time
import uuid


SCHEMA_VERSION = 1

STAGES = (
    "preflight_readonly",
    "xhand_single_joint_0p005",
    "xhand_whole_hand_0p005",
    "fr3_relative_pregrasp",
    "xhand_grasp",
    "fr3_relative_grasp",
    "fr3_rehab_stage_1",
    "controlled_finish",
)

STATUS_ARMED = "armed"
STATUS_RUNNING = "running"
STATUS_FAULTED = "faulted"
STATUS_COMPLETE = "complete"

VALID_STATUS = {
    STATUS_ARMED,
    STATUS_RUNNING,
    STATUS_FAULTED,
    STATUS_COMPLETE,
}


class ExecutionStateError(RuntimeError):
    pass


def _finite_vector(values, count, label):
    result = tuple(float(v) for v in values)

    if len(result) != count:
        raise ExecutionStateError(
            f"{label} must contain {count} values"
        )

    if not all(
        math.isfinite(v)
        for v in result
    ):
        raise ExecutionStateError(
            f"{label} contains non-finite value"
        )

    return result


def _make_fr3_targets(q_start):
    q0 = _finite_vector(
        q_start,
        7,
        "q_start",
    )

    q1 = list(q0)
    q1[0] += 0.005

    q2 = list(q1)
    q2[0] += 0.010

    q3 = list(q2)
    q3[0] += 0.005

    return {
        "q_start": list(q0),
        "q1_pregrasp": q1,
        "q2_grasp": q2,
        "q3_rehab": q3,
    }


def create_new_state(
    *,
    q_start,
    run_id=None,
):
    if run_id is None:
        run_id = (
            time.strftime("%Y%m%d_%H%M%S")
            + "_"
            + uuid.uuid4().hex[:8]
        )

    targets = _make_fr3_targets(
        q_start
    )

    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": str(run_id),
        "status": STATUS_ARMED,
        "created_unix_s": time.time(),
        "updated_unix_s": time.time(),

        # Important:
        # This anchor NEVER changes for one execution run.
        "fr3_anchor": targets,

        "completed_stages": [],
        "active_stage": None,
        "fault": None,
    }


def validate_state(state):
    if not isinstance(state, dict):
        raise ExecutionStateError(
            "execution state must be a dict"
        )

    if int(
        state.get(
            "schema_version",
            -1,
        )
    ) != SCHEMA_VERSION:
        raise ExecutionStateError(
            "unsupported execution-state schema"
        )

    if state.get("status") not in VALID_STATUS:
        raise ExecutionStateError(
            "invalid execution status"
        )

    anchor = state.get(
        "fr3_anchor",
        {},
    )

    for key in (
        "q_start",
        "q1_pregrasp",
        "q2_grasp",
        "q3_rehab",
    ):
        if key not in anchor:
            raise ExecutionStateError(
                f"missing FR3 anchor {key}"
            )

        _finite_vector(
            anchor[key],
            7,
            key,
        )

    completed = tuple(
        state.get(
            "completed_stages",
            [],
        )
    )

    # completed_stages 必须严格是 STAGES 的前缀。
    expected_prefix = (
        STAGES[
            :len(completed)
        ]
    )

    if completed != expected_prefix:
        raise ExecutionStateError(
            "completed_stages is not "
            "a valid ordered prefix"
        )

    active = state.get(
        "active_stage"
    )

    if (
        active is not None
        and active not in STAGES
    ):
        raise ExecutionStateError(
            "invalid active_stage"
        )

    return state


def next_stage(state):
    validate_state(state)

    completed = state[
        "completed_stages"
    ]

    if len(completed) == len(STAGES):
        return None

    return STAGES[
        len(completed)
    ]


def mark_stage_started(
    state,
    stage,
):
    validate_state(state)

    if state["status"] == STATUS_COMPLETE:
        raise ExecutionStateError(
            "completed run cannot restart"
        )

    expected = next_stage(
        state
    )

    if stage != expected:
        raise ExecutionStateError(
            f"cannot start {stage}; "
            f"expected {expected}"
        )

    result = dict(state)

    result[
        "completed_stages"
    ] = list(
        state["completed_stages"]
    )

    result["status"] = (
        STATUS_RUNNING
    )

    result["active_stage"] = stage

    result["fault"] = None

    result["updated_unix_s"] = (
        time.time()
    )

    validate_state(result)

    return result


def mark_stage_completed(
    state,
    stage,
):
    validate_state(state)

    if (
        state.get("active_stage")
        != stage
    ):
        raise ExecutionStateError(
            "stage completion does not "
            "match active stage"
        )

    expected = next_stage(
        state
    )

    if stage != expected:
        raise ExecutionStateError(
            "stage completion order invalid"
        )

    result = dict(state)

    completed = list(
        state["completed_stages"]
    )

    completed.append(stage)

    result[
        "completed_stages"
    ] = completed

    result["active_stage"] = None

    if len(completed) == len(STAGES):
        result["status"] = (
            STATUS_COMPLETE
        )
    else:
        result["status"] = (
            STATUS_ARMED
        )

    result["updated_unix_s"] = (
        time.time()
    )

    validate_state(result)

    return result


def mark_fault(
    state,
    *,
    reason,
):
    validate_state(state)

    result = dict(state)

    result[
        "completed_stages"
    ] = list(
        state["completed_stages"]
    )

    # 保留 active_stage。
    # 这很重要：
    # 说明故障发生时哪个阶段可能部分执行。
    result["status"] = (
        STATUS_FAULTED
    )

    result["fault"] = {
        "reason": str(reason),
        "unix_s": time.time(),
    }

    result["updated_unix_s"] = (
        time.time()
    )

    validate_state(result)

    return result


def expected_pose_after_completed(
    state,
):
    validate_state(state)

    completed = set(
        state["completed_stages"]
    )

    anchor = state[
        "fr3_anchor"
    ]

    if (
        "fr3_rehab_stage_1"
        in completed
    ):
        return tuple(
            anchor["q3_rehab"]
        )

    if (
        "fr3_relative_grasp"
        in completed
    ):
        return tuple(
            anchor["q2_grasp"]
        )

    if (
        "fr3_relative_pregrasp"
        in completed
    ):
        return tuple(
            anchor["q1_pregrasp"]
        )

    return tuple(
        anchor["q_start"]
    )


def pose_error(
    current,
    expected,
):
    a = _finite_vector(
        current,
        7,
        "current",
    )

    b = _finite_vector(
        expected,
        7,
        "expected",
    )

    return max(
        abs(x - y)
        for x, y in zip(a, b)
    )


def require_resume_pose(
    state,
    current,
    *,
    tolerance_rad=0.010,
):
    validate_state(state)

    if state["status"] != STATUS_FAULTED:
        raise ExecutionStateError(
            "resume requires faulted state"
        )

    # 如果故障时某一阶段已经 started
    # 但未 completed，则绝不自动猜测它完成了多少。
    if state.get(
        "active_stage"
    ) is not None:
        raise ExecutionStateError(
            "fault occurred inside active stage; "
            "manual stage audit required before resume"
        )

    expected = (
        expected_pose_after_completed(
            state
        )
    )

    error = pose_error(
        current,
        expected,
    )

    if error > float(
        tolerance_rad
    ):
        raise ExecutionStateError(
            "live FR3 pose does not match "
            f"saved checkpoint: error={error:.6f} rad"
        )

    return error


def save_state(
    state,
    path,
):
    validate_state(state)

    path = Path(path)

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    payload = dict(state)

    payload["updated_unix_s"] = (
        time.time()
    )

    fd, tmp_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
    )

    try:
        with os.fdopen(
            fd,
            "w",
            encoding="utf-8",
        ) as f:
            json.dump(
                payload,
                f,
                indent=2,
                sort_keys=True,
            )

            f.write("\n")

            f.flush()

            os.fsync(
                f.fileno()
            )

        os.replace(
            tmp_name,
            path,
        )

    finally:
        if os.path.exists(
            tmp_name
        ):
            os.unlink(
                tmp_name
            )


def load_state(path):
    path = Path(path)

    state = json.loads(
        path.read_text(
            encoding="utf-8"
        )
    )

    validate_state(state)

    return state
