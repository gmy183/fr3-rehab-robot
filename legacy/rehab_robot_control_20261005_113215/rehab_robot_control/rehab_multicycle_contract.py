from __future__ import annotations

import math

from rehab_robot_control.rehab_microcycle_executor import (
    build_cycle_targets,
)


ALLOWED_CYCLES = (1, 3, 5)

DEFAULT_HOLD_S = 0.5
MAX_HOLD_S = 5.0

CYCLE_START_TOLERANCE_RAD = 0.010


class RehabMultiCycleContractError(RuntimeError):
    pass


def validate_multicycle_parameters(
    cycles,
    hold_s,
):
    try:
        cycles = int(cycles)
    except Exception as exc:
        raise RehabMultiCycleContractError(
            "cycles must be an integer"
        ) from exc

    if cycles not in ALLOWED_CYCLES:
        raise RehabMultiCycleContractError(
            "cycles must be one of "
            f"{ALLOWED_CYCLES}"
        )

    try:
        hold_s = float(hold_s)
    except Exception as exc:
        raise RehabMultiCycleContractError(
            "hold_s must be numeric"
        ) from exc

    if not math.isfinite(hold_s):
        raise RehabMultiCycleContractError(
            "hold_s must be finite"
        )

    if not 0.0 <= hold_s <= MAX_HOLD_S:
        raise RehabMultiCycleContractError(
            "hold_s must be in "
            f"[0, {MAX_HOLD_S}] s"
        )

    return cycles, hold_s


def pose_error(
    current,
    expected,
):
    current = tuple(
        float(v)
        for v in current
    )

    expected = tuple(
        float(v)
        for v in expected
    )

    if len(current) != 7:
        raise RehabMultiCycleContractError(
            "current FR3 pose length != 7"
        )

    if len(expected) != 7:
        raise RehabMultiCycleContractError(
            "expected FR3 pose length != 7"
        )

    if not all(
        math.isfinite(v)
        for v in current + expected
    ):
        raise RehabMultiCycleContractError(
            "FR3 pose contains non-finite value"
        )

    return max(
        abs(a - b)
        for a, b in zip(
            current,
            expected,
        )
    )


def assert_cycle_start(
    current,
    q0,
    *,
    tolerance_rad=CYCLE_START_TOLERANCE_RAD,
):
    error = pose_error(
        current,
        q0,
    )

    if error > float(tolerance_rad):
        raise RehabMultiCycleContractError(
            "cycle start differs from "
            f"original run q0 by {error:.6f} rad"
        )

    return error


def build_multicycle_plan(
    q_start,
    *,
    cycles,
    hold_s,
):
    cycles, hold_s = (
        validate_multicycle_parameters(
            cycles,
            hold_s,
        )
    )

    q0, q_out, q_return = (
        build_cycle_targets(
            q_start
        )
    )

    stages = []

    for index in range(
        1,
        cycles + 1,
    ):
        stages.append(
            {
                "cycle": index,
                "expected_start": q0,
                "outbound_target": q_out,
                "return_target": q_return,
                "hold_s": hold_s,
            }
        )

    return {
        "cycles": cycles,
        "hold_s": hold_s,
        "q0": q0,
        "q_out": q_out,
        "q_return": q_return,
        "stages": tuple(stages),
    }
