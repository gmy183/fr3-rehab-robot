#!/usr/bin/env python3

from __future__ import annotations

import argparse

from rehab_robot_control.empty_grasp_executor import (
    load_configs,
)

from rehab_robot_control.rehab_microcycle_executor import (
    FR3_DELTA_RAD,
    FR3_VELOCITY_RAD_S,
    validate_cycle_contract,
)

from rehab_robot_control.rehab_multicycle_contract import (
    ALLOWED_CYCLES,
    DEFAULT_HOLD_S,
    CYCLE_START_TOLERANCE_RAD,
    build_multicycle_plan,
)


def print_vector(
    label,
    values,
):
    print(
        label,
        [
            round(float(v), 6)
            for v in values
        ],
    )


def software_check(
    *,
    cycles,
    hold_s,
):
    profile, sequence, micro = (
        load_configs()
    )

    active = validate_cycle_contract(
        profile,
        sequence,
        micro,
    )

    # Synthetic software-only pose.
    q_sample = (
        0.04,
        -0.65,
        0.00,
        -2.54,
        -0.02,
        1.56,
        1.58,
    )

    plan = build_multicycle_plan(
        q_sample,
        cycles=cycles,
        hold_s=hold_s,
    )

    print(
        "============================================================"
    )
    print(
        "FR3 + XHand REHAB MULTI-CYCLE V1.2"
    )
    print(
        "SOFTWARE CHECK ONLY / NO HARDWARE"
    )
    print(
        "============================================================"
    )

    print_vector(
        "synthetic run q0     =",
        plan["q0"],
    )

    print_vector(
        "synthetic q_out      =",
        plan["q_out"],
    )

    print_vector(
        "synthetic q_return   =",
        plan["q_return"],
    )

    print()
    print(
        f"cycles       = {plan['cycles']}"
    )
    print(
        f"hold_s       = {plan['hold_s']:.3f}"
    )
    print(
        f"allowed      = {ALLOWED_CYCLES}"
    )

    print()
    print("FR3 contract:")
    print(
        f"  J1 delta = +{FR3_DELTA_RAD:.6f} rad"
    )
    print(
        f"  vmax = {FR3_VELOCITY_RAD_S:.3f} rad/s"
    )
    print(
        "  q0 is captured once per run"
    )
    print(
        "  every cycle must return to the same q0"
    )
    print(
        "  q0 is NOT silently redefined between cycles"
    )
    print(
        "  cycle-start tolerance = "
        f"{CYCLE_START_TOLERANCE_RAD:.3f} rad"
    )

    print()
    print("XHand contract:")
    print(
        f"  active joints = {list(active)}"
    )
    print(
        "  delta = 0.005 rad"
    )
    print(
        "  effort = 150 mA"
    )
    print(
        "  refresh = 20 Hz"
    )
    print(
        "  watchdog = 0.5 s"
    )

    print()
    print("planned stages:")

    for stage in plan["stages"]:

        index = stage["cycle"]

        print(
            f"  cycle {index}: "
            "q0 -> q0+0.005 -> q0 "
            f"| hold={stage['hold_s']:.3f}s"
        )

    print()
    print("fault semantics:")
    print(
        "  fault in cycle N -> stop immediately"
    )
    print(
        "  cycle N+1 is NOT started"
    )
    print(
        "  current FR3 action -> cancel"
    )
    print(
        "  XHand -> powerless"
    )
    print(
        "  NO automatic FR3 recovery"
    )
    print(
        "  NO automatic FR3 return after fault"
    )

    print()
    print(
        "[PASS] REHAB MULTI-CYCLE V1.2 "
        "SOFTWARE CONTRACT"
    )

    return 0


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--check",
        action="store_true",
        required=True,
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
    )

    parser.add_argument(
        "--hold",
        type=float,
        default=DEFAULT_HOLD_S,
    )

    ns = parser.parse_args(args)

    return software_check(
        cycles=ns.cycles,
        hold_s=ns.hold,
    )


if __name__ == "__main__":
    try:
        raise SystemExit(
            main()
        )

    except Exception as exc:
        print(
            "[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] V1.2 software layer "
            "contains no hardware execution"
        )

        raise SystemExit(2)
