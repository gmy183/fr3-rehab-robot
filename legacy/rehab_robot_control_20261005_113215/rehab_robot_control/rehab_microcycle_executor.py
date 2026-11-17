#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import os
import time

import rclpy

from rehab_robot_control.empty_grasp_executor import (
    CombinedNode,
    RuntimeSafetyError,
    XHAND_JOINT_NAMES,
    build_micro_command,
    cancel_goal,
    fr3_pose_error,
    load_configs,
    ordered_fr3_state,
    refresh_fr3_state,
    run_xhand_window,
    validate_micro_gate,
    wait_fr3_live,
    xhand_baseline,
)

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)

from rehab_robot_control.rehab_microcycle_ptp import (
    execute_ptp_validated,
)

from rehab_robot_control.fr3_ptp_contract import (
    relative_target,
    validate_target,
)

from rehab_robot_control.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)


PTP_ACTION = "/action_server/ptp_motion"

FR3_DELTA_RAD = 0.005
FR3_VELOCITY_RAD_S = 0.020

TAKEOVER_SECONDS = 0.8
GRASP_SECONDS = 1.0
TURNAROUND_HOLD_SECONDS = 0.5
FINAL_HOLD_SECONDS = 0.5


class RehabMicrocycleError(RuntimeError):
    pass


def _close(a, b, tol=1e-9):
    return math.isclose(
        float(a),
        float(b),
        rel_tol=0.0,
        abs_tol=float(tol),
    )


def validate_cycle_contract(
    profile,
    sequence,
    micro,
):
    """
    Reuse the already hardware-validated bounded-micro contract.

    This does NOT convert the candidate XHand gains into formally
    verified production gains.
    """

    active = validate_micro_gate(
        profile,
        sequence,
        micro,
    )

    if not _close(
        micro["delta_rad"],
        0.005,
    ):
        raise RehabMicrocycleError(
            "XHand delta must remain 0.005 rad"
        )

    if int(
        micro["effort_limit_ma"]
    ) != 150:
        raise RehabMicrocycleError(
            "XHand effort limit must remain 150 mA"
        )

    if not _close(
        micro["refresh_hz"],
        20.0,
    ):
        raise RehabMicrocycleError(
            "XHand refresh must remain 20 Hz"
        )

    if not _close(
        micro["watchdog_s"],
        0.5,
    ):
        raise RehabMicrocycleError(
            "XHand watchdog must remain 0.5 s"
        )

    safety = micro["safety_guard"]

    if float(
        safety["max_abs_current_ma"]
    ) > 170.0:
        raise RehabMicrocycleError(
            "XHand current guard exceeds 170 mA"
        )

    if float(
        safety["max_abs_joint_motion_rad"]
    ) > 0.020:
        raise RehabMicrocycleError(
            "XHand motion guard exceeds 0.020 rad"
        )

    if (
        sequence["fr3"][
            "automatic_recovery_on_fault"
        ]
        is not False
    ):
        raise RehabMicrocycleError(
            "automatic FR3 recovery must remain disabled"
        )

    if (
        sequence["fr3"][
            "automatic_return_on_fault"
        ]
        is not False
    ):
        raise RehabMicrocycleError(
            "automatic FR3 return-on-fault must remain disabled"
        )

    return tuple(active)


def build_cycle_targets(q_start):
    """
    Normal planned rehabilitation cycle:

        q0 -> q0 + 0.005 rad on J1 -> q0

    The second motion is part of the normal planned trajectory.
    It is NOT a fault-recovery action.
    """

    q0 = validate_target(
        tuple(float(v) for v in q_start)
    )

    q_out = relative_target(
        q0,
        joint_index=0,
        delta_rad=FR3_DELTA_RAD,
    )

    q_return = validate_target(q0)

    return q0, q_out, q_return


def print_vector(label, values):
    print(
        label,
        [
            round(float(v), 6)
            for v in values
        ],
    )


def software_check():
    profile, sequence, micro = (
        load_configs()
    )

    active = validate_cycle_contract(
        profile,
        sequence,
        micro,
    )

    # Clearly synthetic software-only pose.
    q_sample = (
        0.03,
        -0.65,
        0.00,
        -2.54,
        -0.02,
        1.56,
        1.58,
    )

    q0, q_out, q_return = (
        build_cycle_targets(q_sample)
    )

    if q_return != q0:
        raise RehabMicrocycleError(
            "planned return target differs from q0"
        )

    if not _close(
        q_out[0] - q0[0],
        FR3_DELTA_RAD,
    ):
        raise RehabMicrocycleError(
            "FR3 outbound delta is not +0.005 rad"
        )

    for i in range(1, 7):
        if not _close(
            q_out[i],
            q0[i],
        ):
            raise RehabMicrocycleError(
                f"unexpected motion planned on FR3 joint {i + 1}"
            )

    print(
        "============================================================"
    )
    print(
        "FR3 + XHand REHAB MICRO-CYCLE SOFTWARE CHECK"
    )
    print(
        "NO HARDWARE / NO MOTION"
    )
    print(
        "============================================================"
    )

    print_vector(
        "synthetic q0      =",
        q0,
    )
    print_vector(
        "synthetic q_out   =",
        q_out,
    )
    print_vector(
        "synthetic q_return=",
        q_return,
    )

    print()
    print(
        "FR3 plan:"
    )
    print(
        "  J1: q0 -> q0 + 0.005 -> q0"
    )
    print(
        "  vmax = 0.020 rad/s"
    )
    print(
        "  cycles = 1"
    )

    print()
    print(
        "XHand contract:"
    )
    print(
        "  active joints =",
        list(active),
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
    print(
        "  formal gain verification remains FALSE"
    )

    print()
    print(
        "Fault semantics:"
    )
    print(
        "  cancel accepted FR3 goal"
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
        "[PASS] REHAB MICRO-CYCLE SOFTWARE CONTRACT"
    )

    return 0


def execute_cycle():
    if os.geteuid() != 0:
        raise RehabMicrocycleError(
            "real micro-cycle must run through sudo wrapper"
        )

    profile, sequence, micro = (
        load_configs()
    )

    active = validate_cycle_contract(
        profile,
        sequence,
        micro,
    )

    interface = os.environ["XHAND_IF"]

    expected_serial = (
        os.environ.get(
            "XHAND_SERIAL",
            "",
        )
        .strip()
    )

    rclpy.init()

    node = CombinedNode()
    driver = None

    try:
        print(
            "============================================================"
        )
        print(
            "FR3 + XHand REHAB MICRO-CYCLE V1"
        )
        print(
            "ONE CYCLE / EMPTY LOAD / BOUNDED MOTION"
        )
        print(
            "============================================================"
        )

        # ------------------------------------------------------
        # Stage 1: Capture this run's live FR3 origin.
        # ------------------------------------------------------

        q_start = wait_fr3_live(
            node
        )

        q0, q_out, q_return = (
            build_cycle_targets(q_start)
        )

        print_vector(
            "FR3 q0       =",
            q0,
        )
        print_vector(
            "FR3 q_out    =",
            q_out,
        )
        print_vector(
            "FR3 q_return =",
            q_return,
        )

        print(
            "[PLAN] J1 outbound +0.005 rad, "
            "then planned return to this run's q0"
        )

        client = FR3PTPClient(
            node,
            PTP_ACTION,
        )

        if not client.wait_ready(
            timeout_sec=5.0
        ):
            raise RehabMicrocycleError(
                "FR3 PTP server unavailable"
            )

        # ------------------------------------------------------
        # Stage 2: Open XHand.
        # ------------------------------------------------------

        backend = VendorXHandBackend()

        driver = XHandDriverCore(
            backend,
            command_timeout_s=float(
                micro["watchdog_s"]
            ),
            disable_frames=int(
                micro["disable_frames"]
            ),
        )

        device_id = driver.open(
            interface
        )

        serial_ret = (
            backend._control
            .get_serial_number(
                device_id
            )
        )

        actual_serial = (
            str(serial_ret[1])
            .strip()
            .strip("\x00")
        )

        print(
            "XHand serial =",
            actual_serial,
        )

        if (
            expected_serial
            and actual_serial
            != expected_serial
        ):
            raise RehabMicrocycleError(
                "XHand serial mismatch"
            )

        driver.disable_multiframe()
        time.sleep(0.15)

        # ------------------------------------------------------
        # Stage 3: Passive baseline.
        # ------------------------------------------------------

        x0 = xhand_baseline(
            driver,
            require_mode0=True,
        )

        print(
            "[PASS] XHand passive baseline"
        )

        # ------------------------------------------------------
        # Stage 4: Whole-hand takeover at current position.
        # ------------------------------------------------------

        takeover_targets = list(x0)

        takeover_cmd = build_micro_command(
            x0,
            active_joints=active,
            targets=takeover_targets,
            micro=micro,
        )

        takeover = run_xhand_window(
            node,
            driver,
            takeover_cmd,
            reference=x0,
            active_joints=active,
            micro=micro,
            seconds=TAKEOVER_SECONDS,
            label="cycle takeover",
        )

        drift_limit = float(
            micro[
                "safety_guard"
            ][
                "max_takeover_drift_rad"
            ]
        )

        for name in active:
            if (
                takeover[
                    "peak_abs"
                ][name]
                > drift_limit
            ):
                raise RuntimeSafetyError(
                    f"{name} takeover drift exceeded"
                )

        x1 = takeover[
            "last_positions"
        ]

        print(
            "[PASS] XHand takeover"
        )

        # ------------------------------------------------------
        # Stage 5: Bounded XHand +0.005 grasp.
        # ------------------------------------------------------

        grasp_targets = list(x1)

        for name in active:
            index = (
                XHAND_JOINT_NAMES
                .index(name)
            )

            grasp_targets[index] += float(
                micro["delta_rad"]
            )

        grasp_cmd = build_micro_command(
            x1,
            active_joints=active,
            targets=grasp_targets,
            micro=micro,
        )

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=x1,
            active_joints=active,
            micro=micro,
            seconds=GRASP_SECONDS,
            label="cycle grasp",
        )

        print(
            "[PASS] XHand bounded grasp armed"
        )

        # ------------------------------------------------------
        # Stage 6: Normal outbound FR3 motion.
        # q0 -> q0 + 0.005 J1
        # ------------------------------------------------------

        execute_ptp_validated(
            node,
            client,
            q_out,
            driver=driver,
            hold_command=grasp_cmd,
            xhand_reference=x1,
            active_joints=active,
            micro=micro,
            velocity=FR3_VELOCITY_RAD_S,
            label="FR3 cycle outbound +0.005",
            expected_current=q0,
        )

        # ------------------------------------------------------
        # Stage 7: Short validated XHand hold before reversal.
        # ------------------------------------------------------

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=x1,
            active_joints=active,
            micro=micro,
            seconds=TURNAROUND_HOLD_SECONDS,
            label="cycle turnaround hold",
        )

        # ------------------------------------------------------
        # Stage 8: Planned NORMAL return.
        #
        # This is part of the requested healthy trajectory.
        # If any previous stage faults, execution jumps out and
        # this return goal is NOT sent.
        # ------------------------------------------------------

        execute_ptp_validated(
            node,
            client,
            q_return,
            driver=driver,
            hold_command=grasp_cmd,
            xhand_reference=x1,
            active_joints=active,
            micro=micro,
            velocity=FR3_VELOCITY_RAD_S,
            label="FR3 cycle planned return to q0",
            expected_current=q_out,
        )

        # ------------------------------------------------------
        # Stage 9: Explicit return verification.
        # ------------------------------------------------------

        refresh_fr3_state(
            node,
            timeout_s=1.0,
            max_age_s=0.05,
        )

        q_final, _ = ordered_fr3_state(
            node
        )

        return_error = fr3_pose_error(
            q_final,
            q0,
        )

        print(
            f"[INFO] FR3 return max error="
            f"{return_error:.6f} rad"
        )

        if return_error > 0.010:
            raise RuntimeSafetyError(
                "FR3 return-to-q0 verification failed: "
                f"{return_error:.6f} rad"
            )

        # ------------------------------------------------------
        # Stage 10: Final short hold, then XHand powerless.
        # ------------------------------------------------------

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=x1,
            active_joints=active,
            micro=micro,
            seconds=FINAL_HOLD_SECONDS,
            label="cycle final hold",
        )

        driver.disable_multiframe()

        print()
        print(
            "============================================================"
        )
        print(
            "[PASS] REHAB MICRO-CYCLE COMPLETE"
        )
        print(
            "============================================================"
        )
        print(
            "FR3: planned outbound +0.005 rad "
            "and normal return to captured q0 completed."
        )
        print(
            f"FR3 return max error="
            f"{return_error:.6f} rad"
        )
        print(
            "XHand: powerless."
        )
        print(
            "Cycles completed: 1"
        )
        print(
            "No automatic fault recovery was used."
        )

        return 0

    except KeyboardInterrupt:
        print(
            "\n[FAULT] operator interrupt"
        )

        if (
            node.active_goal_handle
            is not None
        ):
            cancel_goal(
                node,
                node.active_goal_handle,
            )

        raise

    except Exception:
        if (
            node.active_goal_handle
            is not None
        ):
            cancel_goal(
                node,
                node.active_goal_handle,
            )

        raise

    finally:
        if driver is not None:
            try:
                driver.close()

                print(
                    "[SAFE] XHand powerless x3 "
                    "+ EtherCAT closed"
                )

            except Exception as exc:
                print(
                    "[WARN] XHand close:",
                    exc,
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()

    group = parser.add_mutually_exclusive_group(
        required=True
    )

    group.add_argument(
        "--check",
        action="store_true",
        help="software-only contract check",
    )

    group.add_argument(
        "--execute",
        action="store_true",
        help="execute exactly one real bounded cycle",
    )

    ns = parser.parse_args(args)

    if ns.check:
        return software_check()

    return execute_cycle()


if __name__ == "__main__":
    try:
        raise SystemExit(
            main()
        )

    except KeyboardInterrupt:
        print(
            "[FAIL] interrupted"
        )

        print(
            "[SAFE] no automatic FR3 return/recovery"
        )

        raise SystemExit(130)

    except Exception as exc:
        print(
            "[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] no automatic FR3 return/recovery"
        )

        raise SystemExit(2)
