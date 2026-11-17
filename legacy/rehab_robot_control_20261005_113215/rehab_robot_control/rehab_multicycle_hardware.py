#!/usr/bin/env python3

from __future__ import annotations

import argparse
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
    wait_fr3_live,
    xhand_baseline,
)

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)

from rehab_robot_control.rehab_microcycle_executor import (
    FR3_VELOCITY_RAD_S,
    validate_cycle_contract,
)

from rehab_robot_control.rehab_microcycle_ptp import (
    execute_ptp_validated,
)

from rehab_robot_control.rehab_multicycle_contract import (
    CYCLE_START_TOLERANCE_RAD,
    assert_cycle_start,
    build_multicycle_plan,
    validate_multicycle_parameters,
)

from rehab_robot_control.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)


PTP_ACTION = "/action_server/ptp_motion"

REAL_VALIDATION_CYCLES = 3
REAL_VALIDATION_HOLD_S = 0.5

TAKEOVER_SECONDS = 0.8
GRASP_SECONDS = 1.0
FINAL_HOLD_SECONDS = 0.5


class RehabMultiCycleHardwareError(RuntimeError):
    pass


def validate_real_request(
    cycles,
    hold_s,
):
    cycles, hold_s = (
        validate_multicycle_parameters(
            cycles,
            hold_s,
        )
    )

    # V1.2 first hardware validation is intentionally locked
    # to exactly 3 cycles.
    if cycles != REAL_VALIDATION_CYCLES:
        raise RehabMultiCycleHardwareError(
            "V1.2 real execution currently permits "
            "exactly 3 cycles"
        )

    if abs(
        float(hold_s)
        - REAL_VALIDATION_HOLD_S
    ) > 1e-12:
        raise RehabMultiCycleHardwareError(
            "V1.2 real validation hold must be "
            "exactly 0.5 s"
        )

    return cycles, hold_s


def run_cycle_sequence(
    plan,
    run_one_cycle,
):
    """
    Pure sequencing helper.

    If run_one_cycle raises in cycle N, the exception propagates
    immediately and cycle N+1 is never called.
    """

    completed = 0

    for stage in plan["stages"]:
        run_one_cycle(stage)
        completed += 1

    return completed


def software_check(
    cycles,
    hold_s,
):
    cycles, hold_s = validate_real_request(
        cycles,
        hold_s,
    )

    profile, sequence, micro = (
        load_configs()
    )

    active = validate_cycle_contract(
        profile,
        sequence,
        micro,
    )

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

    print("=" * 60)
    print("FR3 + XHand MULTI-CYCLE V1.2 HARDWARE CONTRACT")
    print("SOFTWARE CHECK ONLY / NO HARDWARE")
    print("=" * 60)

    print(
        f"real validation cycles = {cycles}"
    )
    print(
        f"hold time              = {hold_s:.3f} s"
    )
    print(
        f"active XHand joints    = {len(active)}"
    )
    print(
        "XHand                  = "
        "0.005 rad / 150 mA / 20 Hz / watchdog 0.5 s"
    )
    print(
        "FR3                    = "
        "live q0 -> q0+0.005 -> same q0"
    )
    print(
        "FR3 velocity           = "
        f"{FR3_VELOCITY_RAD_S:.3f} rad/s"
    )
    print(
        "cycle-start tolerance  = "
        f"{CYCLE_START_TOLERANCE_RAD:.3f} rad"
    )

    for stage in plan["stages"]:
        print(
            f"cycle {stage['cycle']}: "
            "same q0 -> q_out -> same q0"
        )

    print()
    print("Fault behavior:")
    print("  cancel accepted FR3 action")
    print("  XHand powerless")
    print("  stop current run immediately")
    print("  DO NOT start next cycle")
    print("  DO NOT auto-return FR3 after fault")
    print("  DO NOT auto-recover FR3")

    print()
    print(
        "[PASS] V1.2 REAL-EXECUTION SOFTWARE CONTRACT"
    )

    return 0


def execute_real(
    cycles,
    hold_s,
):
    if os.geteuid() != 0:
        raise RehabMultiCycleHardwareError(
            "real multi-cycle must run through sudo wrapper"
        )

    cycles, hold_s = validate_real_request(
        cycles,
        hold_s,
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
        print("=" * 60)
        print("FR3 + XHand REHAB MULTI-CYCLE V1.2")
        print("3 CYCLES / EMPTY LOAD / BOUNDED MOTION")
        print("=" * 60)

        # ------------------------------------------------------
        # Capture ONE q0 for the complete run.
        # It is never silently redefined between cycles.
        # ------------------------------------------------------

        q_start = wait_fr3_live(
            node
        )

        plan = build_multicycle_plan(
            q_start,
            cycles=cycles,
            hold_s=hold_s,
        )

        q0 = plan["q0"]
        q_out = plan["q_out"]

        print(
            "FR3 run q0 =",
            [
                round(v, 6)
                for v in q0
            ],
        )

        print(
            "FR3 q_out  =",
            [
                round(v, 6)
                for v in q_out
            ],
        )

        print(
            "[PLAN] q0 captured once; "
            "all 3 cycles reuse this same q0"
        )

        client = FR3PTPClient(
            node,
            PTP_ACTION,
        )

        if not client.wait_ready(
            timeout_sec=5.0
        ):
            raise RehabMultiCycleHardwareError(
                "FR3 PTP server unavailable"
            )

        # ------------------------------------------------------
        # XHand open.
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
            raise RehabMultiCycleHardwareError(
                "XHand serial mismatch"
            )

        driver.disable_multiframe()
        time.sleep(0.15)

        # ------------------------------------------------------
        # Passive baseline.
        # ------------------------------------------------------

        x0 = xhand_baseline(
            driver,
            require_mode0=True,
        )

        print(
            "[PASS] XHand passive baseline"
        )

        # ------------------------------------------------------
        # Validated takeover.
        # ------------------------------------------------------

        takeover_cmd = build_micro_command(
            x0,
            active_joints=active,
            targets=list(x0),
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
            label="multicycle takeover",
        )

        drift_limit = float(
            micro[
                "safety_guard"
            ][
                "max_takeover_drift_rad"
            ]
        )

        for name in active:
            peak = float(
                takeover[
                    "peak_abs"
                ][name]
            )

            if peak > drift_limit:
                raise RuntimeSafetyError(
                    f"{name} takeover drift exceeded: "
                    f"{peak:.6f} rad"
                )

        x1 = takeover[
            "last_positions"
        ]

        print(
            "[PASS] XHand takeover"
        )

        # ------------------------------------------------------
        # Arm the validated +0.005 grasp ONCE.
        # Keep this hold through all three cycles.
        # ------------------------------------------------------

        targets = list(x1)

        for name in active:
            index = (
                XHAND_JOINT_NAMES
                .index(name)
            )

            targets[index] += float(
                micro["delta_rad"]
            )

        grasp_cmd = build_micro_command(
            x1,
            active_joints=active,
            targets=targets,
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
            label="multicycle grasp",
        )

        print(
            "[PASS] XHand bounded grasp armed"
        )

        # ------------------------------------------------------
        # One normal cycle.
        # ------------------------------------------------------

        def run_one_cycle(stage):
            cycle = int(
                stage["cycle"]
            )

            print()
            print("=" * 60)
            print(
                f"CYCLE {cycle} / {cycles}"
            )
            print("=" * 60)

            # --------------------------------------------------
            # Before EVERY cycle, verify current FR3 pose against
            # the ORIGINAL q0 captured at run start.
            # --------------------------------------------------

            refresh_fr3_state(
                node,
                timeout_s=1.0,
                max_age_s=0.05,
            )

            q_current, _ = (
                ordered_fr3_state(
                    node
                )
            )

            start_error = (
                assert_cycle_start(
                    q_current,
                    q0,
                )
            )

            print(
                f"[PASS] cycle {cycle} start "
                f"q0_error={start_error:.6f} rad"
            )

            # --------------------------------------------------
            # Outbound q0 -> q0 + 0.005.
            # --------------------------------------------------

            node.microcycle_xhand_max_gap_s = 0.0

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
                label=(
                    f"cycle {cycle} outbound +0.005"
                ),
                expected_current=q0,
            )

            # --------------------------------------------------
            # Hold at q_out with validated 20 Hz XHand scheduler.
            # --------------------------------------------------

            if hold_s > 0.0:
                run_xhand_window(
                    node,
                    driver,
                    grasp_cmd,
                    reference=x1,
                    active_joints=active,
                    micro=micro,
                    seconds=hold_s,
                    label=(
                        f"cycle {cycle} outbound hold"
                    ),
                )

            # --------------------------------------------------
            # Confirm FR3 still near q_out before normal return.
            # --------------------------------------------------

            refresh_fr3_state(
                node,
                timeout_s=1.0,
                max_age_s=0.05,
            )

            q_hold, _ = (
                ordered_fr3_state(
                    node
                )
            )

            hold_error = fr3_pose_error(
                q_hold,
                q_out,
            )

            if hold_error > 0.010:
                raise RuntimeSafetyError(
                    f"cycle {cycle} outbound hold "
                    f"FR3 drift={hold_error:.6f} rad"
                )

            print(
                f"[PASS] cycle {cycle} outbound hold "
                f"pose_error={hold_error:.6f} rad"
            )

            # --------------------------------------------------
            # NORMAL planned return.
            #
            # This is reached only when every previous operation
            # in this cycle has succeeded.
            # --------------------------------------------------

            node.microcycle_xhand_max_gap_s = 0.0

            execute_ptp_validated(
                node,
                client,
                q0,
                driver=driver,
                hold_command=grasp_cmd,
                xhand_reference=x1,
                active_joints=active,
                micro=micro,
                velocity=FR3_VELOCITY_RAD_S,
                label=(
                    f"cycle {cycle} planned return"
                ),
                expected_current=q_out,
            )

            # --------------------------------------------------
            # Explicit cycle-end q0 verification.
            # --------------------------------------------------

            refresh_fr3_state(
                node,
                timeout_s=1.0,
                max_age_s=0.05,
            )

            q_returned, _ = (
                ordered_fr3_state(
                    node
                )
            )

            return_error = (
                assert_cycle_start(
                    q_returned,
                    q0,
                )
            )

            print(
                f"[PASS] cycle {cycle} complete "
                f"q0_error={return_error:.6f} rad"
            )

        completed = run_cycle_sequence(
            plan,
            run_one_cycle,
        )

        if completed != cycles:
            raise RuntimeSafetyError(
                "completed cycle count mismatch"
            )

        # ------------------------------------------------------
        # Normal final hold and disable.
        # ------------------------------------------------------

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=x1,
            active_joints=active,
            micro=micro,
            seconds=FINAL_HOLD_SECONDS,
            label="multicycle final hold",
        )

        driver.disable_multiframe()

        refresh_fr3_state(
            node,
            timeout_s=1.0,
            max_age_s=0.05,
        )

        q_final, _ = (
            ordered_fr3_state(
                node
            )
        )

        final_error = assert_cycle_start(
            q_final,
            q0,
        )

        print()
        print("=" * 60)
        print(
            "[PASS] REHAB MULTI-CYCLE V1.2 COMPLETE"
        )
        print("=" * 60)

        print(
            f"cycles completed = {completed}"
        )

        print(
            f"final q0 error   = "
            f"{final_error:.6f} rad"
        )

        print(
            "XHand             = powerless"
        )

        print(
            "FR3               = normal planned "
            "return to original run q0"
        )

        print(
            "No automatic fault recovery/return was used."
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

        # IMPORTANT:
        # No return-to-q0 command is issued here.
        # The exception terminates the cycle sequence immediately.
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

    group = (
        parser.add_mutually_exclusive_group(
            required=True
        )
    )

    group.add_argument(
        "--check",
        action="store_true",
    )

    group.add_argument(
        "--execute",
        action="store_true",
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=REAL_VALIDATION_CYCLES,
    )

    parser.add_argument(
        "--hold",
        type=float,
        default=REAL_VALIDATION_HOLD_S,
    )

    ns = parser.parse_args(args)

    if ns.check:
        return software_check(
            ns.cycles,
            ns.hold,
        )

    return execute_real(
        ns.cycles,
        ns.hold,
    )


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
            "[SAFE] no automatic FR3 "
            "recovery/return"
        )

        raise SystemExit(130)

    except Exception as exc:
        print(
            "[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] no automatic FR3 "
            "recovery/return"
        )

        raise SystemExit(2)
