#!/usr/bin/env python3

import time
import rclpy

from rehab_robot_tasks.fr3_teach_path_executor import (
    P15Node,
    P15Error,
    OfficialTeachMoveItClient,
    require_active_controllers,
    wait_relevant_moveit,
    wait_fresh_joint_state,
    plan_joint_target,
    execute_joint_target,
)


DELTA_RAD = 0.12          # ≈ 6.9 deg
MAX_ALLOWED_DRIFT = 0.02  # rad
Q7_SOFT_ABS_LIMIT = 2.50  # conservative local gate


def main():

    rclpy.init()

    node = P15Node()
    client = OfficialTeachMoveItClient(node)

    try:
        print("=" * 72)
        print("FR3 HAND-EYE CALIBRATION JOG")
        print("official MoveIt only")
        print("joint7 relative motion = +0.12 rad (~6.9 deg)")
        print("NO retry / NO auto-return / NO ErrorRecovery")
        print("=" * 72)

        # ---------------------------------------------------------
        # Gate
        # ---------------------------------------------------------
        require_active_controllers(node)
        wait_relevant_moveit(node, client)

        q0 = wait_fresh_joint_state(node)

        print()
        print("========== LIVE START ==========")

        for i, q in enumerate(q0, 1):
            print(f"fr3_joint{i} = {q:+.6f} rad")

        q_target = list(q0)
        q_target[6] += DELTA_RAD

        if abs(q_target[6]) > Q7_SOFT_ABS_LIMIT:
            raise P15Error(
                f"joint7 target blocked: {q_target[6]:+.6f} rad"
            )

        print()
        print("========== TARGET ==========")
        print(f"joint7 start  = {q0[6]:+.6f} rad")
        print(f"joint7 target = {q_target[6]:+.6f} rad")
        print(f"delta         = {DELTA_RAD:+.6f} rad")
        print(f"delta         = {DELTA_RAD * 180.0 / 3.141592653589793:+.3f} deg")

        # ---------------------------------------------------------
        # Plan only first
        # ---------------------------------------------------------
        print()
        print("========== PLAN ==========")

        npoints = plan_joint_target(
            node,
            client,
            q_target,
        )

        print(f"[PASS] MoveIt plan")
        print(f"trajectory_points = {npoints}")

        print()
        print("=" * 72)
        print("REAL FR3 CALIBRATION MOTION")
        print("Required:")
        print("  workspace clear")
        print("  camera / XHand clearance visually checked")
        print("  E-stop reachable")
        print("  checkerboard remains fixed")
        print("NO retry / NO automatic return / NO ErrorRecovery")
        print("=" * 72)

        token = "EXECUTE-CAL-J7-P012"
        answer = input(
            f"Type exactly {token} : "
        ).strip()

        if answer != token:
            raise P15Error("confirmation mismatch")

        # ---------------------------------------------------------
        # Re-read live state before execution
        # ---------------------------------------------------------
        q_live = wait_fresh_joint_state(node)

        drift = max(
            abs(a - b)
            for a, b in zip(q_live, q0)
        )

        print()
        print(f"pre-execution max drift = {drift:.6f} rad")

        if drift > MAX_ALLOWED_DRIFT:
            raise P15Error(
                f"robot moved after planning: drift={drift:.6f} rad"
            )

        require_active_controllers(node)

        print()
        print("========== EXECUTE ==========")

        wrapped = execute_joint_target(
            node,
            client,
            q_target,
        )

        print(
            f"action_status = {wrapped.status}"
        )
        print(
            f"moveit_error_code = "
            f"{int(wrapped.result.error_code.val)}"
        )

        # ---------------------------------------------------------
        # Settle + verify
        # ---------------------------------------------------------
        settle_until = time.monotonic() + 1.5

        while (
            rclpy.ok()
            and time.monotonic() < settle_until
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        qf = wait_fresh_joint_state(node)

        actual_delta = qf[6] - q0[6]
        final_error = max(
            abs(a - b)
            for a, b in zip(qf, q_target)
        )

        print()
        print("========== FINAL ==========")

        for i, q in enumerate(qf, 1):
            print(f"fr3_joint{i} = {q:+.6f} rad")

        print()
        print(
            f"actual joint7 delta = "
            f"{actual_delta:+.6f} rad"
        )
        print(
            f"actual joint7 delta = "
            f"{actual_delta * 180.0 / 3.141592653589793:+.3f} deg"
        )
        print(
            f"max final joint error = "
            f"{final_error:.6f} rad"
        )

        if abs(actual_delta) < 0.06:
            raise P15Error(
                "actual calibration rotation too small"
            )

        print()
        print("=" * 72)
        print("[PASS] FR3 CALIBRATION JOG COMPLETE")
        print("[SAFE] robot left at calibration pose")
        print("[SAFE] no automatic return")
        print("=" * 72)

    except KeyboardInterrupt:
        print()
        print("[ABORT] operator interrupt")
        print("[SAFE] no retry / no auto-return")
        raise SystemExit(130)

    except Exception as exc:
        print()
        print(
            f"[FAULT] {type(exc).__name__}: {exc}"
        )
        print("[SAFE] motion stopped")
        print("[SAFE] no retry")
        print("[SAFE] no automatic return")
        print("[SAFE] no ErrorRecovery")
        raise SystemExit(2)

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
