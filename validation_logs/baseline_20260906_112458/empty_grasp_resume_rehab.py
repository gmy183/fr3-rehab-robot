#!/usr/bin/env python3

from __future__ import annotations

import os
import time

import rclpy

from rehab_robot_control.empty_grasp_executor import (
    CombinedNode,
    XHAND_JOINT_NAMES,
    build_micro_command,
    cancel_goal,
    execute_ptp,
    load_configs,
    read_xhand_sample,
    run_xhand_window,
    validate_micro_gate,
    wait_fr3_live,
    xhand_baseline,
)

from rehab_robot_control.fr3_ptp_client import (
    FR3PTPClient,
)

from rehab_robot_control.fr3_ptp_contract import (
    relative_target,
)

from rehab_robot_control.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)


PTP_ACTION = "/action_server/ptp_motion"


class ResumeError(RuntimeError):
    pass


def main():

    if os.geteuid() != 0:
        raise ResumeError(
            "must run through sudo wrapper"
        )

    profile, sequence, micro = (
        load_configs()
    )

    active = validate_micro_gate(
        profile,
        sequence,
        micro,
    )

    if float(micro["delta_rad"]) != 0.005:
        raise ResumeError(
            "unexpected XHand delta"
        )

    if int(micro["effort_limit_ma"]) != 150:
        raise ResumeError(
            "unexpected XHand current limit"
        )

    if float(micro["refresh_hz"]) != 20.0:
        raise ResumeError(
            "unexpected XHand refresh rate"
        )

    interface = os.environ["XHAND_IF"]

    expected_serial = (
        os.environ.get(
            "XHAND_SERIAL",
            "",
        ).strip()
    )

    rclpy.init()

    node = CombinedNode()

    driver = None

    try:
        print(
            "============================================================"
        )
        print(
            "RESUME ONLY: XHand hold + FR3 rehab +0.005"
        )
        print(
            "NO pregrasp / NO grasp +0.010 stage"
        )
        print(
            "============================================================"
        )

        # ====================================================
        # 1. FR3 当前实时姿态作为唯一新基线
        # ====================================================

        q_start = wait_fr3_live(
            node
        )

        q_target = relative_target(
            q_start,
            joint_index=0,
            delta_rad=0.005,
        )

        print(
            "FR3 resume q_start =",
            [
                round(v, 6)
                for v in q_start
            ],
        )

        print(
            "FR3 resume target  =",
            [
                round(v, 6)
                for v in q_target
            ],
        )

        print(
            "planned joint1 delta = +0.005000 rad"
        )

        client = FR3PTPClient(
            node,
            PTP_ACTION,
        )

        if not client.wait_ready(
            timeout_sec=5.0
        ):
            raise ResumeError(
                "PTP server unavailable"
            )

        # ====================================================
        # 2. 打开 XHand
        # ====================================================

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
            raise ResumeError(
                "XHand serial mismatch"
            )

        driver.disable_multiframe()

        time.sleep(0.15)

        # ====================================================
        # 3. passive baseline
        # ====================================================

        x0 = xhand_baseline(
            driver,
            require_mode0=True,
        )

        print(
            "[PASS] XHand passive baseline"
        )

        # ====================================================
        # 4. whole-hand takeover
        # ====================================================

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
            seconds=0.8,
            label="resume takeover",
        )

        x1 = takeover[
            "last_positions"
        ]

        print(
            "[PASS] XHand takeover"
        )

        # ====================================================
        # 5. +0.005 XHand bounded grasp
        # ====================================================

        targets = list(x1)

        for name in active:

            i = (
                XHAND_JOINT_NAMES
                .index(name)
            )

            targets[i] += 0.005

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
            seconds=1.0,
            label="resume grasp",
        )

        print(
            "[PASS] XHand bounded grasp armed"
        )

        # ====================================================
        # 6. 唯一 FR3 运动：
        # 当前真实位置 -> joint1 +0.005
        # XHand 使用已验证 20 Hz hold scheduler
        # ====================================================

        execute_ptp(
            node,
            client,
            q_target,
            driver=driver,
            hold_command=grasp_cmd,
            xhand_reference=x1,
            active_joints=active,
            micro=micro,
            velocity=0.020,
            label="FR3 RESUME rehab +0.005",
            expected_current=q_start,
        )

        # ====================================================
        # 7. 短时保持
        # ====================================================

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=x1,
            active_joints=active,
            micro=micro,
            seconds=0.5,
            label="resume final hold",
        )

        # ====================================================
        # 8. XHand powerless
        # FR3 保持最终位置，不自动回位
        # ====================================================

        driver.disable_multiframe()

        print()
        print(
            "============================================================"
        )
        print(
            "[PASS] RESUME REHAB STAGE COMPLETE"
        )
        print(
            "============================================================"
        )
        print(
            "FR3 moved only +0.005 rad from "
            "this run's live q_start."
        )
        print(
            "XHand is powerless."
        )
        print(
            "No automatic FR3 return/recovery."
        )

        return 0

    except KeyboardInterrupt:

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


if __name__ == "__main__":

    try:
        raise SystemExit(
            main()
        )

    except KeyboardInterrupt:

        print(
            "[FAIL] interrupted"
        )

        raise SystemExit(130)

    except Exception as exc:

        print(
            "[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] no automatic "
            "FR3 return/recovery"
        )

        raise SystemExit(2)
