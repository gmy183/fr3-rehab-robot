#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import statistics
import sys
import time

from xhand_controller.xhand_control import HandCommand_t

from rehab_robot_control.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)
from rehab_robot_control.xhand_state_mapper import JOINT_NAMES


MODE = 3
KP = 225
KI = 0
KD = 0

DELTA = 0.005
TOR_MAX = 150

WATCHDOG = 0.5
DISABLE_FRAMES = 3
REFRESH_HZ = 20.0

SINGLE_INDEX = 4

ACTIVE = [
    0,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
]

INACTIVE = [
    1,
    2,
    3,
]

MAX_TAKEOVER_DRIFT = 0.015
MAX_MOTION = 0.020
MAX_CURRENT = 170.0


class ValidationError(RuntimeError):
    pass


def read_state(driver):
    fresh = driver.read_fresh_state(False)

    if fresh is None:
        raise ValidationError("no fresh state")

    fingers = fresh.state.finger_state

    if len(fingers) != 12:
        raise ValidationError(
            f"finger count={len(fingers)}"
        )

    pos = []
    cur = []
    mode = []

    for i, f in enumerate(fingers):
        q = float(f.position)
        c = float(f.torque)
        raw = int(f.raw_position)

        if not math.isfinite(q):
            raise ValidationError(
                f"{JOINT_NAMES[i]} position non-finite"
            )

        if not math.isfinite(c):
            raise ValidationError(
                f"{JOINT_NAMES[i]} current non-finite"
            )

        errors = (
            int(f.commboard_err),
            int(f.jonitboard_err),
            int(f.tipboard_err),
        )

        if any(errors):
            raise ValidationError(
                f"{JOINT_NAMES[i]} errors={errors}"
            )

        pos.append(q)
        cur.append(c)
        mode.append(raw & 0xF)

    return pos, cur, mode


def baseline(driver, n=25):
    rows = []
    currents = []

    for _ in range(n):
        p, c, _ = read_state(driver)

        rows.append(p)
        currents.append(c)

        time.sleep(0.01)

    pos = [
        statistics.median(
            row[i]
            for row in rows
        )
        for i in range(12)
    ]

    cur = [
        statistics.median(
            row[i]
            for row in currents
        )
        for i in range(12)
    ]

    return pos, cur


def command(
    reference,
    active,
    targets,
):
    cmd = HandCommand_t()

    for i in range(12):
        f = cmd.finger_command[i]

        f.id = i
        f.position = float(reference[i])

        f.mode = 0
        f.kp = 0
        f.ki = 0
        f.kd = 0
        f.tor_max = 0

    for i in active:
        f = cmd.finger_command[i]

        f.position = float(targets[i])

        f.mode = MODE
        f.kp = KP
        f.ki = KI
        f.kd = KD
        f.tor_max = TOR_MAX

    return cmd


def run_window(
    driver,
    cmd,
    reference,
    active,
    seconds,
    label,
):
    samples = {
        i: []
        for i in active
    }

    max_current = {
        i: 0.0
        for i in active
    }

    modes = {
        i: set()
        for i in range(12)
    }

    deadline = time.monotonic() + seconds
    period = 1.0 / REFRESH_HZ

    while time.monotonic() < deadline:
        start = time.monotonic()

        driver.refresh_active_command(cmd)

        time.sleep(0.02)

        pos, cur, state_modes = read_state(
            driver
        )

        for i in range(12):
            modes[i].add(state_modes[i])

        for i in active:
            delta = pos[i] - reference[i]

            samples[i].append(delta)

            max_current[i] = max(
                max_current[i],
                abs(cur[i]),
            )

            if abs(delta) > MAX_MOTION:
                raise ValidationError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} runaway "
                    f"{delta:+.6f}"
                )

            if abs(cur[i]) > MAX_CURRENT:
                raise ValidationError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} current "
                    f"{cur[i]:.1f}mA"
                )

        driver.poll_watchdog()

        elapsed = time.monotonic() - start

        if elapsed < period:
            time.sleep(period - elapsed)

    result = {}

    for i in active:
        values = samples[i]

        if not values:
            raise ValidationError(
                f"{JOINT_NAMES[i]} no samples"
            )

        result[i] = {
            "median": statistics.median(values),
            "min": min(values),
            "max": max(values),
            "peak_abs": max(abs(v) for v in values),
            "imax": max_current[i],
        }

    return result, modes


def check_inactive_modes(modes):
    for i in INACTIVE:
        seen = modes[i]

        bad = [
            m
            for m in seen
            if m != 0
        ]

        if bad:
            raise ValidationError(
                f"inactive {JOINT_NAMES[i]} "
                f"entered modes={sorted(seen)}"
            )


def main():
    if os.geteuid() != 0:
        raise ValidationError(
            "must run via sudo wrapper"
        )

    interface = os.environ["XHAND_IF"]
    expected_serial = os.environ.get(
        "XHAND_SERIAL",
        "",
    ).strip()

    print(
        "============================================================"
    )
    print("XHand Phase-1 FINAL validation")
    print(
        "============================================================"
    )
    print("mode=3")
    print("kp=225")
    print("ki=0")
    print("kd=0")
    print("tor_max=150mA")
    print("delta=+0.005rad")
    print()
    print(
        "注意：本验收检查安全接管/隔离/有界微动，"
    )
    print(
        "不再用单次编码器量化步作为9关节伺服性能判据。"
    )

    backend = VendorXHandBackend()

    driver = XHandDriverCore(
        backend,
        command_timeout_s=WATCHDOG,
        disable_frames=DISABLE_FRAMES,
    )

    opened = False

    try:
        device_id = driver.open(interface)
        opened = True

        print()
        print(
            f"[PASS] EtherCAT open device_id={device_id}"
        )

        control = backend._control

        serial_ret = control.get_serial_number(
            device_id
        )

        actual_serial = (
            str(serial_ret[1])
            .strip()
            .strip("\x00")
        )

        print(f"serial={actual_serial}")

        if (
            expected_serial
            and actual_serial != expected_serial
        ):
            raise ValidationError(
                "serial mismatch"
            )

        # ====================================================
        # A. index_joint1 单关节
        # ====================================================

        print()
        print(
            "============================================================"
        )
        print("A. index_joint1 single-joint")
        print(
            "============================================================"
        )

        driver.disable_multiframe()
        time.sleep(0.15)

        p0, c0 = baseline(driver)

        print(
            f"passive q={p0[SINGLE_INDEX]:+.6f} "
            f"I={c0[SINGLE_INDEX]:+.1f}mA"
        )

        hold_targets = list(p0)

        hold = command(
            p0,
            [SINGLE_INDEX],
            hold_targets,
        )

        driver.send_command(hold)

        hold_result, hold_modes = run_window(
            driver,
            hold,
            p0,
            [SINGLE_INDEX],
            0.8,
            "single/takeover",
        )

        check_inactive_modes(hold_modes)

        if 3 not in hold_modes[SINGLE_INDEX]:
            raise ValidationError(
                "index_joint1 did not enter mode=3"
            )

        h = hold_result[SINGLE_INDEX]

        print(
            "takeover "
            f"median={h['median']:+.6f} "
            f"range=[{h['min']:+.6f},"
            f"{h['max']:+.6f}] "
            f"Imax={h['imax']:.1f}"
        )

        if h["peak_abs"] > MAX_TAKEOVER_DRIFT:
            raise ValidationError(
                "single takeover drift too large"
            )

        pa, _ = baseline(
            driver,
            n=15,
        )

        targets = list(pa)
        targets[SINGLE_INDEX] += DELTA

        move = command(
            pa,
            [SINGLE_INDEX],
            targets,
        )

        driver.send_command(move)

        move_result, move_modes = run_window(
            driver,
            move,
            pa,
            [SINGLE_INDEX],
            1.2,
            "single/move",
        )

        check_inactive_modes(move_modes)

        if 3 not in move_modes[SINGLE_INDEX]:
            raise ValidationError(
                "index_joint1 lost mode=3"
            )

        m = move_result[SINGLE_INDEX]

        print(
            "move "
            f"median={m['median']:+.6f} "
            f"range=[{m['min']:+.6f},"
            f"{m['max']:+.6f}] "
            f"Imax={m['imax']:.1f}"
        )

        # 方向已由上一轮肉眼验证。
        # 此处只要求没有反向失控。
        if m["min"] < -0.010:
            raise ValidationError(
                "index_joint1 unexpected reverse motion"
            )

        driver.disable_multiframe()

        print(
            "[PASS] index_joint1 control/safety contract"
        )

        # ====================================================
        # B. 9关节整手
        # ====================================================

        print()
        print(
            "============================================================"
        )
        print("B. 9-joint whole-hand micro validation")
        print(
            "============================================================"
        )

        time.sleep(0.15)

        p0, _ = baseline(driver)

        hold_targets = list(p0)

        whole_hold = command(
            p0,
            ACTIVE,
            hold_targets,
        )

        driver.send_command(
            whole_hold
        )

        takeover, takeover_modes = run_window(
            driver,
            whole_hold,
            p0,
            ACTIVE,
            1.0,
            "whole/takeover",
        )

        check_inactive_modes(
            takeover_modes
        )

        for i in ACTIVE:
            if 3 not in takeover_modes[i]:
                raise ValidationError(
                    f"{JOINT_NAMES[i]} "
                    "did not enter mode=3"
                )

        print()
        print("takeover:")

        for i in ACTIVE:
            r = takeover[i]

            print(
                f"{JOINT_NAMES[i]:18s} "
                f"range=[{r['min']:+.6f},"
                f"{r['max']:+.6f}] "
                f"Imax={r['imax']:.1f}"
            )

            if r["peak_abs"] > MAX_TAKEOVER_DRIFT:
                raise ValidationError(
                    f"{JOINT_NAMES[i]} "
                    "takeover drift too large"
                )

        pa, _ = baseline(
            driver,
            n=15,
        )

        targets = list(pa)

        for i in ACTIVE:
            targets[i] += DELTA

        whole_move = command(
            pa,
            ACTIVE,
            targets,
        )

        driver.send_command(
            whole_move
        )

        motion, motion_modes = run_window(
            driver,
            whole_move,
            pa,
            ACTIVE,
            1.5,
            "whole/move",
        )

        check_inactive_modes(
            motion_modes
        )

        print()
        print("+0.005 group command:")

        for i in ACTIVE:
            if 3 not in motion_modes[i]:
                raise ValidationError(
                    f"{JOINT_NAMES[i]} "
                    "lost mode=3"
                )

            r = motion[i]

            print(
                f"{JOINT_NAMES[i]:18s} "
                f"median={r['median']:+.6f} "
                f"range=[{r['min']:+.6f},"
                f"{r['max']:+.6f}] "
                f"Imax={r['imax']:.1f}"
            )

        driver.disable_multiframe()

        print()
        print(
            "============================================================"
        )
        print(
            "[PASS] XHand PHASE-1 FINAL MICRO VALIDATION"
        )
        print(
            "============================================================"
        )
        print(
            "index_joint1 direction was already "
            "visually confirmed as closing."
        )
        print(
            "Production active_gains is NOT modified by this script."
        )

        return 0

    finally:
        if opened:
            print()
            print(
                "[SAFE] powerless x3 + close EtherCAT"
            )

            driver.close()

            print(
                "[PASS] XHand powerless + EtherCAT closed"
            )


if __name__ == "__main__":
    try:
        raise SystemExit(main())

    except KeyboardInterrupt:
        print(
            "\n[FAIL] interrupted",
            file=sys.stderr,
        )
        raise SystemExit(130)

    except Exception as exc:
        print(
            "\n[FAIL] "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(2)
