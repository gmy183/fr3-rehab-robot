#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from pathlib import Path

import yaml

from xhand_controller.xhand_control import HandCommand_t

from rehab_robot_control.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)
from rehab_robot_control.xhand_state_mapper import (
    JOINT_NAMES,
    map_hand_state,
)


WS = Path.home() / "rehab_robot_ws"
PROFILE = WS / "config" / "xhand_motion_profile.yaml"

TARGET_NAME = "index_joint1"
TARGET_INDEX = 4

DELTA_RAD = 0.005

MODE = 3
KP = int(os.environ.get("XHAND_TEST_KP", "150"))
KI = 0
KD = 0

ALLOWED_TEST_KP = {150, 225}

TOR_MAX_MA = 150

WATCHDOG_S = 0.5
DISABLE_FRAMES = 3
REFRESH_HZ = 20.0

# 第一阶段重新验证保护阈值。
# 这些不是历史控制参数，不写入生产配置。
MAX_POSITION_ABS_RAD = 3.5
MAX_TAKEOVER_DRIFT_RAD = 0.015
MAX_RUNAWAY_DELTA_RAD = 0.030
MIN_ACCEPTED_DELTA_RAD = 0.001
MAX_ACCEPTED_DELTA_RAD = 0.012
MAX_OBSERVED_CURRENT_MA = 170.0


class ValidationError(RuntimeError):
    pass


def make_command(base_positions, active_position):
    if len(base_positions) != 12:
        raise ValidationError(
            f"expected 12 positions, got {len(base_positions)}"
        )

    command = HandCommand_t()

    for index in range(12):
        finger = command.finger_command[index]

        finger.id = index
        finger.position = float(base_positions[index])

        finger.mode = 0
        finger.kp = 0
        finger.ki = 0
        finger.kd = 0
        finger.tor_max = 0

    active = command.finger_command[TARGET_INDEX]

    active.position = float(active_position)
    active.mode = MODE
    active.kp = KP
    active.ki = KI
    active.kd = KD
    active.tor_max = TOR_MAX_MA

    return command


def static_check():
    print(
        "============================================================"
    )
    print("XHand Phase-1 一键工具：静态门禁")
    print(
        "============================================================"
    )

    if len(JOINT_NAMES) != 12:
        raise ValidationError(
            f"joint count changed: {len(JOINT_NAMES)}"
        )

    if JOINT_NAMES[TARGET_INDEX] != TARGET_NAME:
        raise ValidationError(
            "joint mapping mismatch: "
            f"index {TARGET_INDEX}="
            f"{JOINT_NAMES[TARGET_INDEX]}"
        )

    print(
        f"[PASS] {TARGET_NAME} SDK index = "
        f"{TARGET_INDEX}"
    )

    data = yaml.safe_load(
        PROFILE.read_text(encoding="utf-8")
    )

    xhand = data["xhand"]

    gains = xhand["active_gains"]
    single = xhand["single_joint"]
    inactive = xhand["inactive_joint"]
    watchdog = xhand["watchdog"]

    if gains["verified"] is not False:
        raise ValidationError(
            "production active_gains must remain "
            "verified=false"
        )

    if any(
        gains[key] is not None
        for key in ("kp", "ki", "kd")
    ):
        raise ValidationError(
            "production active gains unexpectedly "
            "contain numeric values"
        )

    if single["joint"] != TARGET_NAME:
        raise ValidationError(
            "single-joint target changed"
        )

    if (
        abs(
            float(single["delta_rad"])
            - DELTA_RAD
        )
        > 1e-12
    ):
        raise ValidationError(
            "single-joint delta changed"
        )

    if (
        int(single["effort_limit_ma"])
        != TOR_MAX_MA
    ):
        raise ValidationError(
            "single-joint current limit changed"
        )

    for key in (
        "mode",
        "kp",
        "ki",
        "kd",
        "tor_max",
    ):
        if int(inactive[key]) != 0:
            raise ValidationError(
                "inactive joint is not powerless: "
                f"{key}={inactive[key]}"
            )

    if (
        abs(
            float(
                watchdog[
                    "command_timeout_s"
                ]
            )
            - WATCHDOG_S
        )
        > 1e-12
    ):
        raise ValidationError(
            "watchdog timeout changed"
        )

    if (
        int(watchdog["disable_frames"])
        != DISABLE_FRAMES
    ):
        raise ValidationError(
            "disable frame count changed"
        )

    # 在不打开硬件的条件下，
    # 验证 Python -> SDK command 的实际字段。
    command = make_command(
        [0.0] * 12,
        DELTA_RAD,
    )

    for index, finger in enumerate(
        command.finger_command
    ):
        actual = (
            int(finger.mode),
            int(finger.kp),
            int(finger.ki),
            int(finger.kd),
            int(finger.tor_max),
        )

        if index == TARGET_INDEX:
            expected = (
                MODE,
                KP,
                KI,
                KD,
                TOR_MAX_MA,
            )
        else:
            expected = (
                0,
                0,
                0,
                0,
                0,
            )

        if actual != expected:
            raise ValidationError(
                f"joint {index} command mismatch: "
                f"{actual} != {expected}"
            )

    print(
        "[PASS] production motion gate remains CLOSED"
    )
    print(
        "[PASS] single = 0.005 rad / 150 mA"
    )
    if KP not in ALLOWED_TEST_KP:
        raise ValidationError(
            f"unapproved test kp={KP}; "
            f"allowed={sorted(ALLOWED_TEST_KP)}"
        )

    print(
        "[PASS] candidate = "
        f"mode=3 kp={KP} ki=0 kd=0"
    )
    print(
        "[PASS] other 11 joints = fully powerless"
    )
    print(
        "[PASS] watchdog = 0.5 s / "
        "disable frames = 3"
    )
    print()
    print(
        "[PASS] STATIC CHECK COMPLETE"
    )
    print(
        "No EtherCAT opened."
    )
    print(
        "No send_command() called."
    )


def read_state(driver):
    fresh = driver.read_fresh_state(False)

    if fresh is None:
        raise ValidationError(
            "no fresh XHand state"
        )

    mapped = map_hand_state(
        fresh.state,
        hand_id=int(
            driver.device_id
            if driver.device_id is not None
            else 0
        ),
        hand_name="xhand",
        hand_type="R",
    )

    joints = mapped.joint_state

    if tuple(joints.name) != tuple(
        JOINT_NAMES
    ):
        raise ValidationError(
            "runtime joint order changed"
        )

    positions = [
        float(v)
        for v in joints.position
    ]

    currents = [
        float(v)
        for v in joints.effort
    ]

    errors = [
        int(v)
        for v in joints.error_code
    ]

    if (
        len(positions) != 12
        or len(currents) != 12
        or len(errors) != 12
    ):
        raise ValidationError(
            "runtime state length mismatch"
        )

    if any(errors):
        bad = [
            index
            for index, value
            in enumerate(errors)
            if value != 0
        ]

        raise ValidationError(
            "hardware error_code nonzero: "
            f"{bad}"
        )

    values = positions + currents

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValidationError(
            "non-finite XHand state"
        )

    if any(
        abs(value)
        > MAX_POSITION_ABS_RAD
        for value in positions
    ):
        raise ValidationError(
            "position exceeds radian "
            "sanity gate"
        )

    return positions, currents


def sample_baseline(
    driver,
    count=30,
):
    position_rows = []
    current_rows = []

    for _ in range(count):
        positions, currents = read_state(
            driver
        )

        position_rows.append(
            positions
        )

        current_rows.append(
            currents
        )

        time.sleep(0.01)

    positions = [
        statistics.median(
            row[index]
            for row in position_rows
        )
        for index in range(12)
    ]

    currents = [
        statistics.median(
            row[index]
            for row in current_rows
        )
        for index in range(12)
    ]

    return positions, currents


def active_window(
    driver,
    command,
    *,
    seconds,
    reference,
    label,
):
    samples = []
    max_current = 0.0

    period = 1.0 / REFRESH_HZ
    deadline = (
        time.monotonic()
        + seconds
    )

    while (
        time.monotonic()
        < deadline
    ):
        driver.refresh_active_command(
            command
        )

        time.sleep(0.02)

        positions, currents = read_state(
            driver
        )

        position = positions[
            TARGET_INDEX
        ]

        current = abs(
            currents[TARGET_INDEX]
        )

        samples.append(position)

        max_current = max(
            max_current,
            current,
        )

        displacement = (
            position
            - reference
        )

        if (
            abs(displacement)
            > MAX_RUNAWAY_DELTA_RAD
        ):
            raise ValidationError(
                f"{label}: runaway position "
                f"{displacement:+.6f} rad"
            )

        if (
            current
            > MAX_OBSERVED_CURRENT_MA
        ):
            raise ValidationError(
                f"{label}: current "
                f"{current:.1f} mA exceeds "
                f"{MAX_OBSERVED_CURRENT_MA:.1f}"
            )

        driver.poll_watchdog()

        remain = period - 0.02

        if remain > 0:
            time.sleep(remain)

    if len(samples) < 5:
        raise ValidationError(
            f"{label}: insufficient samples"
        )

    final_position = (
        statistics.median(
            samples[-5:]
        )
    )

    return (
        final_position,
        max_current,
    )


def verify_serial(
    backend,
    device_id,
    expected,
):
    control = getattr(
        backend,
        "_control",
        None,
    )

    if (
        control is None
        or not hasattr(
            control,
            "get_serial_number",
        )
    ):
        raise ValidationError(
            "get_serial_number unavailable"
        )

    result = (
        control.get_serial_number(
            device_id
        )
    )

    if (
        not isinstance(
            result,
            tuple,
        )
        or len(result) != 2
    ):
        raise ValidationError(
            "unexpected serial result"
        )

    error = result[0]

    if (
        hasattr(
            error,
            "error_code",
        )
        and int(
            error.error_code
        )
        != 0
    ):
        raise ValidationError(
            "serial read failed: "
            f"{error.error_code}"
        )

    actual = (
        str(result[1])
        .strip()
        .strip("\x00")
    )

    print(
        f"serial={actual}"
    )

    if (
        expected
        and actual != expected
    ):
        raise ValidationError(
            "serial mismatch: "
            f"expected={expected}, "
            f"actual={actual}"
        )


def single_test():
    static_check()

    if os.geteuid() != 0:
        raise ValidationError(
            "--single must run through "
            "the sudo wrapper"
        )

    interface = (
        os.environ
        .get(
            "XHAND_IF",
            "",
        )
        .strip()
    )

    expected_serial = (
        os.environ
        .get(
            "XHAND_SERIAL",
            "",
        )
        .strip()
    )

    if not interface:
        raise ValidationError(
            "XHAND_IF missing"
        )

    carrier = (
        Path(
            "/sys/class/net"
        )
        / interface
        / "carrier"
    )

    if not carrier.exists():
        raise ValidationError(
            f"interface missing: "
            f"{interface}"
        )

    if (
        carrier
        .read_text()
        .strip()
        != "1"
    ):
        raise ValidationError(
            f"{interface} carrier=0"
        )

    print()
    print(
        "============================================================"
    )
    print(
        "REAL XHand Phase-1 single-joint validation"
    )
    print(
        "index_joint1 / +0.005 rad / 150 mA"
    )
    print(
        "FR3 is NOT used."
    )
    print(
        "============================================================"
    )

    backend = VendorXHandBackend()

    driver = XHandDriverCore(
        backend,
        command_timeout_s=WATCHDOG_S,
        disable_frames=DISABLE_FRAMES,
    )

    opened = False

    try:
        device_id = driver.open(
            interface
        )

        opened = True

        print(
            "[PASS] EtherCAT open, "
            f"device_id={device_id}"
        )

        verify_serial(
            backend,
            device_id,
            expected_serial,
        )

        # 1. 被动基线
        passive_pos, passive_cur = (
            sample_baseline(
                driver
            )
        )

        q0 = passive_pos[
            TARGET_INDEX
        ]

        i0 = passive_cur[
            TARGET_INDEX
        ]

        print(
            "passive baseline: "
            f"q={q0:+.6f} rad, "
            f"I={i0:+.1f} mA"
        )

        if (
            abs(i0)
            > MAX_OBSERVED_CURRENT_MA
        ):
            raise ValidationError(
                "passive current already "
                "above validation gate"
            )

        # 2. 当前位姿接管
        hold = make_command(
            passive_pos,
            q0,
        )

        driver.send_command(
            hold
        )

        takeover_q, takeover_i = (
            active_window(
                driver,
                hold,
                seconds=0.8,
                reference=q0,
                label="takeover",
            )
        )

        drift = takeover_q - q0

        print(
            "takeover: "
            f"drift={drift:+.6f} rad, "
            f"max_I={takeover_i:.1f} mA"
        )

        if (
            abs(drift)
            > MAX_TAKEOVER_DRIFT_RAD
        ):
            raise ValidationError(
                "takeover drift too large"
            )

        # 3. 主动保持基线
        active_pos, active_cur = (
            sample_baseline(
                driver,
                count=20,
            )
        )

        active_q = active_pos[
            TARGET_INDEX
        ]

        print(
            "active baseline: "
            f"q={active_q:+.6f} rad, "
            f"I={active_cur[TARGET_INDEX]:+.1f} mA"
        )

        # 4. 真正 +0.005 rad
        target_q = (
            active_q
            + DELTA_RAD
        )

        move = make_command(
            active_pos,
            target_q,
        )

        driver.send_command(
            move
        )

        final_q, move_i = (
            active_window(
                driver,
                move,
                seconds=1.0,
                reference=active_q,
                label="move",
            )
        )

        observed = (
            final_q
            - active_q
        )

        print(
            "move: "
            f"target_delta={DELTA_RAD:+.6f}, "
            f"observed_delta={observed:+.6f} rad, "
            f"max_I={move_i:.1f} mA"
        )

        if (
            observed
            < MIN_ACCEPTED_DELTA_RAD
        ):
            raise ValidationError(
                "positive target did not "
                "produce positive motion"
            )

        if (
            observed
            > MAX_ACCEPTED_DELTA_RAD
        ):
            raise ValidationError(
                "observed motion too large"
            )

        # 5. 回到主动基线
        return_command = (
            make_command(
                active_pos,
                active_q,
            )
        )

        driver.send_command(
            return_command
        )

        returned_q, return_i = (
            active_window(
                driver,
                return_command,
                seconds=0.8,
                reference=active_q,
                label="return",
            )
        )

        return_error = (
            returned_q
            - active_q
        )

        print(
            "return: "
            f"error={return_error:+.6f} rad, "
            f"max_I={return_i:.1f} mA"
        )

        if (
            abs(return_error)
            > 0.008
        ):
            raise ValidationError(
                "return error too large"
            )

        print()
        print(
            "============================================================"
        )
        print(
            "[PASS] XHand single-joint candidate validation"
        )
        print(
            "============================================================"
        )
        print(
            "Production active_gains remains "
            "verified=false."
        )
        print(
            "请观察：刚才 index_joint1 的微小正向动作"
            "是否属于预期的屈曲/闭合方向。"
        )

        return 0

    finally:
        if opened:
            print()
            print(
                "[SAFE] 发送多帧 mode=0 并关闭 EtherCAT"
            )

            driver.close()

            print(
                "[PASS] XHand powerless + EtherCAT closed"
            )


def main():
    parser = argparse.ArgumentParser()

    group = (
        parser
        .add_mutually_exclusive_group(
            required=True
        )
    )

    group.add_argument(
        "--check",
        action="store_true",
    )

    group.add_argument(
        "--single",
        action="store_true",
    )

    args = parser.parse_args()

    try:
        if args.check:
            static_check()
            return 0

        return single_test()

    except KeyboardInterrupt:
        print(
            "\n[FAIL] interrupted",
            file=sys.stderr,
        )
        return 130

    except Exception as exc:
        print(
            "\n[FAIL] "
            f"{type(exc).__name__}: "
            f"{exc}",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
