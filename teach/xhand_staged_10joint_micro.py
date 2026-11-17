import os
import time

from rehab_robot_xhand.xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)
from rehab_robot_xhand.xhand_sdk_adapter import (
    JointCommandSpec,
    build_command,
)

IFACE = "enx6c1ff7641594"

JOINT_NAMES = [
    "thumb_bend_joint",
    "thumb_rota_joint1",
    "thumb_rota_joint2",
    "index_bend_joint",
    "index_joint1",
    "index_joint2",
    "mid_joint1",
    "mid_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
]

# 已经逐个实机 PASS 的 9 个关节
ACTIVE_ORDER = [
    0,
    2,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
]

DELTA = 0.005

MODE = 3
KP = 225
KI = 0
KD = 0
TOR_MAX = 150

REFRESH_HZ = 20.0
PERIOD = 1.0 / REFRESH_HZ

WATCHDOG = 0.5
DISABLE_FRAMES = 3

CURRENT_GUARD = 170.0
TAKEOVER_DRIFT_GUARD = 0.015
TOTAL_MOTION_GUARD = 0.020
FINAL_ERROR_GUARD = 0.015


if os.environ.get("XHAND_EXECUTE_CONFIRM") != \
        "EXECUTE-XHAND-STAGED-MICRO":
    raise SystemExit(
        "[ABORT] hardware confirmation missing"
    )


def read_state(driver):
    fresh = driver.read_fresh_state(
        force_update=True
    )

    if fresh is None:
        raise RuntimeError(
            "no fresh XHand state"
        )

    fingers = fresh.state.finger_state

    if len(fingers) != 12:
        raise RuntimeError(
            f"joint count={len(fingers)}, expected 12"
        )

    q = []
    current = []

    for i, finger in enumerate(fingers):

        errors = (
            int(finger.commboard_err),
            int(finger.jonitboard_err),
            int(finger.tipboard_err),
        )

        if any(errors):
            raise RuntimeError(
                f"{JOINT_NAMES[i]} "
                f"hardware errors={errors}"
            )

        q.append(
            float(finger.position)
        )

        current.append(
            float(finger.torque)
        )

    return q, current


def make_command(
    q0,
    enabled,
    delta,
):
    enabled = set(enabled)

    specs = []

    for i in range(12):

        if i in enabled:

            specs.append(
                JointCommandSpec(
                    position=float(
                        q0[i] + delta
                    ),
                    effort_limit=TOR_MAX,
                    mode=MODE,
                    kp=KP,
                    ki=KI,
                    kd=KD,
                )
            )

        else:

            specs.append(
                JointCommandSpec(
                    position=float(q0[i]),
                    effort_limit=0,
                    mode=0,
                    kp=0,
                    ki=0,
                    kd=0,
                )
            )

    return build_command(specs)


def run_window(
    driver,
    command,
    q0,
    enabled,
    duration,
    label,
    motion_guard,
):
    enabled = list(enabled)

    peak_i = {
        i: 0.0
        for i in enabled
    }

    peak_dq = {
        i: 0.0
        for i in enabled
    }

    deadline = (
        time.monotonic()
        + duration
    )

    samples = 0

    while time.monotonic() < deadline:

        cycle_start = time.monotonic()

        driver.refresh_active_command(
            command
        )

        time.sleep(0.020)

        q, current = read_state(
            driver
        )

        samples += 1

        for i in enabled:

            dq = q[i] - q0[i]
            cur = abs(current[i])

            peak_i[i] = max(
                peak_i[i],
                cur,
            )

            peak_dq[i] = max(
                peak_dq[i],
                abs(dq),
            )

            if cur > CURRENT_GUARD:
                raise RuntimeError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} "
                    f"current guard exceeded: "
                    f"{current[i]:+.1f} mA"
                )

            if abs(dq) > motion_guard:
                raise RuntimeError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} "
                    f"motion guard exceeded: "
                    f"{dq:+.6f} rad"
                )

        driver.poll_watchdog()

        elapsed = (
            time.monotonic()
            - cycle_start
        )

        if elapsed < PERIOD:
            time.sleep(
                PERIOD - elapsed
            )

    worst_i = max(
        peak_i.values(),
        default=0.0,
    )

    worst_dq = max(
        peak_dq.values(),
        default=0.0,
    )

    print(
        f"[PASS] {label}: "
        f"samples={samples} "
        f"peak_I={worst_i:.1f}mA "
        f"peak_dq={worst_dq:.6f}rad"
    )

    return peak_i, peak_dq


print("=" * 78)
print("XHAND STAGED 10-JOINT TAKEOVER + WHOLE-HAND MICRO MOTION")
print("=" * 78)
print("mode       = 3")
print("kp/ki/kd   = 225 / 0 / 0")
print("tor_max    = 150 mA")
print("delta      = +0.005 rad")
print("refresh    = 20 Hz")
print("watchdog   = 0.5 s")
print("current    <= 170 mA")
print("NO FR3 MOTION")
print("=" * 78)


backend = VendorXHandBackend()

driver = XHandDriverCore(
    backend,
    command_timeout_s=WATCHDOG,
    disable_frames=DISABLE_FRAMES,
)

opened = False

try:

    device_id = driver.open(
        IFACE
    )

    opened = True

    print(
        f"[PASS] device_id={device_id}"
    )

    q0, i0 = read_state(
        driver
    )

    print()
    print("========== LIVE BASELINE ==========")

    for i in range(12):

        print(
            f"{i:02d} "
            f"{JOINT_NAMES[i]:20s} "
            f"q={q0[i]:+.6f} "
            f"I={i0[i]:+.1f}mA"
        )

    print()
    print(
        "========== STAGED TAKEOVER =========="
    )

    enabled = []

    for stage, index in enumerate(
        ACTIVE_ORDER,
        start=1,
    ):

        enabled.append(index)

        print(
            f"[STAGE {stage}/10] "
            f"enable {JOINT_NAMES[index]}"
        )

        command = make_command(
            q0,
            enabled,
            0.0,
        )

        run_window(
            driver,
            command,
            q0,
            enabled,
            0.50,
            f"TAKEOVER-{stage}",
            TAKEOVER_DRIFT_GUARD,
        )


    print()
    print(
        "========== ALL-10 HOLD =========="
    )

    hold_command = make_command(
        q0,
        ACTIVE_ORDER,
        0.0,
    )

    run_window(
        driver,
        hold_command,
        q0,
        ACTIVE_ORDER,
        0.75,
        "ALL10-HOLD",
        TAKEOVER_DRIFT_GUARD,
    )


    print()
    print(
        "========== WHOLE-HAND +0.005 =========="
    )

    move_command = make_command(
        q0,
        ACTIVE_ORDER,
        DELTA,
    )

    move_peak_i, move_peak_dq = run_window(
        driver,
        move_command,
        q0,
        ACTIVE_ORDER,
        1.50,
        "WHOLE-MOVE",
        TOTAL_MOTION_GUARD,
    )


    print()
    print(
        "========== RETURN TO LIVE BASELINE =========="
    )

    return_command = make_command(
        q0,
        ACTIVE_ORDER,
        0.0,
    )

    run_window(
        driver,
        return_command,
        q0,
        ACTIVE_ORDER,
        1.50,
        "RETURN",
        TOTAL_MOTION_GUARD,
    )


    qf, ifinal = read_state(
        driver
    )

    print()
    print(
        "========== FINAL RESULTS =========="
    )

    worst_final = 0.0

    for i in ACTIVE_ORDER:

        final_error = (
            qf[i] - q0[i]
        )

        worst_final = max(
            worst_final,
            abs(final_error),
        )

        print(
            f"{JOINT_NAMES[i]:20s} "
            f"move_peak="
            f"{move_peak_dq[i]:.6f}rad "
            f"peak_I="
            f"{move_peak_i[i]:.1f}mA "
            f"final_error="
            f"{final_error:+.6f}rad"
        )

    print()

    if worst_final > FINAL_ERROR_GUARD:
        raise RuntimeError(
            f"final error guard exceeded: "
            f"{worst_final:.6f} rad"
        )

    print(
        f"[PASS] worst_final_error="
        f"{worst_final:.6f} rad"
    )

    print()
    print(
        "[PASS] XHAND STAGED WHOLE-HAND "
        "MICRO VALIDATION COMPLETE"
    )


except KeyboardInterrupt:

    print()
    print(
        "[STOP] operator interrupt"
    )


except Exception as exc:

    print()
    print(
        f"[FAULT] "
        f"{type(exc).__name__}: {exc}"
    )


finally:

    if opened:

        print()
        print(
            "[SAFE] powerless x3 "
            "+ EtherCAT close"
        )

        try:

            driver.close()

            print(
                "[PASS] XHand safely closed"
            )

        except Exception as exc:

            print(
                f"[WARN] close: {exc}"
            )

print("=" * 78)
