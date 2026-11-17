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

TEST_JOINTS = [1, 2]

LIMITS = {
    1: (-1.05, 1.57),
    2: (0.00, 1.57),
}

MODE = 3
KP = 225
KI = 0
KD = 0
TOR_MAX = 150

DELTA = 0.010

CURRENT_GUARD = 170.0
MOTION_GUARD = 0.020

REFRESH_HZ = 20.0
PERIOD = 1.0 / REFRESH_HZ

if os.environ.get("XHAND_CONFIRM") != "EXECUTE-THUMB-MICRO":
    raise SystemExit("[ABORT] confirmation missing")


def read_state(driver):
    fresh = driver.read_fresh_state(force_update=True)

    if fresh is None:
        raise RuntimeError("no fresh state")

    fs = fresh.state.finger_state

    q = [float(f.position) for f in fs]
    cur = [float(f.torque) for f in fs]

    for i, f in enumerate(fs):
        errors = (
            int(f.commboard_err),
            int(f.jonitboard_err),
            int(f.tipboard_err),
        )

        if any(errors):
            raise RuntimeError(
                f"{JOINT_NAMES[i]} errors={errors}"
            )

    return q, cur


def make_command(q0, joint, target):
    specs = []

    for i in range(12):
        if i == joint:
            specs.append(
                JointCommandSpec(
                    position=float(target),
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


def run_window(driver, command, q0, joint, seconds, label):
    deadline = time.monotonic() + seconds

    peak_i = 0.0
    peak_dq = 0.0
    last_q = q0[joint]

    while time.monotonic() < deadline:
        start = time.monotonic()

        driver.refresh_active_command(command)

        time.sleep(0.020)

        q, cur = read_state(driver)

        dq = q[joint] - q0[joint]

        peak_i = max(
            peak_i,
            abs(cur[joint]),
        )

        peak_dq = max(
            peak_dq,
            abs(dq),
        )

        last_q = q[joint]

        if abs(cur[joint]) > CURRENT_GUARD:
            raise RuntimeError(
                f"{label}: current={cur[joint]:+.1f}mA"
            )

        if abs(dq) > MOTION_GUARD:
            raise RuntimeError(
                f"{label}: dq={dq:+.6f}rad"
            )

        driver.poll_watchdog()

        elapsed = time.monotonic() - start

        if elapsed < PERIOD:
            time.sleep(PERIOD - elapsed)

    print(
        f"[PASS] {label:12s} "
        f"q={last_q:+.6f} "
        f"peak_I={peak_i:.1f}mA "
        f"peak_dq={peak_dq:.6f}rad"
    )

    return last_q


backend = VendorXHandBackend()

driver = XHandDriverCore(
    backend,
    command_timeout_s=0.5,
    disable_frames=3,
)

opened = False

try:
    did = driver.open(IFACE)
    opened = True

    print("=" * 76)
    print("XHAND THUMB ROTATION MICRO VALIDATION")
    print("=" * 76)
    print(f"[PASS] device_id={did}")

    for joint in TEST_JOINTS:
        print()
        print(
            f"========== {JOINT_NAMES[joint]} =========="
        )

        q0, cur0 = read_state(driver)

        lo, hi = LIMITS[joint]

        print(
            f"baseline q={q0[joint]:+.6f} "
            f"I={cur0[joint]:+.1f}mA "
            f"limit=[{lo:+.2f}, {hi:+.2f}]"
        )

        # 留出 0.02 rad 安全余量，避免在机械/软件限位附近上驱动
        if not (
            lo + 0.020
            <= q0[joint]
            <= hi - 0.020
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint]} baseline too close "
                f"to/outside limit: q={q0[joint]:+.6f}. "
                f"Move it manually to a middle position first."
            )

        hold = make_command(
            q0,
            joint,
            q0[joint],
        )

        plus = make_command(
            q0,
            joint,
            q0[joint] + DELTA,
        )

        minus = make_command(
            q0,
            joint,
            q0[joint] - DELTA,
        )

        print("[1/5] TAKEOVER HOLD")
        run_window(
            driver,
            hold,
            q0,
            joint,
            0.60,
            "HOLD",
        )

        print("[2/5] +0.005")
        q_plus = run_window(
            driver,
            plus,
            q0,
            joint,
            0.80,
            "PLUS",
        )

        print("[3/5] RETURN")
        run_window(
            driver,
            hold,
            q0,
            joint,
            0.80,
            "RETURN1",
        )

        print("[4/5] -0.005")
        q_minus = run_window(
            driver,
            minus,
            q0,
            joint,
            0.80,
            "MINUS",
        )

        print("[5/5] RETURN")
        q_final = run_window(
            driver,
            hold,
            q0,
            joint,
            0.80,
            "RETURN2",
        )

        print(
            f"[RESULT] "
            f"plus_delta={q_plus-q0[joint]:+.6f} "
            f"minus_delta={q_minus-q0[joint]:+.6f} "
            f"final_error={q_final-q0[joint]:+.6f}"
        )

        driver.disable_multiframe()

        print(
            f"[PASS] {JOINT_NAMES[joint]} "
            "micro validation complete"
        )

        time.sleep(0.30)

    print()
    print(
        "[PASS] BOTH THUMB ROTATION JOINTS COMPLETE"
    )

except Exception as exc:
    print()
    print(
        f"[FAULT] {type(exc).__name__}: {exc}"
    )

finally:
    if opened:
        print()
        print(
            "[SAFE] powerless x3 + EtherCAT close"
        )

        try:
            driver.close()
            print("[PASS] safely closed")
        except Exception as exc:
            print(f"[WARN] close: {exc}")
