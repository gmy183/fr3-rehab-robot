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
JOINT = 2

MODE = 3
KP = 225
KI = 0
KD = 0
TOR_MAX = 150

CURRENT_GUARD = 170.0
MOTION_GUARD = 0.020

STEPS = [0.005, 0.010, 0.015]

if os.environ.get("XHAND_CONFIRM") != "EXECUTE-THUMB2-DIRECTION":
    raise SystemExit("[ABORT] confirmation missing")


def read_state(driver):
    fresh = driver.read_fresh_state(
        force_update=True
    )

    if fresh is None:
        raise RuntimeError("no fresh state")

    fs = fresh.state.finger_state

    return (
        [float(f.position) for f in fs],
        [float(f.torque) for f in fs],
    )


def command_for(q0, target):
    specs = []

    for i in range(12):

        if i == JOINT:
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

    print("=" * 72)
    print("THUMB_ROTA_JOINT2 DIRECTION PROBE")
    print("=" * 72)
    print(f"[PASS] device_id={did}")

    q0, i0 = read_state(driver)

    base = q0[JOINT]

    print(
        f"[BASELINE] q={base:+.6f} "
        f"I={i0[JOINT]:+.1f}mA"
    )

    if not (0.05 <= base <= 1.53):
        raise RuntimeError(
            f"baseline unsuitable: {base:+.6f}"
        )

    # 先稳定接管
    hold = command_for(q0, base)

    end = time.monotonic() + 0.50

    while time.monotonic() < end:
        driver.refresh_active_command(hold)
        time.sleep(0.02)

        q, cur = read_state(driver)

        dq = q[JOINT] - base

        if abs(cur[JOINT]) > CURRENT_GUARD:
            raise RuntimeError(
                f"HOLD current={cur[JOINT]:+.1f}"
            )

        if abs(dq) > 0.015:
            raise RuntimeError(
                f"HOLD drift={dq:+.6f}"
            )

        time.sleep(0.03)

    print("[PASS] HOLD")

    for step in STEPS:

        target = base + step

        if target >= 1.55:
            raise RuntimeError(
                f"target too high: {target:+.6f}"
            )

        cmd = command_for(
            q0,
            target,
        )

        peak_abs = 0.0
        last_dq = 0.0

        end = time.monotonic() + 0.35

        while time.monotonic() < end:

            driver.refresh_active_command(cmd)

            time.sleep(0.02)

            q, cur = read_state(driver)

            dq = q[JOINT] - base
            last_dq = dq

            peak_abs = max(
                peak_abs,
                abs(dq),
            )

            if abs(cur[JOINT]) > CURRENT_GUARD:
                raise RuntimeError(
                    f"step={step:.3f} "
                    f"current={cur[JOINT]:+.1f}"
                )

            if abs(dq) > MOTION_GUARD:
                raise RuntimeError(
                    f"step={step:.3f} "
                    f"motion={dq:+.6f}"
                )

            # 出现明确反向运动，立即终止
            if dq < -0.006:
                raise RuntimeError(
                    f"clear reverse response: "
                    f"command=+{step:.3f}, "
                    f"dq={dq:+.6f}"
                )

            driver.poll_watchdog()

            time.sleep(0.03)

        print(
            f"[STEP +{step:.3f}] "
            f"final_dq={last_dq:+.6f} "
            f"peak_abs={peak_abs:.6f}"
        )

    print()
    print(
        "[PASS] direction probe completed"
    )

finally:

    if opened:
        print(
            "[SAFE] powerless x3 + EtherCAT close"
        )

        try:
            driver.close()
            print("[PASS] safely closed")
        except Exception as exc:
            print(f"[WARN] close: {exc}")
