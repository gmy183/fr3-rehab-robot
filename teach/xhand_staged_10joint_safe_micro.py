import os
import time
import statistics

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

ACTIVE = [
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

LIMITS = {
    0:  (0.00, 1.57),
    2:  (0.00, 1.57),
    4:  (0.00, 1.92),
    5:  (0.00, 1.92),
    6:  (0.00, 1.92),
    7:  (0.00, 1.92),
    8:  (0.00, 1.92),
    9:  (0.00, 1.92),
    10: (0.00, 1.92),
    11: (0.00, 1.92),
}

MODE = 3
KP = 225
KI = 0
KD = 0
TOR_MAX = 150

MICRO = 0.005

CURRENT_GUARD = 170.0
TAKEOVER_GUARD = 0.020

WATCHDOG = 0.5
DISABLE_FRAMES = 3

RATE = 20.0
PERIOD = 1.0 / RATE

if os.environ.get("XHAND_CONFIRM") != \
        "EXECUTE-XHAND-10-SAFE":
    raise SystemExit(
        "[ABORT] confirmation missing"
    )


def read_state(driver, max_attempts=8):
    """
    Read one semantically valid XHand state.

    The vendor SDK can transiently return a default HandState_t
    immediately after EtherCAT startup.  A successful SDK return
    alone is therefore not sufficient.
    """

    last_reason = "unknown"

    for _ in range(max_attempts):

        fresh = driver.read_fresh_state(
            force_update=True
        )

        if fresh is None:
            last_reason = "SDK read returned no fresh state"
            time.sleep(0.01)
            continue

        fs = fresh.state.finger_state

        if len(fs) != 12:
            last_reason = (
                f"joint count={len(fs)}"
            )
            time.sleep(0.01)
            continue

        q = []
        current = []
        raw = []

        hardware_error = None

        for i, f in enumerate(fs):

            errors = (
                int(f.commboard_err),
                int(f.jonitboard_err),
                int(f.tipboard_err),
            )

            if any(errors):
                hardware_error = (
                    f"{JOINT_NAMES[i]} "
                    f"errors={errors}"
                )
                break

            q.append(
                float(f.position)
            )

            current.append(
                float(f.torque)
            )

            raw.append(
                int(f.raw_position)
            )

        if hardware_error is not None:
            raise RuntimeError(
                hardware_error
            )

        # This real XHand normally reports large absolute
        # raw encoder values.  raw_position == 0 here indicates
        # a default/uninitialised finger-state entry.
        if any(v == 0 for v in raw):
            last_reason = (
                "default frame: raw_position contains zero"
            )
            time.sleep(0.01)
            continue

        # Additional protection against an all-zero default state.
        all_q_zero = all(
            abs(v) < 1e-12
            for v in q
        )

        all_i_zero = all(
            abs(v) < 1e-12
            for v in current
        )

        if all_q_zero and all_i_zero:
            last_reason = (
                "default frame: all positions/currents zero"
            )
            time.sleep(0.01)
            continue

        return q, current

    raise RuntimeError(
        "no semantically valid XHand state "
        f"after {max_attempts} attempts; "
        f"last_reason={last_reason}"
    )


def acquire_baseline(driver, n=15):
    """
    Acquire a robust powerless baseline before any active command.
    """

    rows_q = []
    rows_i = []

    deadline = time.monotonic() + 2.0

    while (
        len(rows_q) < n
        and time.monotonic() < deadline
    ):

        try:
            q, current = read_state(
                driver,
                max_attempts=4,
            )
        except RuntimeError as exc:

            text = str(exc)

            # Hardware errors are real faults and must not be hidden.
            if "errors=" in text:
                raise

            time.sleep(0.02)
            continue

        rows_q.append(q)
        rows_i.append(current)

        time.sleep(0.01)

    if len(rows_q) < n:
        raise RuntimeError(
            f"baseline acquisition failed: "
            f"{len(rows_q)}/{n} valid frames"
        )

    q0 = [
        statistics.median(
            row[i]
            for row in rows_q
        )
        for i in range(12)
    ]

    current0 = [
        statistics.median(
            row[i]
            for row in rows_i
        )
        for i in range(12)
    ]

    spans = [
        max(row[i] for row in rows_q)
        - min(row[i] for row in rows_q)
        for i in range(12)
    ]

    print(
        f"[PASS] baseline: "
        f"{n} valid frames, "
        f"worst_span={max(spans):.6f} rad"
    )

    if max(spans) > 0.020:
        raise RuntimeError(
            f"baseline not stationary: "
            f"worst_span={max(spans):.6f}"
        )

    return q0, current0


def make_command(q0, targets, enabled):
    enabled = set(enabled)

    specs = []

    for i in range(12):

        if i in enabled:
            specs.append(
                JointCommandSpec(
                    position=float(targets[i]),
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
    q_reference,
    enabled,
    duration,
    label,
):
    enabled = list(enabled)

    deadline = (
        time.monotonic()
        + duration
    )

    peak_current = 0.0
    peak_motion = 0.0
    worst_joint = None

    while time.monotonic() < deadline:

        cycle = time.monotonic()

        driver.refresh_active_command(
            command
        )

        time.sleep(0.020)

        q, current = read_state(
            driver
        )

        for i in enabled:

            dq = (
                q[i]
                - q_reference[i]
            )

            cur = abs(
                current[i]
            )

            if cur > peak_current:
                peak_current = cur

            if abs(dq) > peak_motion:
                peak_motion = abs(dq)
                worst_joint = i

            if cur > CURRENT_GUARD:
                raise RuntimeError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} "
                    f"current={cur:.1f}mA"
                )

            if abs(dq) > TAKEOVER_GUARD:
                raise RuntimeError(
                    f"{label}: "
                    f"{JOINT_NAMES[i]} "
                    f"dq={dq:+.6f}rad"
                )

        driver.poll_watchdog()

        elapsed = (
            time.monotonic()
            - cycle
        )

        if elapsed < PERIOD:
            time.sleep(
                PERIOD - elapsed
            )

    worst_name = (
        JOINT_NAMES[worst_joint]
        if worst_joint is not None
        else "none"
    )

    print(
        f"[PASS] {label:12s} "
        f"peak_I={peak_current:.1f}mA "
        f"peak_dq={peak_motion:.6f}rad "
        f"worst={worst_name}"
    )


print("=" * 78)
print("XHAND SAFE 10-JOINT STAGED INTEGRATION")
print("=" * 78)
print("mode      = 3")
print("kp        = 225")
print("tor_max   = 150 mA")
print("guard     = 170 mA / 0.020 rad")
print("FR3       = NO MOTION")
print("=" * 78)


backend = VendorXHandBackend()

driver = XHandDriverCore(
    backend,
    command_timeout_s=WATCHDOG,
    disable_frames=DISABLE_FRAMES,
)

opened = False

try:

    did = driver.open(IFACE)
    opened = True

    print(
        f"[PASS] device_id={did}"
    )

    print()
    print("[WAIT] acquiring valid XHand baseline...")

    q0, current0 = acquire_baseline(
        driver,
        n=15,
    )

    print()
    print("========== BASELINE ==========")

    for i in range(12):
        print(
            f"{i:02d} "
            f"{JOINT_NAMES[i]:20s} "
            f"q={q0[i]:+.6f} "
            f"I={current0[i]:+.1f}mA"
        )

    # -------------------------
    # 1. 硬限位检查
    # -------------------------

    for i in ACTIVE:

        lo, hi = LIMITS[i]

        if not (
            lo - 0.005
            <= q0[i]
            <= hi + 0.005
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[i]} "
                f"baseline outside control limit: "
                f"{q0[i]:+.6f}"
            )


    # -------------------------
    # 2. 生成安全 HOLD 姿态
    # -------------------------

    q_hold = list(q0)

    # thumb_bend 太靠近 0：
    # 轻微向范围内部偏 0.010 rad
    #
    # 仍然保持在已验证的 0.020 rad
    # 有界运动范围内。
    if q0[0] < 0.080:

        if q0[0] < 0.030:
            raise RuntimeError(
                "thumb_bend too close to "
                "lower mechanical limit; "
                "manually move it inward first"
            )

        q_hold[0] = (
            q0[0]
            + 0.010
        )

        print()
        print(
            "[ADJUST] thumb_bend safe hold: "
            f"{q0[0]:+.6f} "
            f"-> {q_hold[0]:+.6f}"
        )


    # -------------------------
    # 3. 生成每个关节的安全微动方向
    # -------------------------

    q_move = list(q_hold)

    print()
    print("========== SAFE MICRO TARGETS ==========")

    for i in ACTIVE:

        lo, hi = LIMITS[i]

        plus = (
            q_hold[i]
            + MICRO
        )

        minus = (
            q_hold[i]
            - MICRO
        )

        # 优先 +0.005。
        # 如果已经靠近上限，则改为 -0.005。
        if plus <= hi - 0.002:
            target = plus
            direction = "+"

        elif minus >= lo + 0.002:
            target = minus
            direction = "-"

        else:
            raise RuntimeError(
                f"{JOINT_NAMES[i]} "
                "has no safe micro direction"
            )

        q_move[i] = target

        print(
            f"{JOINT_NAMES[i]:20s} "
            f"hold={q_hold[i]:+.6f} "
            f"{direction}0.005 "
            f"-> {target:+.6f}"
        )


    # -------------------------
    # 4. 分阶段累计接管
    # -------------------------

    print()
    print("========== STAGED TAKEOVER ==========")

    enabled = []

    for stage, i in enumerate(
        ACTIVE,
        start=1,
    ):

        enabled.append(i)

        print(
            f"[STAGE {stage}/10] "
            f"enable {JOINT_NAMES[i]}"
        )

        command = make_command(
            q0,
            q_hold,
            enabled,
        )

        run_window(
            driver,
            command,
            q0,
            enabled,
            0.50,
            f"TAKEOVER-{stage}",
        )


    # -------------------------
    # 5. 10关节稳定 HOLD
    # -------------------------

    print()
    print("========== ALL10 HOLD ==========")

    hold_command = make_command(
        q0,
        q_hold,
        ACTIVE,
    )

    run_window(
        driver,
        hold_command,
        q0,
        ACTIVE,
        0.75,
        "ALL10-HOLD",
    )


    # -------------------------
    # 6. 安全方向微动
    # -------------------------

    print()
    print("========== SAFE WHOLE MICRO ==========")

    move_command = make_command(
        q0,
        q_move,
        ACTIVE,
    )

    run_window(
        driver,
        move_command,
        q0,
        ACTIVE,
        1.00,
        "WHOLE-MOVE",
    )


    # -------------------------
    # 7. 回安全 HOLD
    # -------------------------

    print()
    print("========== RETURN SAFE HOLD ==========")

    run_window(
        driver,
        hold_command,
        q0,
        ACTIVE,
        1.00,
        "RETURN-HOLD",
    )


    qf, currentf = read_state(
        driver
    )

    print()
    print("========== FINAL ==========")

    worst = 0.0

    for i in ACTIVE:

        err = (
            qf[i]
            - q_hold[i]
        )

        worst = max(
            worst,
            abs(err),
        )

        print(
            f"{JOINT_NAMES[i]:20s} "
            f"target={q_hold[i]:+.6f} "
            f"final={qf[i]:+.6f} "
            f"err={err:+.6f}"
        )

    print()

    print(
        f"worst_hold_error="
        f"{worst:.6f} rad"
    )

    print()
    print(
        "[PASS] XHAND SAFE 10-JOINT "
        "INTEGRATION COMPLETE"
    )


except KeyboardInterrupt:

    print()
    print("[STOP] operator interrupt")


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
