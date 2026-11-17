from __future__ import annotations

import math
import os
import statistics
import time
from pathlib import Path

import rclpy
import yaml

from action_msgs.msg import GoalStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .fr3_ptp_client import FR3PTPClient
from .fr3_ptp_contract import (
    JOINT_NAMES as FR3_JOINT_NAMES,
    relative_target,
    validate_target,
)

from .xhand_driver_core import (
    VendorXHandBackend,
    XHandDriverCore,
)

from .xhand_sdk_adapter import (
    JointCommandSpec,
    build_command,
)

from .xhand_state_mapper import (
    JOINT_NAMES as XHAND_JOINT_NAMES,
)


WS = Path.home() / "rehab_robot_ws"

PTP_ACTION = "/action_server/ptp_motion"


class ExecutionGateError(RuntimeError):
    pass


class RuntimeSafetyError(RuntimeError):
    pass


def read_yaml(path):
    return yaml.safe_load(
        Path(path).read_text(
            encoding="utf-8"
        )
    )


def load_configs():
    profile = read_yaml(
        WS / "config/xhand_motion_profile.yaml"
    )["xhand"]

    sequence = read_yaml(
        WS / "config/empty_grasp_sequence.yaml"
    )

    micro = read_yaml(
        WS / "config/xhand_micro_validation.yaml"
    )["micro_validation"]

    return profile, sequence, micro


def validate_micro_gate(
    profile,
    sequence,
    micro,
):
    gains = profile["active_gains"]

    # 正式 verified 仍必须保持 false。
    if bool(gains["verified"]):
        raise ExecutionGateError(
            "formal XHand verified unexpectedly true"
        )

    if micro["status"] != "passed":
        raise ExecutionGateError(
            "micro-validation status is not passed"
        )

    if (
        bool(
            micro[
                "formal_active_gains_verified"
            ]
        )
    ):
        raise ExecutionGateError(
            "micro-validation must not claim "
            "formal gain verification"
        )

    expected = (
        int(micro["mode"]),
        int(micro["kp"]),
        int(micro["ki"]),
        int(micro["kd"]),
    )

    actual = (
        int(profile["mode"]),
        int(gains["kp"]),
        int(gains["ki"]),
        int(gains["kd"]),
    )

    if actual != expected:
        raise ExecutionGateError(
            "XHand active parameters differ "
            "from micro-validated values"
        )

    if float(micro["delta_rad"]) > 0.005:
        raise ExecutionGateError(
            "micro delta exceeds 0.005 rad"
        )

    if int(micro["effort_limit_ma"]) > 150:
        raise ExecutionGateError(
            "micro effort exceeds 150 mA"
        )

    if (
        sequence["fr3"][
            "automatic_recovery_on_fault"
        ]
        is not False
    ):
        raise ExecutionGateError(
            "automatic FR3 recovery must remain false"
        )

    if (
        sequence["fr3"][
            "automatic_return_on_fault"
        ]
        is not False
    ):
        raise ExecutionGateError(
            "automatic FR3 return must remain false"
        )

    active = tuple(
        micro["whole_hand"]["active_joints"]
    )

    forbidden = set(
        profile["fixed_joints"]
        + profile["excluded_joints"]
    )

    bad = [
        name
        for name in active
        if name in forbidden
    ]

    if bad:
        raise ExecutionGateError(
            "micro active group contains "
            "forbidden joints: "
            + ", ".join(bad)
        )

    return active


class CombinedNode(Node):
    def __init__(self):
        super().__init__(
            "rehab_empty_grasp_executor"
        )

        self.last_joint_state = None
        self.last_joint_state_time = None
        self.samples = 0
        self.active_goal_handle = None

        # Last actual active XHand command transmission.
        # Used to enforce the validated 20 Hz refresh cadence.
        self.last_xhand_refresh_time = None

        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._joint_cb,
            100,
        )

    def _joint_cb(self, msg):
        self.last_joint_state = msg
        self.last_joint_state_time = (
            time.monotonic()
        )
        self.samples += 1


def spin_sleep(node, seconds):
    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(
                0.02,
                max(
                    0.0,
                    deadline - time.monotonic(),
                ),
            ),
        )


def ordered_fr3_state(node):
    msg = node.last_joint_state

    if msg is None:
        raise RuntimeSafetyError(
            "FR3 joint state unavailable"
        )

    lookup = {
        name: i
        for i, name in enumerate(msg.name)
    }

    missing = [
        name
        for name in FR3_JOINT_NAMES
        if name not in lookup
    ]

    if missing:
        raise RuntimeSafetyError(
            "missing FR3 joints: "
            + ", ".join(missing)
        )

    q = tuple(
        float(
            msg.position[
                lookup[name]
            ]
        )
        for name in FR3_JOINT_NAMES
    )

    if (
        msg.velocity
        and len(msg.velocity) == len(msg.name)
    ):
        dq = tuple(
            float(
                msg.velocity[
                    lookup[name]
                ]
            )
            for name in FR3_JOINT_NAMES
        )
    else:
        dq = tuple(
            0.0
            for _ in FR3_JOINT_NAMES
        )

    if not all(
        math.isfinite(v)
        for v in q + dq
    ):
        raise RuntimeSafetyError(
            "non-finite FR3 state"
        )

    return q, dq


def wait_fr3_live(node):
    deadline = time.monotonic() + 8.0

    while (
        time.monotonic() < deadline
        and node.samples < 50
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

    if node.samples < 50:
        raise RuntimeSafetyError(
            "insufficient FR3 state samples"
        )

    q, dq = ordered_fr3_state(node)

    q = validate_target(q)

    vmax = max(
        abs(v)
        for v in dq
    )

    if vmax > 0.020:
        raise RuntimeSafetyError(
            "FR3 not stationary enough "
            f"before execution: {vmax:.6f} rad/s"
        )

    return q


def assert_fr3_fresh(node):
    if node.last_joint_state_time is None:
        raise RuntimeSafetyError(
            "FR3 state has never been received"
        )

    age = (
        time.monotonic()
        - node.last_joint_state_time
    )

    if age > 0.25:
        raise RuntimeSafetyError(
            f"FR3 state stale: {age:.3f}s"
        )

    _, dq = ordered_fr3_state(node)

    vmax = max(
        abs(v)
        for v in dq
    )

    # 正常 PTP 最大设置为 0.020。
    # 0.050 仅作为额外 runaway software guard。
    if vmax > 0.050:
        raise RuntimeSafetyError(
            "FR3 measured velocity guard exceeded: "
            f"{vmax:.6f} rad/s"
        )



def refresh_fr3_state(
    node,
    *,
    timeout_s=1.0,
    max_age_s=0.05,
):
    """
    Pump the ROS executor until a genuinely fresh FR3 JointState
    has been received.

    This function performs no command operation.
    """

    deadline = (
        time.monotonic()
        + float(timeout_s)
    )

    while time.monotonic() < deadline:

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        stamp = (
            node.last_joint_state_time
        )

        if stamp is None:
            continue

        age = (
            time.monotonic()
            - stamp
        )

        if age <= float(max_age_s):
            assert_fr3_fresh(node)
            return

    age = None

    if node.last_joint_state_time is not None:
        age = (
            time.monotonic()
            - node.last_joint_state_time
        )

    raise RuntimeSafetyError(
        "unable to obtain fresh FR3 state "
        f"before PTP send; age={age}"
    )


def fr3_pose_error(
    current,
    expected,
):
    if len(current) != 7:
        raise RuntimeSafetyError(
            "current FR3 pose length != 7"
        )

    if len(expected) != 7:
        raise RuntimeSafetyError(
            "expected FR3 pose length != 7"
        )

    return max(
        abs(float(a) - float(b))
        for a, b in zip(
            current,
            expected,
        )
    )

def read_xhand_sample(driver):
    fresh = driver.read_fresh_state(False)

    if fresh is None:
        raise RuntimeSafetyError(
            "XHand fresh state unavailable"
        )

    fingers = fresh.state.finger_state

    if len(fingers) != 12:
        raise RuntimeSafetyError(
            "XHand joint count != 12"
        )

    pos = []
    cur = []
    modes = []

    for i, finger in enumerate(fingers):
        q = float(finger.position)

        # 当前 SDK 的 torque 已经是 signed mA。
        current = float(finger.torque)

        if not math.isfinite(q):
            raise RuntimeSafetyError(
                f"{XHAND_JOINT_NAMES[i]} "
                "position non-finite"
            )

        if not math.isfinite(current):
            raise RuntimeSafetyError(
                f"{XHAND_JOINT_NAMES[i]} "
                "current non-finite"
            )

        errors = (
            int(finger.commboard_err),
            int(finger.jonitboard_err),
            int(finger.tipboard_err),
        )

        if any(errors):
            raise RuntimeSafetyError(
                f"{XHAND_JOINT_NAMES[i]} "
                f"hardware errors={errors}"
            )

        pos.append(q)
        cur.append(current)

        # 当前固件模式在 raw_position 低 4 bit。
        modes.append(
            int(finger.raw_position) & 0xF
        )

    return (
        tuple(pos),
        tuple(cur),
        tuple(modes),
    )


def xhand_baseline(
    driver,
    *,
    n=20,
    require_mode0=True,
):
    rows = []

    for _ in range(n):
        p, _, modes = read_xhand_sample(
            driver
        )

        if (
            require_mode0
            and any(
                int(mode) != 0
                for mode in modes
            )
        ):
            raise RuntimeSafetyError(
                "XHand baseline contains "
                f"non-powerless modes={modes}"
            )

        rows.append(p)
        time.sleep(0.01)

    return tuple(
        statistics.median(
            row[i]
            for row in rows
        )
        for i in range(12)
    )


def build_micro_command(
    reference,
    *,
    active_joints,
    targets,
    micro,
):
    active = set(active_joints)

    specs = []

    for i, name in enumerate(
        XHAND_JOINT_NAMES
    ):
        if name in active:
            specs.append(
                JointCommandSpec(
                    position=float(
                        targets[i]
                    ),
                    effort_limit=int(
                        micro[
                            "effort_limit_ma"
                        ]
                    ),
                    mode=int(
                        micro["mode"]
                    ),
                    kp=int(
                        micro["kp"]
                    ),
                    ki=int(
                        micro["ki"]
                    ),
                    kd=int(
                        micro["kd"]
                    ),
                )
            )

        else:
            specs.append(
                JointCommandSpec(
                    position=float(
                        reference[i]
                    ),
                    effort_limit=0,
                    mode=0,
                    kp=0,
                    ki=0,
                    kd=0,
                )
            )

    return build_command(specs)


def check_xhand_sample(
    sample,
    *,
    reference,
    active_joints,
    micro,
    allow_active_mode=True,
):
    pos, cur, modes = sample

    active = set(active_joints)

    max_current = float(
        micro[
            "safety_guard"
        ][
            "max_abs_current_ma"
        ]
    )

    max_motion = float(
        micro[
            "safety_guard"
        ][
            "max_abs_joint_motion_rad"
        ]
    )

    seen_mode3 = False

    for i, name in enumerate(
        XHAND_JOINT_NAMES
    ):
        if abs(cur[i]) > max_current:
            raise RuntimeSafetyError(
                f"{name} current guard exceeded: "
                f"{cur[i]:.1f}mA"
            )

        if name in active:
            delta = (
                pos[i]
                - reference[i]
            )

            if abs(delta) > max_motion:
                raise RuntimeSafetyError(
                    f"{name} motion guard exceeded: "
                    f"q={pos[i]:+.6f}, "
                    f"reference={reference[i]:+.6f}, "
                    f"delta={delta:+.6f} rad, "
                    f"current={cur[i]:+.1f} mA, "
                    f"mode={modes[i]}, "
                    f"limit={max_motion:.6f} rad"
                )

            if allow_active_mode:
                if modes[i] not in (0, 3):
                    raise RuntimeSafetyError(
                        f"{name} unexpected mode "
                        f"{modes[i]}"
                    )

                if modes[i] == 3:
                    seen_mode3 = True

        else:
            if modes[i] != 0:
                raise RuntimeSafetyError(
                    f"inactive {name} "
                    f"entered mode={modes[i]}"
                )

    return seen_mode3


def run_xhand_window(
    node,
    driver,
    command,
    *,
    reference,
    active_joints,
    micro,
    seconds,
    label,
):
    period = (
        1.0
        / float(
            micro["refresh_hz"]
        )
    )

    seen_mode3 = set()

    peaks = {
        name: 0.0
        for name in active_joints
    }

    mins = {
        name: 1e9
        for name in active_joints
    }

    last_pos = None

    deadline = (
        time.monotonic()
        + float(seconds)
    )

    # Critical gate before the FIRST active XHand frame.
    refresh_fr3_state(
        node,
        timeout_s=1.0,
        max_age_s=0.05,
    )

    print(
        f"[PASS] {label} FR3 freshness gate"
    )

    while time.monotonic() < deadline:
        cycle = time.monotonic()

        # Critical ordering:
        # fresh FR3 state FIRST, active XHand refresh SECOND.
        refresh_fr3_state(
            node,
            timeout_s=0.25,
            max_age_s=0.05,
        )

        driver.refresh_active_command(
            command
        )

        node.last_xhand_refresh_time = (
            time.monotonic()
        )

        rclpy.spin_once(
            node,
            timeout_sec=0.01,
        )

        sample = read_xhand_sample(
            driver
        )

        pos, cur, modes = sample

        check_xhand_sample(
            sample,
            reference=reference,
            active_joints=active_joints,
            micro=micro,
            allow_active_mode=True,
        )

        for name in active_joints:
            i = XHAND_JOINT_NAMES.index(
                name
            )

            delta = (
                pos[i]
                - reference[i]
            )

            peaks[name] = max(
                peaks[name],
                abs(delta),
            )

            mins[name] = min(
                mins[name],
                delta,
            )

            if modes[i] == 3:
                seen_mode3.add(name)

        last_pos = pos

        assert_fr3_fresh(node)

        driver.poll_watchdog()

        elapsed = (
            time.monotonic()
            - cycle
        )

        if elapsed < period:
            spin_sleep(
                node,
                period - elapsed,
            )

    missing = [
        name
        for name in active_joints
        if name not in seen_mode3
    ]

    if missing:
        raise RuntimeSafetyError(
            f"{label}: mode=3 not observed for "
            + ", ".join(missing)
        )

    return {
        "last_positions": last_pos,
        "peak_abs": peaks,
        "minimum_delta": mins,
    }


def maintain_xhand_once(
    node,
    driver,
    *,
    command,
    reference,
    active_joints,
    micro,
):
    # Always keep FR3 state callbacks fresh.
    refresh_fr3_state(
        node,
        timeout_s=0.25,
        max_age_s=0.05,
    )

    # IMPORTANT:
    # The 2026-08-30 XHand micro-validation used 20 Hz
    # active-command refresh.  During an FR3 Action we must
    # preserve exactly that validated cadence rather than
    # transmit on every ROS polling iteration.
    if command is not None:

        refresh_hz = float(
            micro["refresh_hz"]
        )

        if refresh_hz <= 0.0:
            raise RuntimeSafetyError(
                "invalid XHand refresh_hz"
            )

        period = 1.0 / refresh_hz

        now = time.monotonic()

        last = getattr(
            node,
            "last_xhand_refresh_time",
            None,
        )

        due = (
            last is None
            or (now - last) >= period
        )

        if due:
            # FR3 was refreshed immediately above.
            # Only now may an active XHand frame be transmitted.
            driver.refresh_active_command(
                command
            )

            node.last_xhand_refresh_time = (
                time.monotonic()
            )

    sample = read_xhand_sample(
        driver
    )

    if command is None:
        _, cur, modes = sample

        max_current = float(
            micro[
                "safety_guard"
            ][
                "max_abs_current_ma"
            ]
        )

        if any(
            abs(v) > max_current
            for v in cur
        ):
            raise RuntimeSafetyError(
                "XHand current guard exceeded "
                "while powerless"
            )

        if any(
            mode != 0
            for mode in modes
        ):
            raise RuntimeSafetyError(
                "XHand not powerless "
                "during FR3 pregrasp"
            )

    else:
        check_xhand_sample(
            sample,
            reference=reference,
            active_joints=active_joints,
            micro=micro,
            allow_active_mode=True,
        )

    assert_fr3_fresh(node)

    driver.poll_watchdog()


def cancel_goal(
    node,
    goal_handle,
):
    if goal_handle is None:
        return

    try:
        future = (
            goal_handle.cancel_goal_async()
        )

        deadline = (
            time.monotonic()
            + 2.0
        )

        while (
            not future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

    except Exception:
        pass


def execute_ptp(
    node,
    client,
    target,
    *,
    driver,
    hold_command,
    xhand_reference,
    active_joints,
    micro,
    velocity,
    label,
    expected_current,
):
    # --------------------------------------------------------
    # CRITICAL PRE-SEND SAFETY GATE
    #
    # No PTP goal object is sent until:
    #   1. a fresh FR3 JointState is obtained;
    #   2. the current pose still matches the expected
    #      previous-stage pose.
    # --------------------------------------------------------

    refresh_fr3_state(
        node,
        timeout_s=1.0,
        max_age_s=0.05,
    )

    q_before, _ = ordered_fr3_state(
        node
    )

    pre_error = fr3_pose_error(
        q_before,
        expected_current,
    )

    if pre_error > 0.010:
        raise RuntimeSafetyError(
            f"{label}: FR3 pre-send pose "
            f"mismatch={pre_error:.6f} rad"
        )

    print(
        f"[PASS] {label} pre-send "
        f"FR3 state fresh; "
        f"pose_error={pre_error:.6f} rad"
    )

    goal = client.build_goal(
        target,
        maximum_velocity_rad_s=velocity,
        goal_tolerance=0.001,
    )

    send_future = (
        client.send_goal_async(
            goal,
            execute=True,
        )
    )

    try:
        while not send_future.done():
            maintain_xhand_once(
                node,
                driver,
                command=hold_command,
                reference=xhand_reference,
                active_joints=active_joints,
                micro=micro,
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.02,
            )

    except Exception:
        try:
            driver.disable_multiframe()
        except Exception:
            pass

        deadline = (
            time.monotonic()
            + 1.5
        )

        while (
            not send_future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        if send_future.done():
            handle = send_future.result()

            if (
                handle is not None
                and handle.accepted
            ):
                cancel_goal(
                    node,
                    handle,
                )

        raise

    handle = send_future.result()

    if (
        handle is None
        or not handle.accepted
    ):
        raise RuntimeSafetyError(
            f"{label}: FR3 goal rejected"
        )

    node.active_goal_handle = handle

    print(
        f"[INFO] {label} PTP goal accepted"
    )

    result_future = (
        handle.get_result_async()
    )

    try:
        while not result_future.done():
            maintain_xhand_once(
                node,
                driver,
                command=hold_command,
                reference=xhand_reference,
                active_joints=active_joints,
                micro=micro,
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.02,
            )

    except Exception:
        cancel_goal(
            node,
            handle,
        )

        raise

    wrapped = result_future.result()

    node.active_goal_handle = None

    if (
        wrapped is None
        or wrapped.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        raise RuntimeSafetyError(
            f"{label}: FR3 action status="
            f"{getattr(wrapped, 'status', None)}"
        )

    spin_sleep(
        node,
        0.15,
    )

    q_now, _ = ordered_fr3_state(
        node
    )

    error = max(
        abs(a - b)
        for a, b in zip(
            q_now,
            target,
        )
    )

    if error > 0.010:
        raise RuntimeSafetyError(
            f"{label}: final FR3 error "
            f"{error:.6f} rad"
        )

    print(
        f"[PASS] {label} "
        f"max_final_error={error:.6f} rad"
    )


def main():
    if os.geteuid() != 0:
        raise ExecutionGateError(
            "combined executor must run "
            "through the sudo wrapper"
        )

    profile, sequence, micro = (
        load_configs()
    )

    whole_active = validate_micro_gate(
        profile,
        sequence,
        micro,
    )

    single_joint = (
        micro["single_joint"]["joint"]
    )

    if single_joint != "index_joint1":
        raise ExecutionGateError(
            "unexpected single-joint "
            "micro-validation target"
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
            "FR3 + XHand COMBINED MICRO EXECUTOR"
        )
        print(
            "EMPTY LOAD / BOUNDED MICRO MOTION"
        )
        print(
            "============================================================"
        )

        q_start = wait_fr3_live(
            node
        )

        print(
            "FR3 q_start =",
            [
                round(v, 6)
                for v in q_start
            ],
        )

        client = FR3PTPClient(
            node,
            PTP_ACTION,
        )

        if not client.wait_ready(
            timeout_sec=5.0
        ):
            raise ExecutionGateError(
                "FR3 PTP server unavailable"
            )

        fr3_cfg = sequence["fr3"]

        q1 = relative_target(
            q_start,
            joint_index=0,
            delta_rad=float(
                fr3_cfg[
                    "pregrasp_delta_joint1_rad"
                ]
            ),
        )

        q2 = relative_target(
            q1,
            joint_index=0,
            delta_rad=float(
                fr3_cfg[
                    "grasp_delta_joint1_rad"
                ]
            ),
        )

        q3 = relative_target(
            q2,
            joint_index=0,
            delta_rad=float(
                fr3_cfg[
                    "rehab_stage_1"
                ][
                    "delta_joint1_rad"
                ]
            ),
        )

        print(
            "FR3 q1 =",
            [
                round(v, 6)
                for v in q1
            ],
        )

        print(
            "FR3 q2 =",
            [
                round(v, 6)
                for v in q2
            ],
        )

        print(
            "FR3 q3 =",
            [
                round(v, 6)
                for v in q3
            ],
        )

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
            f"XHand serial={actual_serial}"
        )

        if (
            expected_serial
            and actual_serial
            != expected_serial
        ):
            raise ExecutionGateError(
                "XHand serial mismatch"
            )

        # ====================================================
        # Stage 1: passive baseline
        # ====================================================

        driver.disable_multiframe()
        time.sleep(0.15)

        x0 = xhand_baseline(
            driver,
            require_mode0=True,
        )

        print(
            "[PASS] XHand passive baseline"
        )

        # ====================================================
        # Stage 2: single index_joint1 micro test
        # ====================================================

        single = (
            single_joint,
        )

        hold_targets = list(x0)

        cmd = build_micro_command(
            x0,
            active_joints=single,
            targets=hold_targets,
            micro=micro,
        )

        takeover = run_xhand_window(
            node,
            driver,
            cmd,
            reference=x0,
            active_joints=single,
            micro=micro,
            seconds=0.6,
            label="single takeover",
        )

        drift_limit = float(
            micro[
                "safety_guard"
            ][
                "max_takeover_drift_rad"
            ]
        )

        if (
            takeover["peak_abs"][
                single_joint
            ]
            > drift_limit
        ):
            raise RuntimeSafetyError(
                "single takeover drift exceeded"
            )

        x1 = (
            takeover["last_positions"]
        )

        move_targets = list(x1)

        i = XHAND_JOINT_NAMES.index(
            single_joint
        )

        move_targets[i] += float(
            micro["delta_rad"]
        )

        move_cmd = build_micro_command(
            x1,
            active_joints=single,
            targets=move_targets,
            micro=micro,
        )

        movement = run_xhand_window(
            node,
            driver,
            move_cmd,
            reference=x1,
            active_joints=single,
            micro=micro,
            seconds=0.8,
            label="single move",
        )

        if (
            movement[
                "minimum_delta"
            ][
                single_joint
            ]
            < -0.010
        ):
            raise RuntimeSafetyError(
                "index_joint1 unexpected "
                "reverse motion"
            )

        driver.disable_multiframe()
        time.sleep(0.15)

        print(
            "[PASS] XHand single-joint micro stage"
        )

        # ====================================================
        # Stage 3: whole-hand bounded micro validation
        # ====================================================

        x0 = xhand_baseline(
            driver,
            require_mode0=True,
        )

        hold_targets = list(x0)

        whole_hold = (
            build_micro_command(
                x0,
                active_joints=whole_active,
                targets=hold_targets,
                micro=micro,
            )
        )

        takeover = run_xhand_window(
            node,
            driver,
            whole_hold,
            reference=x0,
            active_joints=whole_active,
            micro=micro,
            seconds=0.8,
            label="whole takeover",
        )

        for name in whole_active:
            if (
                takeover[
                    "peak_abs"
                ][name]
                > drift_limit
            ):
                raise RuntimeSafetyError(
                    f"{name} takeover drift exceeded"
                )

        x1 = (
            takeover["last_positions"]
        )

        targets = list(x1)

        for name in whole_active:
            idx = (
                XHAND_JOINT_NAMES
                .index(name)
            )

            targets[idx] += float(
                micro["delta_rad"]
            )

        whole_test = (
            build_micro_command(
                x1,
                active_joints=whole_active,
                targets=targets,
                micro=micro,
            )
        )

        run_xhand_window(
            node,
            driver,
            whole_test,
            reference=x1,
            active_joints=whole_active,
            micro=micro,
            seconds=1.0,
            label="whole micro test",
        )

        driver.disable_multiframe()
        time.sleep(0.15)

        print(
            "[PASS] XHand whole-hand bounded micro stage"
        )

        # ====================================================
        # Stage 4: FR3 pregrasp
        # XHand 必须是 powerless。
        # ====================================================

        x_powerless = xhand_baseline(
            driver,
            require_mode0=True,
        )

        execute_ptp(
            node,
            client,
            q1,
            driver=driver,
            hold_command=None,
            xhand_reference=x_powerless,
            active_joints=(),
            micro=micro,
            velocity=float(
                fr3_cfg[
                    "max_joint_velocity_rad_s"
                ]
            ),
            label="FR3 pregrasp",
            expected_current=q_start,
        )

        # ====================================================
        # Stage 5: XHand grasp +0.005
        # ====================================================

        xg = xhand_baseline(
            driver,
            require_mode0=True,
        )

        hold_targets = list(xg)

        grasp_takeover_cmd = (
            build_micro_command(
                xg,
                active_joints=whole_active,
                targets=hold_targets,
                micro=micro,
            )
        )

        takeover = run_xhand_window(
            node,
            driver,
            grasp_takeover_cmd,
            reference=xg,
            active_joints=whole_active,
            micro=micro,
            seconds=0.6,
            label="grasp takeover",
        )

        xg1 = (
            takeover["last_positions"]
        )

        grasp_targets = list(xg1)

        for name in whole_active:
            idx = (
                XHAND_JOINT_NAMES
                .index(name)
            )

            grasp_targets[idx] += float(
                micro["delta_rad"]
            )

        grasp_cmd = (
            build_micro_command(
                xg1,
                active_joints=whole_active,
                targets=grasp_targets,
                micro=micro,
            )
        )

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=xg1,
            active_joints=whole_active,
            micro=micro,
            seconds=1.0,
            label="XHand grasp",
        )

        print(
            "[PASS] XHand grasp hold armed"
        )

        # ====================================================
        # Stage 6: FR3 grasp
        # XHand 20 Hz 持续刷新。
        # ====================================================

        execute_ptp(
            node,
            client,
            q2,
            driver=driver,
            hold_command=grasp_cmd,
            xhand_reference=xg1,
            active_joints=whole_active,
            micro=micro,
            velocity=float(
                fr3_cfg[
                    "max_joint_velocity_rad_s"
                ]
            ),
            label="FR3 grasp",
            expected_current=q1,
        )

        # ====================================================
        # Stage 7: rehab stage 1
        # ====================================================

        run_xhand_window(
            node,
            driver,
            grasp_cmd,
            reference=xg1,
            active_joints=whole_active,
            micro=micro,
            seconds=float(
                sequence["timing"][
                    "hold_time_s"
                ]
            ),
            label="inter-stage hold",
        )

        execute_ptp(
            node,
            client,
            q3,
            driver=driver,
            hold_command=grasp_cmd,
            xhand_reference=xg1,
            active_joints=whole_active,
            micro=micro,
            velocity=float(
                fr3_cfg[
                    "max_joint_velocity_rad_s"
                ]
            ),
            label="FR3 rehab stage 1",
            expected_current=q2,
        )

        # ====================================================
        # Stage 8: controlled finish
        # XHand 去使能。
        # FR3 正常结束也不自动回起点。
        # ====================================================

        driver.disable_multiframe()

        print()
        print(
            "============================================================"
        )
        print(
            "[PASS] COMBINED MICRO EXECUTION COMPLETE"
        )
        print(
            "============================================================"
        )
        print(
            "XHand: powerless"
        )
        print(
            "FR3: left at final validated micro pose"
        )
        print(
            "No automatic FR3 return was executed."
        )

        return 0

    except KeyboardInterrupt:
        print(
            "\n[FAULT] operator interrupt"
        )

        if node.active_goal_handle is not None:
            cancel_goal(
                node,
                node.active_goal_handle,
            )

        raise

    except Exception:
        if node.active_goal_handle is not None:
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
                    "[WARN] XHand close: "
                    f"{exc}"
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
            "[SAFE] No automatic FR3 "
            "recovery/return will be attempted."
        )

        raise SystemExit(2)
