from __future__ import annotations

import time

import rclpy

from action_msgs.msg import GoalStatus

from rehab_robot_control.empty_grasp_executor import (
    RuntimeSafetyError,
    assert_fr3_fresh,
    cancel_goal,
    check_xhand_sample,
    fr3_pose_error,
    ordered_fr3_state,
    read_xhand_sample,
    refresh_fr3_state,
    spin_sleep,
)


def maintain_xhand_validated_once(
    node,
    driver,
    *,
    command,
    reference,
    active_joints,
    micro,
):
    """
    FR3 PTP execution-time XHand scheduler.

    The validated XHand micro-motion cadence is 20 Hz.

    Important:
      - FR3 state may be serviced faster.
      - XHand active command TX, state RX and safety checking
        are performed together only on the validated XHand
        refresh cycle.
      - We do NOT sample/check XHand on the intermediate
        high-rate PTP polling iterations.
    """

    refresh_fr3_state(
        node,
        timeout_s=0.25,
        max_age_s=0.05,
    )

    if command is None:
        raise RuntimeSafetyError(
            "microcycle PTP requires active XHand hold command"
        )

    refresh_hz = float(
        micro["refresh_hz"]
    )

    watchdog_s = float(
        micro["watchdog_s"]
    )

    if refresh_hz <= 0.0:
        raise RuntimeSafetyError(
            "invalid XHand refresh_hz"
        )

    if watchdog_s <= 0.0:
        raise RuntimeSafetyError(
            "invalid XHand watchdog"
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

    # ---------------------------------------------------------
    # Intermediate FR3 Action polling iteration.
    #
    # Do NOT perform an extra XHand state read here.
    # Keep XHand TX/RX/check together at validated 20 Hz.
    # ---------------------------------------------------------

    if not due:

        assert_fr3_fresh(node)

        if driver.poll_watchdog():
            raise RuntimeSafetyError(
                "XHand watchdog tripped "
                "between validated refresh cycles"
            )

        return False

    # ---------------------------------------------------------
    # Scheduling guard.
    #
    # 20 Hz nominal period = 0.050 s.
    # Do not allow the combined loop to silently approach the
    # 0.5 s watchdog timeout.
    # ---------------------------------------------------------

    if last is not None:

        gap = now - last

        max_gap = min(
            0.25,
            watchdog_s * 0.5,
        )

        if gap > max_gap:
            raise RuntimeSafetyError(
                "XHand refresh scheduling gap exceeded: "
                f"{gap:.3f}s > {max_gap:.3f}s"
            )

        previous_peak = getattr(
            node,
            "microcycle_xhand_max_gap_s",
            0.0,
        )

        node.microcycle_xhand_max_gap_s = max(
            float(previous_peak),
            float(gap),
        )

    # ---------------------------------------------------------
    # One validated XHand cycle:
    #
    #   active command
    #       -> state read
    #       -> current/motion/mode guards
    # ---------------------------------------------------------

    driver.refresh_active_command(
        command
    )

    node.last_xhand_refresh_time = (
        time.monotonic()
    )

    sample = read_xhand_sample(
        driver
    )

    check_xhand_sample(
        sample,
        reference=reference,
        active_joints=active_joints,
        micro=micro,
        allow_active_mode=True,
    )

    assert_fr3_fresh(node)

    if driver.poll_watchdog():
        raise RuntimeSafetyError(
            "XHand watchdog tripped "
            "after active refresh"
        )

    return True


def execute_ptp_validated(
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
    """
    Microcycle-only PTP executor.

    FR3 safety semantics are inherited from the previously
    validated executor, while XHand TX/RX/check are kept on
    the validated 20 Hz cadence.
    """

    # ---------------------------------------------------------
    # Critical pre-send FR3 gate.
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # Wait for Action acceptance.
    # ---------------------------------------------------------

    try:

        while not send_future.done():

            maintain_xhand_validated_once(
                node,
                driver,
                command=hold_command,
                reference=xhand_reference,
                active_joints=active_joints,
                micro=micro,
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.01,
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

    # ---------------------------------------------------------
    # FR3 Action running.
    # XHand TX/RX/check remains exactly cadence-gated.
    # ---------------------------------------------------------

    try:

        while not result_future.done():

            maintain_xhand_validated_once(
                node,
                driver,
                command=hold_command,
                reference=xhand_reference,
                active_joints=active_joints,
                micro=micro,
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.01,
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

    # The 0.15 s post-action settling interval remains shorter
    # than both the scheduling guard and watchdog.
    spin_sleep(
        node,
        0.15,
    )

    q_now, _ = ordered_fr3_state(
        node
    )

    error = fr3_pose_error(
        q_now,
        target,
    )

    if error > 0.010:
        raise RuntimeSafetyError(
            f"{label}: final FR3 error "
            f"{error:.6f} rad"
        )

    max_gap = getattr(
        node,
        "microcycle_xhand_max_gap_s",
        None,
    )

    if max_gap is not None:

        print(
            f"[INFO] {label} "
            f"XHand max refresh gap="
            f"{float(max_gap):.4f}s"
        )

    print(
        f"[PASS] {label} "
        f"max_final_error={error:.6f} rad"
    )
