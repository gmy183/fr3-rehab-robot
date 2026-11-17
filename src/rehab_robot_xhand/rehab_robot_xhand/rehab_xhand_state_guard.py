from __future__ import annotations

import time

from .xhand_state_io import (
    XHandStateError,
    read_xhand_sample,
)
from .xhand_state_mapper import JOINT_NAMES as XHAND_JOINT_NAMES

# Compatibility name retained for the already-validated guard logic.
# It now refers to an XHand-layer exception rather than a task-layer one.
RuntimeSafetyError = XHandStateError


ZERO_EPS = 1e-12

# This physical XHand has several joints with clearly non-zero
# calibrated positions.  A complete 12-joint collapse to exact
# zeros while an active command is expected is therefore not
# accepted as a physical hand state.
REFERENCE_SIGNIFICANT_RAD = 0.020

MAX_CONSECUTIVE_ZERO_FRAMES = 2
DEFAULT_RETRY_COUNT = 2
DEFAULT_RETRY_SLEEP_S = 0.002


def _validate_lengths(
    sample,
):
    pos, cur, modes = sample

    if (
        len(pos) != 12
        or len(cur) != 12
        or len(modes) != 12
    ):
        raise RuntimeSafetyError(
            "XHand guarded sample length != 12"
        )

    return pos, cur, modes


def is_zero_collapse_frame(
    sample,
    *,
    reference,
):
    """
    Detect the specific transient observed on the real device:

        positions = all 0
        currents  = all 0
        modes     = all 0

    It is classified as a communication/state-collapse frame only
    when the expected/reference hand state is clearly non-zero.
    """

    pos, cur, modes = _validate_lengths(
        sample
    )

    reference = tuple(
        float(v)
        for v in reference
    )

    if len(reference) != 12:
        raise RuntimeSafetyError(
            "XHand reference length != 12"
        )

    reference_is_significant = any(
        abs(v) >= REFERENCE_SIGNIFICANT_RAD
        for v in reference
    )

    if not reference_is_significant:
        return False

    positions_zero = all(
        abs(float(v)) <= ZERO_EPS
        for v in pos
    )

    currents_zero = all(
        abs(float(v)) <= ZERO_EPS
        for v in cur
    )

    modes_zero = all(
        int(v) == 0
        for v in modes
    )

    return (
        positions_zero
        and currents_zero
        and modes_zero
    )


def require_active_modes(
    sample,
    *,
    active_joints,
):
    """
    During an already-armed XHand hold used while FR3 is moving,
    every active joint must remain in mode 3.

    This is intentionally stricter than takeover, where a short
    mode0 -> mode3 transition is allowed.
    """

    _, _, modes = _validate_lengths(
        sample
    )

    bad = []

    for name in active_joints:

        index = XHAND_JOINT_NAMES.index(
            name
        )

        mode = int(
            modes[index]
        )

        if mode != 3:
            bad.append(
                f"{name}={mode}"
            )

    if bad:
        raise RuntimeSafetyError(
            "XHand active mode dropped: "
            + ", ".join(bad)
        )


def read_guarded_xhand_sample(
    node,
    driver,
    *,
    reference,
    active_expected=True,
    retry_count=DEFAULT_RETRY_COUNT,
    retry_sleep_s=DEFAULT_RETRY_SLEEP_S,
    max_consecutive_zero_frames=(
        MAX_CONSECUTIVE_ZERO_FRAMES
    ),
):
    """
    Read a genuinely usable XHand sample.

    A transient complete-zero frame is NOT passed into the motion
    guard.  It is dropped and immediately re-read.

    Safety rule:
      * 1-2 consecutive zero-collapse frames -> retry
      * third consecutive zero-collapse frame -> hard fault

    last_good_xhand_sample is retained only for diagnostics.
    It is never substituted as if it were a new hardware sample.
    """

    attempts = int(
        retry_count
    ) + 1

    if attempts <= 0:
        raise RuntimeSafetyError(
            "invalid XHand guarded retry count"
        )

    for attempt in range(
        attempts
    ):
        sample = read_xhand_sample(
            driver
        )

        collapse = (
            active_expected
            and is_zero_collapse_frame(
                sample,
                reference=reference,
            )
        )

        if not collapse:

            node.xhand_zero_frame_consecutive = 0

            node.last_good_xhand_sample = sample

            return sample

        total = int(
            getattr(
                node,
                "xhand_zero_frame_total",
                0,
            )
        ) + 1

        consecutive = int(
            getattr(
                node,
                "xhand_zero_frame_consecutive",
                0,
            )
        ) + 1

        node.xhand_zero_frame_total = total
        node.xhand_zero_frame_consecutive = consecutive

        print(
            "[WARN] XHand transient zero-state "
            f"frame dropped: total={total}, "
            f"consecutive={consecutive}"
        )

        if (
            consecutive
            > int(
                max_consecutive_zero_frames
            )
        ):
            raise RuntimeSafetyError(
                "XHand consecutive zero-state "
                f"frames exceeded: {consecutive}"
            )

        if attempt + 1 < attempts:
            time.sleep(
                float(
                    retry_sleep_s
                )
            )

    raise RuntimeSafetyError(
        "XHand zero-state retry budget exhausted"
    )
