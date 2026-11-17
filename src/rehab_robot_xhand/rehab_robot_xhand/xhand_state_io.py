"""Validated XHand state acquisition helpers.

This module is XHand-only:
- no FR3 dependency
- no rehabilitation-task dependency
- no EtherCAT open on import
- no command transmission
"""

from __future__ import annotations

import math

from .xhand_state_mapper import JOINT_NAMES


class XHandStateError(RuntimeError):
    """Raised when a fresh XHand hardware state is unusable."""


def read_xhand_sample(driver):
    """Read one validated 12-joint XHand sample.

    Returns:
        (positions, currents, modes)

    Notes:
        - torque from the current vendor SDK is already signed mA.
        - firmware mode is stored in the low 4 bits of raw_position.
    """

    fresh = driver.read_fresh_state(False)

    if fresh is None:
        raise XHandStateError(
            "XHand fresh state unavailable"
        )

    fingers = fresh.state.finger_state

    if len(fingers) != 12:
        raise XHandStateError(
            "XHand joint count != 12"
        )

    positions = []
    currents = []
    modes = []

    for index, finger in enumerate(fingers):
        position = float(finger.position)
        current = float(finger.torque)

        if not math.isfinite(position):
            raise XHandStateError(
                f"{JOINT_NAMES[index]} position non-finite"
            )

        if not math.isfinite(current):
            raise XHandStateError(
                f"{JOINT_NAMES[index]} current non-finite"
            )

        errors = (
            int(finger.commboard_err),
            int(finger.jonitboard_err),
            int(finger.tipboard_err),
        )

        if any(errors):
            raise XHandStateError(
                f"{JOINT_NAMES[index]} hardware errors={errors}"
            )

        positions.append(position)
        currents.append(current)

        modes.append(
            int(finger.raw_position) & 0xF
        )

    return (
        tuple(positions),
        tuple(currents),
        tuple(modes),
    )
