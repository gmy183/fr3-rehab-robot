"""
XHand vendor-SDK adapter for rehab_robot_ws.

IMPORTANT:
- This module does NOT open EtherCAT at import time.
- This module does NOT send commands at import time.
- The self-test is software-only.
- Active motion gains/targets are deliberately NOT invented here.

Recovered safety contract from the 2026-07-22 reviewed project:
- XHand command array length: 12
- inactive joint:
    mode = 0
    kp = ki = kd = 0
    tor_max = 0
- command watchdog belongs to the ROS2 driver layer
- hardware read failure must not be converted into a fresh valid state
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

from xhand_controller.xhand_control import (
    FingerCommand_t,
    HandCommand_t,
    XHandControl,
)


XHAND_JOINT_COUNT = 12
POWERLESS_MODE = 0


class XHandAdapterError(RuntimeError):
    """Raised when the vendor SDK does not satisfy the expected contract."""


@dataclass(frozen=True)
class JointCommandSpec:
    position: float
    effort_limit: int
    mode: int
    kp: int
    ki: int
    kd: int


def _finger_commands(command: HandCommand_t):
    fingers = command.finger_command

    if len(fingers) != XHAND_JOINT_COUNT:
        raise XHandAdapterError(
            f"vendor HandCommand_t contains {len(fingers)} joints; "
            f"expected {XHAND_JOINT_COUNT}"
        )

    return fingers


def make_powerless_command() -> HandCommand_t:
    """
    Construct the reviewed safe disabled command.

    This function only constructs an in-memory SDK object.
    It does NOT call send_command().
    """
    command = HandCommand_t()
    fingers = _finger_commands(command)

    for index, finger in enumerate(fingers):
        finger.id = index
        finger.position = 0.0
        finger.kp = 0
        finger.ki = 0
        finger.kd = 0
        finger.tor_max = 0
        finger.mode = POWERLESS_MODE
        finger.res0 = 0
        finger.res1 = 0
        finger.res2 = 0
        finger.res3 = 0

    return command


def build_command(specs: Sequence[JointCommandSpec]) -> HandCommand_t:
    """
    Translate already-validated project-level command specifications to
    the vendor SDK representation.

    No default active gains are supplied intentionally.  Active parameters
    must come from the recovered project configuration, not from guesses.

    Safety rule:
      effort_limit <= 0  -> force POWERLESS_MODE and zero gains/current.
    """
    if len(specs) != XHAND_JOINT_COUNT:
        raise XHandAdapterError(
            f"command specification length={len(specs)}, "
            f"expected={XHAND_JOINT_COUNT}"
        )

    command = HandCommand_t()
    fingers = _finger_commands(command)

    for index, (finger, spec) in enumerate(zip(fingers, specs)):
        finger.id = index
        finger.position = float(spec.position)

        if int(spec.effort_limit) <= 0:
            finger.kp = 0
            finger.ki = 0
            finger.kd = 0
            finger.tor_max = 0
            finger.mode = POWERLESS_MODE
        else:
            if int(spec.mode) == POWERLESS_MODE:
                raise XHandAdapterError(
                    f"joint {index}: positive effort_limit with mode=0"
                )

            finger.kp = int(spec.kp)
            finger.ki = int(spec.ki)
            finger.kd = int(spec.kd)
            finger.tor_max = int(spec.effort_limit)
            finger.mode = int(spec.mode)

        finger.res0 = 0
        finger.res1 = 0
        finger.res2 = 0
        finger.res3 = 0

    return command


def verify_vendor_api() -> None:
    required_control_methods = (
        "open_ethercat",
        "read_state",
        "send_command",
        "close_device",
        "list_hands_id",
    )

    missing_methods = [
        name for name in required_control_methods
        if not hasattr(XHandControl, name)
    ]

    if missing_methods:
        raise XHandAdapterError(
            "vendor XHandControl missing methods: "
            + ", ".join(missing_methods)
        )

    finger = FingerCommand_t()

    required_fields = (
        "id",
        "position",
        "kp",
        "ki",
        "kd",
        "tor_max",
        "mode",
        "res0",
        "res1",
        "res2",
        "res3",
    )

    missing_fields = [
        name for name in required_fields
        if not hasattr(finger, name)
    ]

    if missing_fields:
        raise XHandAdapterError(
            "vendor FingerCommand_t missing fields: "
            + ", ".join(missing_fields)
        )

    command = make_powerless_command()
    fingers = _finger_commands(command)

    for index, finger in enumerate(fingers):
        if int(finger.mode) != 0:
            raise XHandAdapterError(
                f"joint {index}: powerless mode is not zero"
            )

        if any(
            int(value) != 0
            for value in (
                finger.kp,
                finger.ki,
                finger.kd,
                finger.tor_max,
            )
        ):
            raise XHandAdapterError(
                f"joint {index}: powerless command contains "
                "non-zero gain/current"
            )


def main() -> None:
    verify_vendor_api()

    command = make_powerless_command()

    print("============================================================")
    print("XHand SDK adapter software-only self-test")
    print("============================================================")
    print(f"[PASS] vendor API")
    print(f"[PASS] HandCommand_t joints = {len(command.finger_command)}")
    print("[PASS] all 12 inactive joints:")
    print("       mode=0")
    print("       kp=ki=kd=0")
    print("       tor_max=0")
    print()
    print("[PASS] no EtherCAT interface opened")
    print("[PASS] no XHand command sent")
    print("[PASS] no FR3 command sent")
    print("============================================================")


if __name__ == "__main__":
    main()
