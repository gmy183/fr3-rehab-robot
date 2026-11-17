"""
Safety core for the reconstructed XHand driver.

This module contains NO ROS message definitions.

Safety properties:
- construction does not open EtherCAT
- hardware access requires explicit open()
- 0.5 s project-level command watchdog
- watchdog/close sends multiple powerless frames
- failed read_state() never produces a fresh state
- wrong joint count never produces a fresh state
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Optional

from .xhand_sdk_adapter import (
    XHAND_JOINT_COUNT,
    make_powerless_command,
)


DEFAULT_COMMAND_TIMEOUT_S = 0.5
DEFAULT_DISABLE_FRAMES = 3


class XHandDriverError(RuntimeError):
    pass


@dataclass(frozen=True)
class FreshXHandState:
    state: Any
    received_monotonic_s: float


def _sdk_result_ok(result: Any) -> bool:
    """
    Interpret XHand SDK ErrorStruct/bool-style return values.

    Vendor examples use the return object directly in boolean context.
    Keep fallback handling for simple fake backends used in tests.
    """
    if isinstance(result, bool):
        return result

    try:
        return bool(result)
    except Exception:
        pass

    error_code = getattr(result, "error_code", None)
    if error_code is not None:
        try:
            return int(error_code) == 0
        except (TypeError, ValueError):
            return False

    return False


def _command_is_powerless(command: Any) -> bool:
    fingers = command.finger_command

    if len(fingers) != XHAND_JOINT_COUNT:
        raise XHandDriverError(
            f"command joint count={len(fingers)}, "
            f"expected={XHAND_JOINT_COUNT}"
        )

    for finger in fingers:
        if int(finger.mode) != 0:
            return False
        if any(
            int(value) != 0
            for value in (
                finger.kp,
                finger.ki,
                finger.kd,
                finger.tor_max,
            )
        ):
            return False

    return True


class VendorXHandBackend:
    """
    Thin wrapper around the installed vendor SDK.

    Merely constructing this object does not call open_ethercat().
    """

    def __init__(self):
        from xhand_controller.xhand_control import XHandControl
        self._control = XHandControl()

    def open_ethercat(self, interface: str):
        return self._control.open_ethercat(interface)

    def list_hands_id(self):
        return self._control.list_hands_id()

    def send_command(self, device_id: int, command: Any):
        return self._control.send_command(device_id, command)

    def read_state(self, device_id: int, force_update: bool = False):
        return self._control.read_state(device_id, force_update)

    def close_device(self):
        return self._control.close_device()


class XHandDriverCore:
    def __init__(
        self,
        backend: Any,
        command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        disable_frames: int = DEFAULT_DISABLE_FRAMES,
        clock: Callable[[], float] = time.monotonic,
    ):
        if command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be > 0")

        if disable_frames < 1:
            raise ValueError("disable_frames must be >= 1")

        self._backend = backend
        self._command_timeout_s = float(command_timeout_s)
        self._disable_frames = int(disable_frames)
        self._clock = clock

        self._opened = False
        self._device_id: Optional[int] = None

        self._last_command_time: Optional[float] = None
        self._last_good_state_time: Optional[float] = None

        self._watchdog_armed = False
        self._watchdog_tripped = False

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def device_id(self) -> Optional[int]:
        return self._device_id

    @property
    def last_good_state_time(self) -> Optional[float]:
        return self._last_good_state_time

    @property
    def watchdog_armed(self) -> bool:
        return self._watchdog_armed

    @property
    def watchdog_tripped(self) -> bool:
        return self._watchdog_tripped

    def open(self, interface: str) -> int:
        if self._opened:
            raise XHandDriverError("XHand driver is already open")

        if not interface:
            raise ValueError("EtherCAT interface must not be empty")

        result = self._backend.open_ethercat(interface)

        if not _sdk_result_ok(result):
            raise XHandDriverError(
                f"open_ethercat failed: {result!r}"
            )

        hand_ids = list(self._backend.list_hands_id())

        if not hand_ids:
            try:
                self._backend.close_device()
            finally:
                self._opened = False
            raise XHandDriverError("no XHand device id reported")

        self._device_id = int(hand_ids[0])
        self._opened = True
        self._watchdog_armed = False
        self._watchdog_tripped = False
        self._last_command_time = None

        return self._device_id

    def send_command(self, command: Any) -> None:
        self._require_open()

        powerless = _command_is_powerless(command)

        result = self._backend.send_command(
            self._device_id,
            command,
        )

        if not _sdk_result_ok(result):
            raise XHandDriverError(
                f"send_command failed: {result!r}"
            )

        now = self._clock()

        if powerless:
            self._watchdog_armed = False
            self._watchdog_tripped = False
            self._last_command_time = None
        else:
            self._last_command_time = now
            self._watchdog_armed = True
            self._watchdog_tripped = False

    def refresh_active_command(self, command: Any) -> None:
        if _command_is_powerless(command):
            raise XHandDriverError(
                "refresh_active_command received powerless command"
            )

        self.send_command(command)

    def read_fresh_state(
        self,
        force_update: bool = False,
    ) -> Optional[FreshXHandState]:
        """
        Return a state only when this read cycle is successful.

        A failed SDK read returns None and DOES NOT update
        last_good_state_time.
        """
        self._require_open()

        result = self._backend.read_state(
            self._device_id,
            force_update,
        )

        if not isinstance(result, (tuple, list)) or len(result) != 2:
            return None

        error, state = result

        if not _sdk_result_ok(error):
            return None

        fingers = getattr(state, "finger_state", None)

        if fingers is None:
            return None

        try:
            count = len(fingers)
        except TypeError:
            return None

        if count != XHAND_JOINT_COUNT:
            return None

        now = self._clock()
        self._last_good_state_time = now

        return FreshXHandState(
            state=state,
            received_monotonic_s=now,
        )

    def poll_watchdog(self) -> bool:
        """
        Check the project-level command watchdog.

        Returns True only on the cycle that actually trips it.
        """
        if not self._opened:
            return False

        if not self._watchdog_armed:
            return False

        if self._last_command_time is None:
            return False

        elapsed = self._clock() - self._last_command_time

        if elapsed <= self._command_timeout_s:
            return False

        if self._watchdog_tripped:
            return False

        self.disable_multiframe()

        self._watchdog_tripped = True
        self._watchdog_armed = False
        self._last_command_time = None

        return True

    def disable_multiframe(self) -> None:
        """
        Send several independent powerless frames.

        This follows the reviewed project requirement that watchdog,
        normal exit, protection exit and Ctrl+C must not rely on one
        single disable packet.
        """
        self._require_open()

        failures = []

        for index in range(self._disable_frames):
            command = make_powerless_command()

            try:
                result = self._backend.send_command(
                    self._device_id,
                    command,
                )

                if not _sdk_result_ok(result):
                    failures.append(
                        f"frame {index + 1}: {result!r}"
                    )
            except Exception as exc:
                failures.append(
                    f"frame {index + 1}: {exc}"
                )

        self._watchdog_armed = False
        self._last_command_time = None

        if failures:
            raise XHandDriverError(
                "powerless disable incomplete: "
                + "; ".join(failures)
            )

    def close(self) -> None:
        """
        Deterministic shutdown order:
          multiple powerless frames -> close vendor device
        """
        if not self._opened:
            return

        disable_error = None

        try:
            self.disable_multiframe()
        except Exception as exc:
            disable_error = exc

        try:
            self._backend.close_device()
        finally:
            self._opened = False
            self._device_id = None
            self._watchdog_armed = False
            self._last_command_time = None

        if disable_error is not None:
            raise disable_error

    def _require_open(self) -> None:
        if not self._opened or self._device_id is None:
            raise XHandDriverError(
                "XHand device is not open"
            )
