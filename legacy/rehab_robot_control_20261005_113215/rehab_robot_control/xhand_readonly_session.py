"""
Strict read-only XHand session.

Purpose:
    Real hardware state acquisition before any motion-control path is enabled.

Safety contract:
- construction does NOT create/open EtherCAT communication
- open() is explicit
- read_state() is allowed
- send_command() does not exist
- close() calls vendor close_device() directly
- NO powerless frame is sent on close because this session never sends
  active commands in the first place
- failed reads never become fresh states
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Optional

from .xhand_driver_core import _sdk_result_ok
from .xhand_sdk_adapter import XHAND_JOINT_COUNT


class XHandReadOnlyError(RuntimeError):
    pass


@dataclass(frozen=True)
class FreshReadOnlyXHandState:
    state: Any
    received_monotonic_s: float


class VendorXHandReadOnlyBackend:
    """
    Minimal vendor SDK wrapper.

    Deliberately exposes no send_command() method.
    Constructing it creates XHandControl, but does NOT call open_ethercat().
    """

    def __init__(self):
        from xhand_controller.xhand_control import XHandControl
        self._control = XHandControl()

    def open_ethercat(self, interface: str):
        return self._control.open_ethercat(interface)

    def list_hands_id(self):
        return self._control.list_hands_id()

    def read_state(self, device_id: int, force_update: bool = False):
        return self._control.read_state(device_id, force_update)

    def close_device(self):
        return self._control.close_device()


class XHandReadOnlySession:
    def __init__(
        self,
        backend: Any,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._backend = backend
        self._clock = clock

        self._opened = False
        self._device_id: Optional[int] = None
        self._last_good_state_time: Optional[float] = None

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def device_id(self) -> Optional[int]:
        return self._device_id

    @property
    def last_good_state_time(self) -> Optional[float]:
        return self._last_good_state_time

    def open(self, interface: str) -> int:
        if self._opened:
            raise XHandReadOnlyError(
                "read-only XHand session is already open"
            )

        if not interface:
            raise ValueError(
                "EtherCAT interface must not be empty"
            )

        result = self._backend.open_ethercat(interface)

        if not _sdk_result_ok(result):
            raise XHandReadOnlyError(
                f"open_ethercat failed: {result!r}"
            )

        hand_ids = list(self._backend.list_hands_id())

        if not hand_ids:
            try:
                self._backend.close_device()
            finally:
                self._opened = False

            raise XHandReadOnlyError(
                "no XHand device id reported"
            )

        self._device_id = int(hand_ids[0])
        self._opened = True

        return self._device_id

    def read_fresh_state(
        self,
        force_update: bool = False,
    ) -> Optional[FreshReadOnlyXHandState]:

        self._require_open()

        result = self._backend.read_state(
            self._device_id,
            force_update,
        )

        if not isinstance(result, (tuple, list)):
            return None

        if len(result) != 2:
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

        return FreshReadOnlyXHandState(
            state=state,
            received_monotonic_s=now,
        )

    def close(self) -> None:
        """
        Strictly read-only shutdown.

        No disable command is required because this session has no command
        API and therefore cannot have armed any actuator command.
        """

        if not self._opened:
            return

        try:
            self._backend.close_device()
        finally:
            self._opened = False
            self._device_id = None

    def _require_open(self) -> None:
        if not self._opened or self._device_id is None:
            raise XHandReadOnlyError(
                "read-only XHand device is not open"
            )
