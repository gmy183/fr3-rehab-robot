from types import SimpleNamespace

import pytest

from rehab_robot_xhand.xhand_readonly_session import (
    XHandReadOnlyError,
    XHandReadOnlySession,
)


class ManualClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class FakeReadOnlyBackend:
    def __init__(self):
        self.open_calls = []
        self.read_calls = 0
        self.close_calls = 0
        self.read_success = True

    def open_ethercat(self, interface):
        self.open_calls.append(interface)
        return True

    def list_hands_id(self):
        return [0]

    def read_state(self, device_id, force_update=False):
        self.read_calls += 1

        state = SimpleNamespace(
            finger_state=[
                SimpleNamespace(id=i)
                for i in range(12)
            ],
            sensor_data=[],
        )

        return self.read_success, state

    def close_device(self):
        self.close_calls += 1


def test_constructor_does_not_open():
    backend = FakeReadOnlyBackend()
    session = XHandReadOnlySession(backend)

    assert not session.opened
    assert backend.open_calls == []
    assert backend.read_calls == 0
    assert backend.close_calls == 0


def test_open_is_explicit():
    backend = FakeReadOnlyBackend()
    session = XHandReadOnlySession(backend)

    device_id = session.open("fake_readonly")

    assert device_id == 0
    assert session.opened
    assert backend.open_calls == ["fake_readonly"]


def test_successful_read_becomes_fresh():
    backend = FakeReadOnlyBackend()
    clock = ManualClock()

    session = XHandReadOnlySession(
        backend,
        clock=clock,
    )

    session.open("fake_readonly")

    sample = session.read_fresh_state()

    assert sample is not None
    assert sample.received_monotonic_s == 100.0
    assert session.last_good_state_time == 100.0


def test_failed_read_never_refreshes_timestamp():
    backend = FakeReadOnlyBackend()
    clock = ManualClock()

    session = XHandReadOnlySession(
        backend,
        clock=clock,
    )

    session.open("fake_readonly")

    assert session.read_fresh_state() is not None
    assert session.last_good_state_time == 100.0

    clock.advance(10.0)
    backend.read_success = False

    assert session.read_fresh_state() is None

    assert session.last_good_state_time == 100.0


def test_close_only_calls_close_device():
    backend = FakeReadOnlyBackend()
    session = XHandReadOnlySession(backend)

    session.open("fake_readonly")
    session.close()

    assert backend.close_calls == 1
    assert not session.opened


def test_backend_contract_contains_no_send_command():
    backend = FakeReadOnlyBackend()

    assert not hasattr(
        backend,
        "send_command",
    )


def test_read_before_open_is_rejected():
    backend = FakeReadOnlyBackend()
    session = XHandReadOnlySession(backend)

    with pytest.raises(
        XHandReadOnlyError,
        match="not open",
    ):
        session.read_fresh_state()
