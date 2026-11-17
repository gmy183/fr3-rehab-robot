from types import SimpleNamespace

from rehab_robot_xhand.xhand_driver_core import (
    XHandDriverCore,
)
from rehab_robot_xhand.xhand_sdk_adapter import (
    JointCommandSpec,
    build_command,
)


class ManualClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeBackend:
    """
    Pure software backend.

    It never touches the vendor SDK, a NIC, EtherCAT, FR3 or D435i.
    """

    def __init__(self):
        self.open_calls = []
        self.sent = []
        self.closed = False
        self.read_success = True

    def open_ethercat(self, interface):
        self.open_calls.append(interface)
        return True

    def list_hands_id(self):
        return [0]

    def send_command(self, device_id, command):
        self.sent.append((device_id, command))
        return True

    def read_state(self, device_id, force_update=False):
        state = SimpleNamespace(
            finger_state=[
                SimpleNamespace(
                    id=i,
                    position=0.0,
                    torque=0,
                    temperature=0,
                )
                for i in range(12)
            ],
            sensor_data=[],
        )

        return self.read_success, state

    def close_device(self):
        self.closed = True


def active_test_command():
    specs = []

    for index in range(12):
        if index == 4:
            specs.append(
                JointCommandSpec(
                    position=0.001,
                    effort_limit=1,
                    mode=3,
                    kp=1,
                    ki=0,
                    kd=0,
                )
            )
        else:
            specs.append(
                JointCommandSpec(
                    position=0.0,
                    effort_limit=0,
                    mode=0,
                    kp=0,
                    ki=0,
                    kd=0,
                )
            )

    return build_command(specs)


def assert_powerless(command):
    assert len(command.finger_command) == 12

    for finger in command.finger_command:
        assert int(finger.mode) == 0
        assert int(finger.kp) == 0
        assert int(finger.ki) == 0
        assert int(finger.kd) == 0
        assert int(finger.tor_max) == 0


def test_constructor_does_not_open_device():
    backend = FakeBackend()
    clock = ManualClock()

    core = XHandDriverCore(
        backend=backend,
        command_timeout_s=0.5,
        disable_frames=3,
        clock=clock,
    )

    assert not core.opened
    assert backend.open_calls == []
    assert backend.sent == []


def test_open_is_explicit():
    backend = FakeBackend()
    core = XHandDriverCore(backend)

    device_id = core.open("fake0")

    assert device_id == 0
    assert core.opened
    assert backend.open_calls == ["fake0"]


def test_watchdog_trips_after_half_second_and_disables():
    backend = FakeBackend()
    clock = ManualClock()

    core = XHandDriverCore(
        backend=backend,
        command_timeout_s=0.5,
        disable_frames=3,
        clock=clock,
    )

    core.open("fake0")
    core.refresh_active_command(active_test_command())

    assert core.watchdog_armed
    assert not core.poll_watchdog()

    clock.advance(0.49)
    assert not core.poll_watchdog()

    clock.advance(0.02)
    assert core.poll_watchdog()

    assert core.watchdog_tripped
    assert not core.watchdog_armed

    # 1 active command + 3 disable frames.
    assert len(backend.sent) == 4

    for _, command in backend.sent[-3:]:
        assert_powerless(command)


def test_successful_state_read_becomes_fresh():
    backend = FakeBackend()
    clock = ManualClock()

    core = XHandDriverCore(
        backend=backend,
        clock=clock,
    )

    core.open("fake0")

    sample = core.read_fresh_state()

    assert sample is not None
    assert sample.received_monotonic_s == 100.0
    assert core.last_good_state_time == 100.0


def test_failed_read_does_not_refresh_timestamp():
    backend = FakeBackend()
    clock = ManualClock()

    core = XHandDriverCore(
        backend=backend,
        clock=clock,
    )

    core.open("fake0")

    first = core.read_fresh_state()
    assert first is not None
    assert core.last_good_state_time == 100.0

    clock.advance(10.0)
    backend.read_success = False

    failed = core.read_fresh_state()

    assert failed is None

    # Critical safety property:
    # failed read must not fabricate a new timestamp.
    assert core.last_good_state_time == 100.0


def test_wrong_joint_count_is_not_fresh():
    class ShortStateBackend(FakeBackend):
        def read_state(self, device_id, force_update=False):
            state = SimpleNamespace(
                finger_state=[
                    SimpleNamespace(id=i)
                    for i in range(11)
                ]
            )
            return True, state

    backend = ShortStateBackend()
    clock = ManualClock()
    core = XHandDriverCore(
        backend=backend,
        clock=clock,
    )

    core.open("fake0")

    assert core.read_fresh_state() is None
    assert core.last_good_state_time is None


def test_close_disables_before_closing():
    backend = FakeBackend()

    core = XHandDriverCore(
        backend=backend,
        disable_frames=3,
    )

    core.open("fake0")
    core.close()

    assert backend.closed
    assert not core.opened
    assert len(backend.sent) == 3

    for _, command in backend.sent:
        assert_powerless(command)
