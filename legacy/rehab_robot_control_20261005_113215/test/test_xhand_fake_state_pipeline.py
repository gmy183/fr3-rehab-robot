from rehab_robot_control.xhand_driver_core import (
    XHandDriverCore,
)
from rehab_robot_control.xhand_fake_backend import (
    SoftwareFakeXHandBackend,
)
from rehab_robot_control.xhand_ros_state_bridge import (
    mapped_state_to_ros,
)
from rehab_robot_control.xhand_state_mapper import (
    JOINT_NAMES,
    SENSOR_LOCATIONS,
    map_hand_state,
)


def test_fake_backend_read_path_uses_driver_core():
    backend = SoftwareFakeXHandBackend()

    core = XHandDriverCore(
        backend=backend,
        command_timeout_s=0.5,
    )

    device_id = core.open("software_fake")

    assert device_id == 0
    assert backend.opened

    sample = core.read_fresh_state()

    assert sample is not None
    assert backend.read_count == 1
    assert backend.send_command_count == 0


def test_fake_driver_state_maps_to_ros_message():
    backend = SoftwareFakeXHandBackend()
    core = XHandDriverCore(backend=backend)

    core.open("software_fake")

    sample = core.read_fresh_state()
    assert sample is not None

    mapped = map_hand_state(
        sample.state,
        hand_id=core.device_id or 0,
        hand_name="xhand",
        hand_type="R",
    )

    msg = mapped_state_to_ros(mapped)

    assert list(msg.hand_id) == [0]
    assert list(msg.hand_name) == ["xhand"]
    assert list(msg.hand_type) == ["R"]

    assert len(msg.hand_states) == 1
    assert msg.hand_states[0].name == list(JOINT_NAMES)

    assert len(msg.hand_states[0].position) == 12
    assert len(msg.hand_states[0].effort) == 12

    assert msg.hand_states[0].effort[0] == -20.0

    assert len(msg.sensor_states) == 1

    sensors = msg.sensor_states[0].finger_sensor_states

    assert len(sensors) == 5
    assert tuple(x.location for x in sensors) == SENSOR_LOCATIONS

    assert sensors[0].calc_force.x == 1.0
    assert sensors[0].calc_force.y == -1.0
    assert sensors[0].calc_force.z == 10.0

    assert backend.send_command_count == 0


def test_fake_backend_explicitly_blocks_commands():
    backend = SoftwareFakeXHandBackend()
    backend.open_ethercat("software_fake")

    try:
        backend.send_command(0, object())
    except RuntimeError as exc:
        assert "forbids send_command" in str(exc)
    else:
        raise AssertionError(
            "fake backend unexpectedly accepted a command"
        )
