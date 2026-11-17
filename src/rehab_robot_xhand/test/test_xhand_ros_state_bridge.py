from builtin_interfaces.msg import Time
import pytest
from xhand_controller.xhand_control import HandState_t

from rehab_robot_xhand.xhand_ros_state_bridge import (
    mapped_state_to_ros,
)
from rehab_robot_xhand.xhand_state_mapper import (
    JOINT_NAMES,
    SENSOR_LOCATIONS,
    map_hand_state,
)


def make_vendor_state():
    """
    Construct the real installed SDK HandState_t in memory only.

    This does NOT instantiate XHandControl and does NOT open EtherCAT.
    """
    state = HandState_t()

    state.finger_state[0].position = 0.125
    state.finger_state[0].torque = -25
    state.finger_state[0].temperature = 31

    state.finger_state[4].position = 0.005
    state.finger_state[4].torque = 72
    state.finger_state[4].temperature = 33

    state.sensor_data[0].calc_force.fx = -2
    state.sensor_data[0].calc_force.fy = 3
    state.sensor_data[0].calc_force.fz = 12
    state.sensor_data[0].calc_temperature = 41

    return state


def test_real_sdk_object_maps_to_recovered_ros_contract():
    vendor = make_vendor_state()

    mapped = map_hand_state(vendor)
    ros = mapped_state_to_ros(mapped)

    assert list(ros.hand_id) == [0]
    assert ros.hand_name == ["xhand"]
    assert ros.hand_type == ["R"]

    assert len(ros.hand_states) == 1
    assert ros.hand_states[0].name == list(JOINT_NAMES)

    assert len(ros.hand_states[0].position) == 12
    assert len(ros.hand_states[0].effort) == 12
    assert len(ros.hand_states[0].temperature) == 12
    assert len(ros.hand_states[0].error_code) == 12


def test_processed_sdk_values_reach_ros_without_reencoding():
    vendor = make_vendor_state()

    ros = mapped_state_to_ros(
        map_hand_state(vendor)
    )

    joints = ros.hand_states[0]

    assert joints.position[0] == 0.125
    assert joints.effort[0] == -25.0
    assert joints.temperature[0] == 31.0

    assert joints.position[4] == pytest.approx(0.005, abs=1e-7)
    assert joints.effort[4] == 72.0
    assert joints.temperature[4] == 33.0


def test_five_sensor_blocks_match_historical_locations():
    vendor = make_vendor_state()

    ros = mapped_state_to_ros(
        map_hand_state(vendor)
    )

    assert len(ros.sensor_states) == 1

    fingers = ros.sensor_states[0].finger_sensor_states

    assert len(fingers) == 5

    assert tuple(
        sensor.location for sensor in fingers
    ) == SENSOR_LOCATIONS

    thumb = fingers[0]

    assert thumb.calc_force.x == -2.0
    assert thumb.calc_force.y == 3.0
    assert thumb.calc_force.z == 12.0

    assert thumb.calc_temperature == 41

    assert len(thumb.raw_force) == 120
    assert len(thumb.raw_temperature) == 20


def test_header_stamp_is_preserved():
    vendor = make_vendor_state()

    stamp = Time()
    stamp.sec = 123
    stamp.nanosec = 456

    ros = mapped_state_to_ros(
        map_hand_state(vendor),
        stamp=stamp,
        frame_id="",
    )

    assert ros.header.stamp.sec == 123
    assert ros.header.stamp.nanosec == 456
    assert ros.header.frame_id == ""
