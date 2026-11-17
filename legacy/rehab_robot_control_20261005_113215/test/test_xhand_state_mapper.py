from types import SimpleNamespace as NS

import pytest

from rehab_robot_control.xhand_state_mapper import (
    JOINT_NAMES,
    SENSOR_LOCATIONS,
    XHandStateMappingError,
    map_hand_state,
)


def force(x=0, y=0, z=0):
    return NS(fx=x, fy=y, fz=z)


def make_hand_state():
    fingers = []

    for i in range(12):
        fingers.append(
            NS(
                id=i,
                sensor_id=0,
                position=0.01 * i,

                # 300 deliberately verifies that the mapper does NOT
                # reinterpret this processed value as signed raw 9-bit.
                torque=300 if i == 0 else 70 + i,

                raw_position=0,
                temperature=30 + i,
                commboard_err=0,
                jonitboard_err=0,
                tipboard_err=0,
                default5=0,
                default6=0,
                default7=0,
            )
        )

    sensors = []

    for i in range(5):
        sensors.append(
            NS(
                calc_force=force(i + 1, -(i + 2), 10 + i),
                raw_force=[
                    force(
                        (j % 5) - 2,
                        (j % 3) - 1,
                        j % 8,
                    )
                    for j in range(120)
                ],
                temperature=[40 + i] * 20,
                calc_temperature=41 + i,
            )
        )

    return NS(
        finger_state=fingers,
        sensor_data=sensors,
    )


def test_recovered_joint_and_hand_contract():
    mapped = map_hand_state(make_hand_state())

    assert mapped.hand_id == 0
    assert mapped.hand_name == "xhand"
    assert mapped.hand_type == "R"

    assert mapped.joint_state.name == JOINT_NAMES
    assert len(mapped.joint_state.position) == 12
    assert len(mapped.joint_state.effort) == 12
    assert len(mapped.joint_state.temperature) == 12
    assert len(mapped.joint_state.error_code) == 12


def test_processed_torque_is_used_directly_as_signed_ma():
    hand = make_hand_state()

    hand.finger_state[0].torque = 300
    hand.finger_state[1].torque = -25

    mapped = map_hand_state(hand)

    assert mapped.joint_state.effort[0] == 300.0
    assert mapped.joint_state.effort[1] == -25.0


def test_raw_error_triplet_collapses_to_nonzero_sentinel():
    hand = make_hand_state()

    hand.finger_state[3].jonitboard_err = 9
    hand.finger_state[7].tipboard_err = 4

    mapped = map_hand_state(hand)

    assert mapped.joint_state.error_code[0] == 0
    assert mapped.joint_state.error_code[3] == 1
    assert mapped.joint_state.error_code[7] == 1


def test_recovered_sensor_contract():
    mapped = map_hand_state(make_hand_state())

    assert tuple(s.location for s in mapped.sensor_states) == SENSOR_LOCATIONS
    assert len(mapped.sensor_states) == 5

    index_sensor = mapped.sensor_states[1]

    assert index_sensor.location == "index"
    assert index_sensor.calc_force == (2.0, -3.0, 11.0)
    assert len(index_sensor.raw_force) == 120
    assert len(index_sensor.raw_temperature) == 20
    assert index_sensor.calc_temperature == 42


def test_wrong_joint_count_is_rejected():
    hand = make_hand_state()
    hand.finger_state = hand.finger_state[:-1]

    with pytest.raises(XHandStateMappingError, match="finger_state count mismatch"):
        map_hand_state(hand)


def test_wrong_sensor_count_is_rejected():
    hand = make_hand_state()
    hand.sensor_data = hand.sensor_data[:-1]

    with pytest.raises(XHandStateMappingError, match="sensor_data count mismatch"):
        map_hand_state(hand)


def test_wrong_raw_force_count_is_rejected():
    hand = make_hand_state()
    hand.sensor_data[2].raw_force = hand.sensor_data[2].raw_force[:-1]

    with pytest.raises(
        XHandStateMappingError,
        match=r"sensor_data\[2\]\.raw_force count mismatch",
    ):
        map_hand_state(hand)


def test_wrong_temperature_count_is_rejected():
    hand = make_hand_state()
    hand.sensor_data[4].temperature = hand.sensor_data[4].temperature[:-1]

    with pytest.raises(
        XHandStateMappingError,
        match=r"sensor_data\[4\]\.temperature count mismatch",
    ):
        map_hand_state(hand)
