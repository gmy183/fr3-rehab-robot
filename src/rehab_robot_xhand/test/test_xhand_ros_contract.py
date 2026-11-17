from rehab_robot_xhand.xhand_ros2_node import XHAND_JOINT_NAMES
from rehab_robot_interfaces.msg import (
    XHandFingerSensorState,
    XHandJointState,
    XHandSensorState,
    XHandState,
)


EXPECTED_JOINTS = (
    "thumb_bend_joint",
    "thumb_rota_joint1",
    "thumb_rota_joint2",
    "index_bend_joint",
    "index_joint1",
    "index_joint2",
    "mid_joint1",
    "mid_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
)


def test_joint_contract():
    assert XHAND_JOINT_NAMES == EXPECTED_JOINTS
    assert len(XHAND_JOINT_NAMES) == 12


def test_state_message_contract():
    msg = XHandState()

    assert hasattr(msg, "header")
    assert hasattr(msg, "hand_id")
    assert hasattr(msg, "hand_name")
    assert hasattr(msg, "hand_type")
    assert hasattr(msg, "hand_states")
    assert hasattr(msg, "sensor_states")


def test_nested_message_contract():
    joint = XHandJointState()
    sensor = XHandSensorState()
    finger = XHandFingerSensorState()

    for name in (
        "name",
        "position",
        "effort",
        "temperature",
        "error_code",
    ):
        assert hasattr(joint, name)

    assert hasattr(sensor, "finger_sensor_states")

    for name in (
        "location",
        "calc_force",
        "raw_force",
        "calc_temperature",
        "raw_temperature",
    ):
        assert hasattr(finger, name)
