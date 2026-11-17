"""
Convert reconstructed XHand mapped state into the recovered ROS 2 state message.

Pure software layer:
- no EtherCAT
- no read_state()
- no send_command()
"""

from __future__ import annotations

from typing import Optional

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3

from rehab_robot_interfaces.msg import (
    XHandFingerSensorState,
    XHandJointState,
    XHandSensorState,
    XHandState,
)

from .xhand_state_mapper import MappedXHandState


def _vector3(xyz) -> Vector3:
    msg = Vector3()
    msg.x = float(xyz[0])
    msg.y = float(xyz[1])
    msg.z = float(xyz[2])
    return msg


def mapped_state_to_ros(
    state: MappedXHandState,
    *,
    stamp: Optional[Time] = None,
    frame_id: str = "",
) -> XHandState:
    """
    Build one recovered XHandState ROS message.

    Historical contract:
      hand_id      -> one-element array
      hand_name    -> one-element array
      hand_type    -> one-element array
      hand_states  -> one joint-state block
      sensor_states-> one sensor block containing five fingers
    """

    msg = XHandState()

    if stamp is not None:
        msg.header.stamp = stamp

    msg.header.frame_id = str(frame_id)

    msg.hand_id = [int(state.hand_id)]
    msg.hand_name = [str(state.hand_name)]
    msg.hand_type = [str(state.hand_type)]

    joints = XHandJointState()

    joints.name = list(state.joint_state.name)
    joints.position = [
        float(v) for v in state.joint_state.position
    ]
    joints.effort = [
        float(v) for v in state.joint_state.effort
    ]
    joints.temperature = [
        float(v) for v in state.joint_state.temperature
    ]
    joints.error_code = [
        int(v) for v in state.joint_state.error_code
    ]

    msg.hand_states = [joints]

    sensor_group = XHandSensorState()
    finger_messages = []

    for sensor in state.sensor_states:
        finger = XHandFingerSensorState()

        finger.location = str(sensor.location)
        finger.calc_force = _vector3(sensor.calc_force)

        finger.raw_force = [
            _vector3(value)
            for value in sensor.raw_force
        ]

        finger.calc_temperature = int(
            sensor.calc_temperature
        )

        finger.raw_temperature = [
            int(value)
            for value in sensor.raw_temperature
        ]

        finger_messages.append(finger)

    sensor_group.finger_sensor_states = finger_messages
    msg.sensor_states = [sensor_group]

    return msg
