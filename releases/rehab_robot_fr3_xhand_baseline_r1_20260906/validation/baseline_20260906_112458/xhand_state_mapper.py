from __future__ import annotations

from dataclasses import dataclass
from typing import Any


JOINT_NAMES = (
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

SENSOR_LOCATIONS = (
    "thumb",
    "index",
    "middle",
    "ring",
    "pinky",
)

JOINT_COUNT = 12
SENSOR_COUNT = 5
RAW_FORCE_COUNT = 120
RAW_TEMPERATURE_COUNT = 20


class XHandStateMappingError(ValueError):
    """Vendor state does not satisfy the recovered XHand state contract."""


@dataclass(frozen=True)
class MappedJointState:
    name: tuple[str, ...]
    position: tuple[float, ...]
    effort: tuple[float, ...]
    temperature: tuple[float, ...]
    error_code: tuple[int, ...]


@dataclass(frozen=True)
class MappedFingerSensorState:
    location: str
    calc_force: tuple[float, float, float]
    raw_force: tuple[tuple[float, float, float], ...]
    calc_temperature: int
    raw_temperature: tuple[int, ...]


@dataclass(frozen=True)
class MappedXHandState:
    hand_id: int
    hand_name: str
    hand_type: str
    joint_state: MappedJointState
    sensor_states: tuple[MappedFingerSensorState, ...]


def _fixed_sequence(value: Any, expected: int, label: str) -> tuple[Any, ...]:
    try:
        result = tuple(value)
    except Exception as exc:
        raise XHandStateMappingError(
            f"{label} is not a readable sequence"
        ) from exc

    if len(result) != expected:
        raise XHandStateMappingError(
            f"{label} count mismatch: expected {expected}, got {len(result)}"
        )

    return result


def _force_xyz(value: Any, label: str) -> tuple[float, float, float]:
    try:
        return (
            float(value.fx),
            float(value.fy),
            float(value.fz),
        )
    except Exception as exc:
        raise XHandStateMappingError(
            f"{label} does not contain valid fx/fy/fz"
        ) from exc


def _joint_error_code(finger: Any) -> int:
    """
    Recovered compatibility semantics:

    0 = all three vendor hardware error fields are zero
    1 = at least one vendor hardware error field is non-zero

    The exact historical non-zero packing has not been recovered, so we do
    not invent a numeric encoding. Upper safety layers only need zero/nonzero.
    """
    try:
        raw_errors = (
            int(finger.commboard_err),
            int(finger.jonitboard_err),  # vendor spelling
            int(finger.tipboard_err),
        )
    except Exception as exc:
        raise XHandStateMappingError(
            "finger state is missing hardware error fields"
        ) from exc

    return 0 if raw_errors == (0, 0, 0) else 1


def map_hand_state(
    hand_state: Any,
    *,
    hand_id: int = 0,
    hand_name: str = "xhand",
    hand_type: str = "R",
) -> MappedXHandState:
    """
    Convert one processed vendor HandState_t into the recovered ROS contract.

    Important:
    FingerState_t.torque from the currently installed SDK is already a
    processed signed current value in mA. It must NOT be decoded again as a
    raw 9-bit field.
    """

    try:
        fingers = _fixed_sequence(
            hand_state.finger_state,
            JOINT_COUNT,
            "finger_state",
        )
        sensors = _fixed_sequence(
            hand_state.sensor_data,
            SENSOR_COUNT,
            "sensor_data",
        )
    except AttributeError as exc:
        raise XHandStateMappingError(
            "hand state does not expose finger_state/sensor_data"
        ) from exc

    positions: list[float] = []
    efforts: list[float] = []
    temperatures: list[float] = []
    error_codes: list[int] = []

    for index, finger in enumerate(fingers):
        try:
            positions.append(float(finger.position))

            # DO NOT call torque_codec.decode_signed_9bit() here.
            # Current FingerState_t.torque is already signed current in mA.
            efforts.append(float(finger.torque))

            temperatures.append(float(finger.temperature))
            error_codes.append(_joint_error_code(finger))
        except XHandStateMappingError:
            raise
        except Exception as exc:
            raise XHandStateMappingError(
                f"invalid finger_state[{index}]"
            ) from exc

    mapped_sensors: list[MappedFingerSensorState] = []

    for index, sensor in enumerate(sensors):
        try:
            raw_force_objects = _fixed_sequence(
                sensor.raw_force,
                RAW_FORCE_COUNT,
                f"sensor_data[{index}].raw_force",
            )

            raw_temperature_objects = _fixed_sequence(
                sensor.temperature,
                RAW_TEMPERATURE_COUNT,
                f"sensor_data[{index}].temperature",
            )

            calc_force = _force_xyz(
                sensor.calc_force,
                f"sensor_data[{index}].calc_force",
            )

            raw_force = tuple(
                _force_xyz(
                    value,
                    f"sensor_data[{index}].raw_force[{force_index}]",
                )
                for force_index, value in enumerate(raw_force_objects)
            )

            raw_temperature = tuple(
                int(value) for value in raw_temperature_objects
            )

            mapped_sensors.append(
                MappedFingerSensorState(
                    location=SENSOR_LOCATIONS[index],
                    calc_force=calc_force,
                    raw_force=raw_force,
                    calc_temperature=int(sensor.calc_temperature),
                    raw_temperature=raw_temperature,
                )
            )
        except XHandStateMappingError:
            raise
        except Exception as exc:
            raise XHandStateMappingError(
                f"invalid sensor_data[{index}]"
            ) from exc

    return MappedXHandState(
        hand_id=int(hand_id),
        hand_name=str(hand_name),
        hand_type=str(hand_type),
        joint_state=MappedJointState(
            name=JOINT_NAMES,
            position=tuple(positions),
            effort=tuple(efforts),
            temperature=tuple(temperatures),
            error_code=tuple(error_codes),
        ),
        sensor_states=tuple(mapped_sensors),
    )
