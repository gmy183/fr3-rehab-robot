from __future__ import annotations

import os
import time

from rehab_robot_control.xhand_readonly_session import (
    VendorXHandReadOnlyBackend,
    XHandReadOnlySession,
)
from rehab_robot_control.xhand_state_mapper import (
    map_hand_state,
)
from rehab_robot_control.xhand_ros_state_bridge import (
    mapped_state_to_ros,
)


interface = os.environ["XHAND_IF"]

print("=" * 60, flush=True)
print("XHand project read-only pipeline one-shot", flush=True)
print("=" * 60, flush=True)

backend = VendorXHandReadOnlyBackend()

print("[1] backend created", flush=True)
print(
    "[PASS] backend has send_command =",
    hasattr(backend, "send_command"),
    flush=True,
)

session = XHandReadOnlySession(
    backend=backend,
)

print("[2] BEFORE session.open()", flush=True)

device_id = session.open(interface)

print(
    f"[3] AFTER session.open(): device_id={device_id}",
    flush=True,
)

print("[4] wait 1.0 s for EtherCAT PDO", flush=True)
time.sleep(1.0)


print("[5] BEFORE session.read_fresh_state()", flush=True)

t0 = time.monotonic()

sample = session.read_fresh_state(
    force_update=False,
)

elapsed = time.monotonic() - t0

print(
    f"[6] AFTER session.read_fresh_state(): "
    f"elapsed={elapsed:.6f}s",
    flush=True,
)


if sample is None:
    print("[FAIL] session returned None", flush=True)
    session.close()
    raise SystemExit(1)

print(
    "[PASS] session returned fresh state",
    flush=True,
)


print("[7] BEFORE map_hand_state()", flush=True)

mapped = map_hand_state(
    sample.state,
    hand_id=device_id,
    hand_name="xhand",
    hand_type="R",
)

print("[8] AFTER map_hand_state()", flush=True)

print(
    "mapped joint count =",
    len(mapped.joint_state.name),
    flush=True,
)

print(
    "mapped positions =",
    [
        round(float(x), 6)
        for x in mapped.joint_state.position
    ],
    flush=True,
)

print(
    "mapped effort(mA) =",
    [
        int(x)
        for x in mapped.joint_state.effort
    ],
    flush=True,
)

print(
    "mapped error_code =",
    list(mapped.joint_state.error_code),
    flush=True,
)

print(
    "mapped sensors =",
    len(mapped.sensor_state.finger_sensor_states),
    flush=True,
)


print("[9] BEFORE mapped_state_to_ros()", flush=True)

msg = mapped_state_to_ros(
    mapped,
    frame_id="",
)

print("[10] AFTER mapped_state_to_ros()", flush=True)

print("ROS hand_id =", list(msg.hand_id), flush=True)
print("ROS hand_name =", list(msg.hand_name), flush=True)
print("ROS hand_type =", list(msg.hand_type), flush=True)

print(
    "ROS hand_states =",
    len(msg.hand_states),
    flush=True,
)

print(
    "ROS sensor_states =",
    len(msg.sensor_states),
    flush=True,
)


if len(msg.hand_states) != 1:
    raise RuntimeError(
        "expected one ROS hand_states block"
    )

hand = msg.hand_states[0]

print(
    "ROS joint_count =",
    len(hand.name),
    flush=True,
)

print(
    "ROS names =",
    list(hand.name),
    flush=True,
)

print(
    "ROS position =",
    [
        round(float(x), 6)
        for x in hand.position
    ],
    flush=True,
)

print(
    "ROS effort =",
    [
        int(x)
        for x in hand.effort
    ],
    flush=True,
)

print(
    "ROS error_code =",
    list(hand.error_code),
    flush=True,
)


if len(msg.sensor_states) != 1:
    raise RuntimeError(
        "expected one ROS sensor_states block"
    )

sensors = (
    msg.sensor_states[0]
    .finger_sensor_states
)

print(
    "ROS finger sensors =",
    len(sensors),
    flush=True,
)

print(
    "ROS locations =",
    [x.location for x in sensors],
    flush=True,
)

print(
    "ROS raw force lengths =",
    [len(x.raw_force) for x in sensors],
    flush=True,
)

print(
    "ROS raw temp lengths =",
    [len(x.raw_temperature) for x in sensors],
    flush=True,
)


assert len(hand.name) == 12
assert len(sensors) == 5

assert all(
    len(x.raw_force) == 120
    for x in sensors
)

assert all(
    len(x.raw_temperature) == 20
    for x in sensors
)


print("[11] BEFORE session.close()", flush=True)

session.close()

print("[12] AFTER session.close()", flush=True)

print()
print(
    "[PASS] REAL XHand project state pipeline",
    flush=True,
)
print(
    "[PASS] SDK -> ReadOnlySession -> mapper -> ROS message",
    flush=True,
)
print(
    "[PASS] no send_command API used",
    flush=True,
)
