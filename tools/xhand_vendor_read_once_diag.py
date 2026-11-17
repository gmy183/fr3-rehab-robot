from __future__ import annotations

import os
import time

from xhand_controller.xhand_control import XHandControl


interface = os.environ["XHAND_IF"]

print("============================================================", flush=True)
print("Direct XHand vendor read_state diagnostic", flush=True)
print("============================================================", flush=True)

print(f"[1] interface={interface}", flush=True)

control = XHandControl()

print("[2] BEFORE open_ethercat()", flush=True)

open_result = control.open_ethercat(interface)

print("[3] AFTER open_ethercat()", flush=True)
print(
    "    error_code=",
    getattr(open_result, "error_code", None),
    flush=True,
)
print(
    "    error_message=",
    repr(getattr(open_result, "error_message", None)),
    flush=True,
)

if getattr(open_result, "error_code", 0) != 0:
    print("[FAIL] open_ethercat reported error", flush=True)
    raise SystemExit(2)


hand_ids = list(control.list_hands_id())

print("[4] hand_ids =", hand_ids, flush=True)

if not hand_ids:
    print("[FAIL] no XHand device id", flush=True)
    control.close_device()
    raise SystemExit(3)


device_id = int(hand_ids[0])

print("[5] wait 1.0 s for PDO update thread", flush=True)

time.sleep(1.0)


print(
    f"[6] BEFORE read_state(device_id={device_id}, force_update=False)",
    flush=True,
)

t0 = time.monotonic()

result = control.read_state(
    device_id,
    False,
)

elapsed = time.monotonic() - t0

print(
    f"[7] AFTER read_state(), elapsed={elapsed:.6f} s",
    flush=True,
)


if not isinstance(result, tuple) or len(result) != 2:
    print(
        "[FAIL] unexpected read_state return:",
        type(result),
        repr(result),
        flush=True,
    )
    control.close_device()
    raise SystemExit(4)


error, state = result

print(
    "    error_code=",
    getattr(error, "error_code", None),
    flush=True,
)

print(
    "    error_message=",
    repr(getattr(error, "error_message", None)),
    flush=True,
)


fingers = state.finger_state
sensors = state.sensor_data

print("[8] finger_count =", len(fingers), flush=True)
print("[9] sensor_count =", len(sensors), flush=True)

print(
    "[10] positions =",
    [
        round(float(x.position), 6)
        for x in fingers
    ],
    flush=True,
)

print(
    "[11] torque_mA =",
    [
        int(x.torque)
        for x in fingers
    ],
    flush=True,
)

print(
    "[12] joint_errors =",
    [
        (
            int(x.commboard_err),
            int(x.jonitboard_err),
            int(x.tipboard_err),
        )
        for x in fingers
    ],
    flush=True,
)

print("[13] BEFORE close_device()", flush=True)

control.close_device()

print("[14] AFTER close_device()", flush=True)
print("[PASS] direct vendor read_state returned", flush=True)
