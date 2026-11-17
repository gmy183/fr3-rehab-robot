#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

CONTROL="$HOME/rehab_robot_ws/src/rehab_robot_control"

echo "============================================================"
echo "XHand SDK state -> ROS message software verification"
echo "============================================================"

python3 -m py_compile \
  "$CONTROL/rehab_robot_control/xhand_state_mapper.py" \
  "$CONTROL/rehab_robot_control/xhand_ros_state_bridge.py" \
  "$CONTROL/test/test_xhand_ros_state_bridge.py"

echo "[PASS] Python syntax"

echo
echo "========== regression pytest =========="

python3 -m pytest -q \
  "$CONTROL/test/test_xhand_driver_core.py" \
  "$CONTROL/test/test_xhand_ros_contract.py" \
  "$CONTROL/test/test_xhand_state_mapper.py" \
  "$CONTROL/test/test_xhand_ros_state_bridge.py"

echo
echo "========== real SDK data object =========="

python3 - <<'PY'
from xhand_controller.xhand_control import HandState_t

from rehab_robot_control.xhand_state_mapper import map_hand_state
from rehab_robot_control.xhand_ros_state_bridge import mapped_state_to_ros

vendor = HandState_t()

vendor.finger_state[0].position = 0.123
vendor.finger_state[0].torque = -25

mapped = map_hand_state(vendor)
msg = mapped_state_to_ros(mapped)

assert list(msg.hand_id) == [0]
assert msg.hand_name == ["xhand"]
assert msg.hand_type == ["R"]

assert len(msg.hand_states) == 1
assert len(msg.hand_states[0].name) == 12
assert msg.hand_states[0].effort[0] == -25.0

assert len(msg.sensor_states) == 1
assert len(
    msg.sensor_states[0].finger_sensor_states
) == 5

print("[PASS] real installed HandState_t object")
print("[PASS] 12 joint states")
print("[PASS] 5 finger sensor states")
print("[PASS] hand_name=xhand")
print("[PASS] hand_type=R")
print("[PASS] processed signed current preserved")
PY

echo
echo "============================================================"
echo "[PASS] XHand state -> ROS message bridge verified"
echo
echo "No XHandControl object was created."
echo "No open_ethercat() call."
echo "No real read_state() call."
echo "No send_command() call."
echo "No FR3 / D435i started."
echo "============================================================"
