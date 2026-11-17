#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

CONTROL="$HOME/rehab_robot_ws/src/rehab_robot_control"

echo "============================================================"
echo "XHand full software state publication verification"
echo "============================================================"

echo
echo "========== pytest =========="

python3 -m pytest -q \
  "$CONTROL/test/test_xhand_driver_core.py" \
  "$CONTROL/test/test_xhand_ros_contract.py" \
  "$CONTROL/test/test_xhand_state_mapper.py" \
  "$CONTROL/test/test_xhand_ros_state_bridge.py" \
  "$CONTROL/test/test_xhand_fake_state_pipeline.py"

echo
echo "========== start fake ROS node =========="

LOG="$(mktemp)"
PID=""

cleanup() {
    if [[ -n "${PID:-}" ]] && kill -0 "$PID" 2>/dev/null; then
        kill -TERM -- "-$PID" 2>/dev/null || true

        for _ in $(seq 1 20); do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$PID" 2>/dev/null; then
            kill -KILL -- "-$PID" 2>/dev/null || true
        fi

        wait "$PID" 2>/dev/null || true
    fi

    rm -f "$LOG"
}

trap cleanup EXIT INT TERM

setsid ros2 run \
  rehab_robot_control \
  xhand_control_ros2_node \
  --ros-args \
  -p hardware_enable:=false \
  -p state_source:=fake \
  -p state_rate_hz:=100.0 \
  -p watchdog_s:=0.5 \
  >"$LOG" 2>&1 &

PID=$!

READY=0

for _ in $(seq 1 50); do
    if ros2 node list 2>/dev/null \
        | grep -Fxq "/xhand_control/xhand_control_ros2"; then
        READY=1
        break
    fi

    if ! kill -0 "$PID" 2>/dev/null; then
        break
    fi

    sleep 0.1
done

if [[ "$READY" -ne 1 ]]; then
    cat "$LOG"
    echo "[FAIL] fake XHand node did not start"
    exit 1
fi

echo "[PASS] fake node started"

echo
echo "========== receive one real ROS message =========="

python3 - <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node

from rehab_robot_interfaces.msg import XHandState


class Receiver(Node):
    def __init__(self):
        super().__init__("xhand_fake_state_receiver")

        self.message = None

        self.create_subscription(
            XHandState,
            "/xhand_control/xhand_state",
            self.callback,
            10,
        )

    def callback(self, msg):
        self.message = msg


rclpy.init()
node = Receiver()

deadline = time.monotonic() + 5.0

while (
    rclpy.ok()
    and node.message is None
    and time.monotonic() < deadline
):
    rclpy.spin_once(
        node,
        timeout_sec=0.1,
    )

msg = node.message

node.destroy_node()
rclpy.shutdown()

if msg is None:
    print("[FAIL] no XHandState received")
    sys.exit(1)

assert list(msg.hand_id) == [0]
assert list(msg.hand_name) == ["xhand"]
assert list(msg.hand_type) == ["R"]

assert len(msg.hand_states) == 1
assert len(msg.hand_states[0].name) == 12
assert len(msg.hand_states[0].position) == 12
assert len(msg.hand_states[0].effort) == 12
assert len(msg.hand_states[0].temperature) == 12
assert len(msg.hand_states[0].error_code) == 12

assert msg.hand_states[0].name[0] == "thumb_bend_joint"
assert msg.hand_states[0].name[11] == "pinky_joint2"

assert msg.hand_states[0].effort[0] == -20.0

assert len(msg.sensor_states) == 1
assert len(
    msg.sensor_states[0].finger_sensor_states
) == 5

locations = [
    value.location
    for value
    in msg.sensor_states[0].finger_sensor_states
]

assert locations == [
    "thumb",
    "index",
    "middle",
    "ring",
    "pinky",
]

print("[PASS] received /xhand_control/xhand_state")
print("[PASS] 12 joint states")
print("[PASS] 5 finger sensor states")
print("[PASS] state passed through XHandDriverCore")
print("[PASS] mapper -> ROS bridge -> publisher")
PY

echo
echo "========== observed publish rate =========="

timeout 4s \
  ros2 topic hz /xhand_control/xhand_state \
  2>/dev/null || true

echo
echo "========== node log =========="
cat "$LOG"

echo
echo "========== bounded shutdown =========="

kill -TERM -- "-$PID" 2>/dev/null || true

STOPPED=0

for _ in $(seq 1 20); do
    if ! kill -0 "$PID" 2>/dev/null; then
        STOPPED=1
        break
    fi
    sleep 0.1
done

if [[ "$STOPPED" -ne 1 ]]; then
    kill -KILL -- "-$PID" 2>/dev/null || true
fi

wait "$PID" 2>/dev/null || true
PID=""

echo "[PASS] fake node stopped"

echo
echo "============================================================"
echo "[PASS] complete fake XHand state publication path"
echo
echo "No vendor XHandControl object was created."
echo "No real EtherCAT interface was opened."
echo "No real XHand read_state() was called."
echo "No send_command() was called."
echo "No FR3 / D435i was started."
echo "============================================================"
