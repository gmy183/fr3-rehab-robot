#!/usr/bin/env bash

main() {

WS="$HOME/rehab_robot_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"

CTRL="fr3_position_probe_controller"
CFG="/tmp/${CTRL}_${STAMP}.yaml"
LOG="$WS/validation_logs/fr3_position_probe_${STAMP}.log"

echo "============================================================"
echo "FR3 POSITION-JTC HOLD PROBE"
echo "NO TRAJECTORY / NO PTP / NO XHAND"
echo "============================================================"

source /opt/ros/jazzy/setup.bash
source "$HOME/franka_ros2_ws/install/setup.bash"
source "$WS/install/setup.bash"
source "$WS/env.sh"

mkdir -p "$WS/validation_logs"


echo
echo "========== 1. Require clean baseline =========="

OLD="$(
    pgrep -f \
    '^/opt/ros/jazzy/lib/controller_manager/ros2_control_node ' \
    || true
)"

if [ -n "$OLD" ]; then
    echo "[BLOCK] ros2_control_node already exists:"
    echo "$OLD"
    return 2
fi

echo "[PASS] no stale controller_manager"


echo
echo "========== 2. Start fresh Franka FCI =========="

nohup bash -lc "
source /opt/ros/jazzy/setup.bash
source \$HOME/franka_ros2_ws/install/setup.bash
source \$HOME/rehab_robot_ws/install/setup.bash

exec ros2 launch \
  franka_bringup \
  franka.launch.py \
  robot_type:=fr3 \
  robot_ip:=192.168.1.25 \
  load_gripper:=false \
  use_fake_hardware:=false \
  fake_sensor_commands:=false \
  load_franka_robot_state_broadcaster:=true
" > "$LOG" 2>&1 &

LAUNCH_PID=$!

echo "launch_pid=$LAUNCH_PID"
echo "bringup_log=$LOG"

HW_OK=0

for I in $(seq 1 25); do

    HW="$(
        ros2 control list_hardware_components \
        2>/dev/null || true
    )"

    if printf '%s\n' "$HW" \
       | grep -A6 'name: FrankaHardwareInterface' \
       | grep -q 'label=active'
    then
        HW_OK=1
        break
    fi

    echo "[WAIT] FCI startup $I/25"
    sleep 1
done

if [ "$HW_OK" -ne 1 ]; then
    echo "[BLOCK] FrankaHardwareInterface not ACTIVE"
    tail -n 100 "$LOG"
    return 2
fi

echo "[PASS] FrankaHardwareInterface ACTIVE"


echo
echo "========== 3. Verify broadcasters =========="

sleep 1

BASE="$(
    ros2 control list_controllers \
    2>&1
)"

printf '%s\n' "$BASE"

if ! printf '%s\n' "$BASE" \
     | grep '^joint_state_broadcaster ' \
     | grep -q ' active$'
then
    echo "[BLOCK] joint_state_broadcaster not active"
    return 2
fi

if ! printf '%s\n' "$BASE" \
     | grep '^franka_robot_state_broadcaster ' \
     | grep -q ' active$'
then
    echo "[BLOCK] franka_robot_state_broadcaster not active"
    return 2
fi

echo "[PASS] baseline broadcasters ACTIVE"


echo
echo "========== 4. Write isolated position-JTC config =========="

cat > "$CFG" <<YAML
$CTRL:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController

    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    interpolation_method: splines
    allow_partial_joints_goal: false
    open_loop_control: false
    set_last_command_interface_value_as_state_on_activation: true
YAML

echo "controller_config=$CFG"


echo
echo "========== 5. Load/configure position-JTC INACTIVE =========="

ros2 run controller_manager spawner \
  "$CTRL" \
  -c /controller_manager \
  -p "$CFG" \
  --inactive

SPAWN_RC=$?

echo "position_controller_spawn_result=$SPAWN_RC"

if [ "$SPAWN_RC" -ne 0 ]; then
    echo "[BLOCK] position-JTC configure failed"
    tail -n 80 "$LOG"
    return 2
fi


CTRL_STATE="$(
    ros2 control list_controllers
)"

printf '%s\n' "$CTRL_STATE"

if ! printf '%s\n' "$CTRL_STATE" \
     | grep "^${CTRL} " \
     | grep -q ' inactive$'
then
    echo "[BLOCK] controller not INACTIVE"
    return 2
fi

echo "[PASS] position-JTC INACTIVE"


echo
echo "========== 6. Verify controller contract =========="

ros2 param get \
  "/$CTRL" \
  command_interfaces \
  2>/dev/null || true

ros2 param get \
  "/$CTRL" \
  set_last_command_interface_value_as_state_on_activation \
  2>/dev/null || true


echo
echo "========== 7. Real activation safety gate =========="
echo
echo "NO FollowJointTrajectory goal will be sent."
echo
echo "The controller will ONLY be activated for 2 seconds."
echo
echo "Confirm:"
echo "  Desk = EXEC"
echo "  FCI = ON"
echo "  Joints = UNLOCKED"
echo "  Hand-guiding = OFF"
echo "  Workspace completely clear"
echo "  NO patient / ankle contact"
echo
echo "Type exactly:"
echo "EXECUTE-FR3-POSITION-HOLD-PROBE"
echo

read -r CONFIRM

if [ "$CONFIRM" != "EXECUTE-FR3-POSITION-HOLD-PROBE" ]; then

    echo "[CANCEL] confirmation mismatch"

    ros2 control unload_controller \
      -c /controller_manager \
      "$CTRL" \
      >/dev/null 2>&1 || true

    return 3
fi


echo
echo "========== 8. Capture q before activation =========="

Q0="$(
python3 <<'PY'
import time
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

wanted = [f"fr3_joint{i}" for i in range(1, 8)]

rclpy.init()
node = Node("position_probe_before_reader")

box = {}

def cb(msg):
    d = dict(zip(msg.name, msg.position))
    if all(j in d for j in wanted):
        box["q"] = [float(d[j]) for j in wanted]

sub = node.create_subscription(
    JointState,
    "/franka/joint_states",
    cb,
    10,
)

deadline = time.monotonic() + 4.0

while "q" not in box and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

if "q" not in box:
    raise SystemExit(2)

print(",".join(f"{x:.12f}" for x in box["q"]))
PY
)"

Q0_RC=$?

if [ "$Q0_RC" -ne 0 ]; then
    echo "[BLOCK] live q unavailable before activation"
    return 2
fi

echo "q_before=$Q0"


echo
echo "========== 9. ACTIVATE position-JTC =========="

ros2 control switch_controllers \
  --activate "$CTRL"

ACT_RC=$?

echo "position_controller_activate_result=$ACT_RC"

if [ "$ACT_RC" -ne 0 ]; then
    echo "[BLOCK] controller activation failed"
    return 2
fi

echo "[WAIT] position controller HOLD 2.0 s"
sleep 2


echo
echo "========== 10. Capture q after hold =========="

Q1="$(
python3 <<'PY'
import time
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

wanted = [f"fr3_joint{i}" for i in range(1, 8)]

rclpy.init()
node = Node("position_probe_after_reader")

box = {}

def cb(msg):
    d = dict(zip(msg.name, msg.position))
    if all(j in d for j in wanted):
        box["q"] = [float(d[j]) for j in wanted]

sub = node.create_subscription(
    JointState,
    "/franka/joint_states",
    cb,
    10,
)

deadline = time.monotonic() + 4.0

while "q" not in box and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

if "q" not in box:
    raise SystemExit(2)

print(",".join(f"{x:.12f}" for x in box["q"]))
PY
)"

Q1_RC=$?

echo "q_after=$Q1"


echo
echo "========== 11. HOLD displacement =========="

if [ "$Q1_RC" -eq 0 ]; then

python3 - "$Q0" "$Q1" <<'PY'
import sys

q0 = [float(x) for x in sys.argv[1].split(",")]
q1 = [float(x) for x in sys.argv[2].split(",")]

d = [abs(b-a) for a,b in zip(q0,q1)]

print(
    "activation_hold_max_joint_shift = "
    f"{max(d):.6f} rad"
)

print(
    "per_joint_shift =",
    [round(x,6) for x in d],
)
PY

else
    echo "[FAULT] joint-state stream disappeared during HOLD"
fi


echo
echo "========== 12. DEACTIVATE position-JTC =========="

ros2 control switch_controllers \
  --deactivate "$CTRL"

DEACT_RC=$?

echo "position_controller_deactivate_result=$DEACT_RC"


echo
echo "========== 13. Unload position-JTC =========="

ros2 control unload_controller \
  -c /controller_manager \
  "$CTRL"

UNLOAD_RC=$?

echo "position_controller_unload_result=$UNLOAD_RC"

sleep 1


echo
echo "========== 14. Hardware survival check =========="

FINAL_HW="$(
    ros2 control list_hardware_components \
    2>&1
)"

printf '%s\n' "$FINAL_HW"

HW_ACTIVE=0

if printf '%s\n' "$FINAL_HW" \
   | grep -A6 'name: FrankaHardwareInterface' \
   | grep -q 'label=active'
then
    HW_ACTIVE=1
fi

echo "hardware_active_after_position_cycle=$HW_ACTIVE"


echo
echo "========== 15. Broadcaster survival check =========="

FINAL_CTRL="$(
    ros2 control list_controllers \
    2>&1
)"

printf '%s\n' "$FINAL_CTRL"

JS_ACTIVE="$(
    printf '%s\n' "$FINAL_CTRL" \
    | grep '^joint_state_broadcaster ' \
    | grep -c ' active$'
)"

RS_ACTIVE="$(
    printf '%s\n' "$FINAL_CTRL" \
    | grep '^franka_robot_state_broadcaster ' \
    | grep -c ' active$'
)"

echo "joint_state_broadcaster_active=$JS_ACTIVE"
echo "franka_robot_state_broadcaster_active=$RS_ACTIVE"


echo
echo "========== 16. robot_state survival check =========="

STATE_OK=0

for I in 1 2 3; do

    OUT="$(
        timeout 4 \
        ros2 topic echo \
          /franka_robot_state_broadcaster/robot_state \
          --once \
          2>/dev/null
    )"

    if printf '%s\n' "$OUT" \
       | grep -q '^robot_mode:'
    then
        echo "[PASS] robot_state $I/3"
        STATE_OK=$((STATE_OK + 1))
    else
        echo "[FAIL] robot_state $I/3"
    fi

    sleep 0.2
done


echo
echo "========== 17. libfranka fault scan =========="

grep -nEi \
  'UDP receive|TCP connection|NetworkException|ControlException|read cycle resulted in an error|Error while performing mode switch' \
  "$LOG" \
  | tail -n 50 \
  || true


echo
echo "============================================================"
echo "POSITION-JTC HOLD PROBE RESULT"
echo "============================================================"
echo "spawn_result=$SPAWN_RC"
echo "activate_result=$ACT_RC"
echo "deactivate_result=$DEACT_RC"
echo "unload_result=$UNLOAD_RC"
echo "hardware_active_after_position_cycle=$HW_ACTIVE"
echo "joint_state_broadcaster_active=$JS_ACTIVE"
echo "franka_robot_state_broadcaster_active=$RS_ACTIVE"
echo "robot_state_samples=$STATE_OK/3"
echo
echo "NO FOLLOWJOINTTRAJECTORY GOAL SENT"
echo "NO PTP"
echo "NO XHAND"
echo "============================================================"

if [ "$ACT_RC" -eq 0 ] \
   && [ "$DEACT_RC" -eq 0 ] \
   && [ "$UNLOAD_RC" -eq 0 ] \
   && [ "$HW_ACTIVE" -eq 1 ] \
   && [ "$JS_ACTIVE" -eq 1 ] \
   && [ "$RS_ACTIVE" -eq 1 ] \
   && [ "$STATE_OK" -eq 3 ]
then
    echo
    echo "[PASS] POSITION-JTC lifecycle survived cleanly"
    echo "[NEXT] position backend is eligible for V3.2 5-mm test"
    return 0
fi

echo
echo "[FAIL] POSITION-JTC lifecycle did not survive cleanly"
echo "[SAFE] DO NOT send any trajectory"
return 2
}

main
