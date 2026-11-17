#!/usr/bin/env bash

set -o pipefail

FROM="${1:-}"
TO="${2:-}"

WS="$HOME/rehab_robot_ws"

TEACH="$WS/teach/teach_path_8pts_v1.yaml"
CONFIG="$WS/config/rehab_teach_executor.yaml"

SERVER="$WS/tools/rehab_move_group_plan_only_server.py"

LOGDIR="$WS/validation_logs/runtime"

SERVER_PID=""
NEW_MOVE_GROUP_PIDS=""

cleanup_teach_moveit() {

    if [ -n "$SERVER_PID" ]; then
        kill -INT "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1

    for P in $NEW_MOVE_GROUP_PIDS; do
        kill -TERM "$P" \
            2>/dev/null || true
    done

    if [ -n "$SERVER_PID" ]; then
        kill -TERM "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1
}

if [ -z "$FROM" ] || [ -z "$TO" ]; then

    echo "Usage:"
    echo "  $0 P6 P1"
    echo "  $0 P1 P8"

    exit 2
fi


cd "$WS" || exit 2

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

mkdir -p "$LOGDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

MOVEIT_LOG="$LOGDIR/teach_path_${FROM}_${TO}_moveit_${STAMP}.log"

RUN_LOG="$LOGDIR/teach_path_${FROM}_${TO}_${STAMP}.log"

RESULT="$LOGDIR/teach_path_${FROM}_${TO}_${STAMP}.json"


echo "============================================================"
echo "FR3 FINAL ONE-CLICK TEACH PATH EXECUTOR"
echo "$FROM -> $TO"
echo "============================================================"


echo
echo "========== 1. Files =========="

for FILE in \
    "$TEACH" \
    "$CONFIG" \
    "$SERVER"
do
    if [ ! -f "$FILE" ]; then
        echo "[BLOCK] missing: $FILE"
        exit 2
    fi
done

echo "[PASS] required files present"


echo
echo "========== 2. Robot state =========="

MODE="$(
    timeout 3 \
    ros2 topic echo \
    /franka_robot_state_broadcaster/robot_state \
    --once \
    2>/dev/null \
    | awk '/^robot_mode:/ {print $2; exit}'
)"

echo "robot_mode=$MODE"

if [ "$MODE" != "1" ]; then

    echo "[BLOCK] FR3 is not IDLE"
    echo "[BLOCK] expected robot_mode=1"

    exit 2
fi


echo
echo "========== 3. Controllers =========="

CONTROLLERS="$(
    timeout 5 \
    ros2 control list_controllers \
    2>/dev/null || true
)"

printf '%s\n' "$CONTROLLERS"

BAD="$(
    printf '%s\n' "$CONTROLLERS" \
    | awk '$NF=="active" {print $1}' \
    | grep -vxE \
      'joint_state_broadcaster|franka_robot_state_broadcaster' \
    || true
)"

if [ -n "$BAD" ]; then

    echo "[BLOCK] unexpected active controller:"
    printf '%s\n' "$BAD"

    exit 2
fi

echo "[PASS] controller gate"


echo
echo "========== 4. Existing MoveIt =========="

PRE_MOVE_GROUP="$(
    pgrep -f \
      '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
      || true
)"

if [ -n "$PRE_MOVE_GROUP" ]; then

    echo "[BLOCK] move_group already running:"
    pgrep -af \
      '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group'

    echo
    echo "Do not start a second planning server."

    exit 2
fi

echo "[PASS] no existing move_group"


echo
echo "========== 5. Route preview =========="

python3 -m \
  rehab_robot_control.rehab_teach_path_executor \
  --teach-file "$TEACH" \
  --config "$CONFIG" \
  --from-point "$FROM" \
  --to-point "$TO" \
  --preview-only

PREVIEW_RC=$?

if [ "$PREVIEW_RC" -ne 0 ]; then

    echo "[BLOCK] route preview failed"

    exit 2
fi


echo
echo "========== 6. Start planning-only MoveIt =========="

python3 "$SERVER" \
  --robot-ip "$FR3_ROBOT_IP" \
  >"$MOVEIT_LOG" 2>&1 &

SERVER_PID=$!

READY=0

for _ in $(seq 1 40); do

    if timeout 2 \
      ros2 service list -t \
      2>/dev/null \
      | grep -q \
        '^/compute_cartesian_path \[moveit_msgs/srv/GetCartesianPath\]'
    then
        READY=1
        break
    fi

    sleep 0.5
done

if [ "$READY" -ne 1 ]; then

    echo "[FAIL] Cartesian planner unavailable"

    tail -n 120 \
      "$MOVEIT_LOG" || true

    cleanup_teach_moveit

    exit 2
fi

NEW_MOVE_GROUP_PIDS="$(
    pgrep -P "$SERVER_PID" \
      -f '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
      || true
)"

echo "[PASS] Cartesian planner ready"


echo
echo "========== 7. Final real-motion conditions =========="

echo
echo "Desk:"
echo "  Mode   = EXEC"
echo "  FCI    = ON"
echo "  Joints = Unlocked"
echo
echo "Workspace:"
echo "  No human contact"
echo "  No ankle/fake-foot contact"
echo "  No obstacle along entire taught route"
echo
echo "Execution:"
echo "  $FROM -> $TO"
echo "  max subsegment = 15 mm"
echo "  PTP speed = 0.015 rad/s"
echo "  fixed task-start flange orientation"
echo "  XHand = no command"
echo "  Camera = not used"
echo "  Fault = immediate stop"
echo "  Automatic return = disabled"
echo


echo "========== 8. Execute =========="

python3 -m \
  rehab_robot_control.rehab_teach_path_executor \
  --teach-file "$TEACH" \
  --config "$CONFIG" \
  --from-point "$FROM" \
  --to-point "$TO" \
  --result "$RESULT" \
  2>&1 | tee "$RUN_LOG"

RUN_RC="${PIPESTATUS[0]}"


echo
echo "========== 9. Cleanup planning-only MoveIt =========="

cleanup_teach_moveit

if pgrep -f \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  >/dev/null
then

    echo "[WARN] move_group still exists:"
    pgrep -af \
      '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group'

else

    echo "[PASS] no residual move_group"
fi


echo
echo "========== 10. Final FR3 state =========="

timeout 3 \
ros2 topic echo \
/franka_robot_state_broadcaster/robot_state \
--once \
2>/dev/null \
| grep -E \
'^robot_mode:|joint_reflex:|cartesian_reflex:|communication_constraints_violation:|instability_detected:' \
|| true

echo

ros2 control list_controllers \
  2>/dev/null || true


echo
echo "============================================================"
echo "teach_path_result=$RUN_RC"
echo "run_log=$RUN_LOG"
echo "result_json=$RESULT"
echo "moveit_log=$MOVEIT_LOG"
echo "============================================================"

echo
echo "[DONE] terminal remains open"

exit "$RUN_RC"
