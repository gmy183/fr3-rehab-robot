#!/usr/bin/env bash

set -o pipefail

EDGE_FROM="${1:-}"
EDGE_TO="${2:-}"
FINAL_TO="${3:-}"

WS="$HOME/rehab_robot_ws"

TEACH="$WS/teach/teach_path_8pts_v1.yaml"
CONFIG="$WS/config/rehab_teach_executor.yaml"
SERVER="$WS/tools/rehab_move_group_plan_only_server.py"

LOGDIR="$WS/validation_logs/runtime"

SERVER_PID=""
MOVEIT_CHILD=""

cleanup_moveit() {

    if [ -n "$SERVER_PID" ]; then
        kill -INT "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1

    for P in $MOVEIT_CHILD; do
        kill -TERM "$P" \
            2>/dev/null || true
    done

    if [ -n "$SERVER_PID" ]; then
        kill -TERM "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1
}


if [ -z "$EDGE_FROM" ] \
   || [ -z "$EDGE_TO" ] \
   || [ -z "$FINAL_TO" ]; then

    echo "Usage:"
    echo "  $0 P4 P3 P1"

    exit 2
fi


cd "$WS" || exit 2

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

mkdir -p "$LOGDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

MOVEIT_LOG="$LOGDIR/teach_resume_moveit_${STAMP}.log"
RUN_LOG="$LOGDIR/teach_resume_${EDGE_FROM}_${EDGE_TO}_${FINAL_TO}_${STAMP}.log"
RESULT="$LOGDIR/teach_resume_${EDGE_FROM}_${EDGE_TO}_${FINAL_TO}_${STAMP}.json"


echo "============================================================"
echo "FR3 ONE-CLICK MID-EDGE RESUME"
echo "$EDGE_FROM -> $EDGE_TO -> ... -> $FINAL_TO"
echo "============================================================"


echo
echo "========== 1. Robot state =========="

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
    exit 2
fi


echo
echo "========== 2. Controller gate =========="

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
    echo "[BLOCK] unexpected active controller"
    exit 2
fi


echo
echo "========== 3. No existing MoveIt =========="

if pgrep -f \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  >/dev/null
then
    echo "[BLOCK] move_group already running"
    exit 2
fi


echo
echo "========== 4. READ-ONLY resume geometry check =========="

python3 -m \
  rehab_robot_control.rehab_teach_resume_executor \
  --teach-file "$TEACH" \
  --config "$CONFIG" \
  --edge-from "$EDGE_FROM" \
  --edge-to "$EDGE_TO" \
  --final-to "$FINAL_TO" \
  --preview-only

PREVIEW_RC=$?

if [ "$PREVIEW_RC" -ne 0 ]; then
    echo "[BLOCK] resume geometry validation failed"
    exit 2
fi


echo
echo "========== 5. Start planning-only MoveIt =========="

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
    echo "[FAIL] planner unavailable"
    cleanup_moveit
    exit 2
fi

MOVEIT_CHILD="$(
    pgrep -P "$SERVER_PID" \
      -f '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
      || true
)"

echo "[PASS] planner ready"


echo
echo "========== 6. Resume execution =========="

echo
echo "This is NOT an automatic retry."
echo "Current pose must still lie on:"
echo "  $EDGE_FROM -> $EDGE_TO"
echo
echo "Remaining route ends at:"
echo "  $FINAL_TO"
echo
echo "Fault -> STOP"
echo "No automatic return"
echo "No automatic error recovery"
echo

python3 -m \
  rehab_robot_control.rehab_teach_resume_executor \
  --teach-file "$TEACH" \
  --config "$CONFIG" \
  --edge-from "$EDGE_FROM" \
  --edge-to "$EDGE_TO" \
  --final-to "$FINAL_TO" \
  --result "$RESULT" \
  2>&1 | tee "$RUN_LOG"

RUN_RC="${PIPESTATUS[0]}"


echo
echo "========== 7. Cleanup =========="

cleanup_moveit

pgrep -af \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  || echo "[PASS] no residual move_group"


echo
echo "============================================================"
echo "resume_result=$RUN_RC"
echo "run_log=$RUN_LOG"
echo "result_json=$RESULT"
echo "============================================================"

echo
echo "[DONE] terminal remains open"

exit "$RUN_RC"
