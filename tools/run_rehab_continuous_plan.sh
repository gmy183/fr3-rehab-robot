#!/usr/bin/env bash

set -o pipefail

WS="$HOME/rehab_robot_ws"
OUTDIR="$WS/validation_logs/runtime"

mkdir -p "$OUTDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

RESULT="$OUTDIR/continuous_plan_current_P2_P1_${STAMP}.json"

MOVEIT_LOG="$OUTDIR/continuous_plan_moveit_${STAMP}.log"

SERVER="$WS/tools/rehab_move_group_plan_only_server.py"

SERVER_PID=""
MOVEIT_CHILD=""

cleanup() {

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


cd "$WS" || exit 2
source "$WS/env.sh"


echo "============================================================"
echo "FR3 CONTINUOUS TRAJECTORY PLAN-ONLY V1"
echo "CURRENT -> P2 -> P1"
echo "============================================================"


echo
echo "========== 1. Safety state =========="

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
    echo "[BLOCK] FR3 not IDLE"
    exit 2
fi


echo
echo "========== 2. Controller gate =========="

CTRL="$(
    ros2 control list_controllers \
    2>/dev/null || true
)"

printf '%s\n' "$CTRL"

BAD="$(
    printf '%s\n' "$CTRL" \
    | awk '$NF=="active" {print $1}' \
    | grep -vxE \
      'joint_state_broadcaster|franka_robot_state_broadcaster' \
    || true
)"

if [ -n "$BAD" ]; then
    echo "[BLOCK] motion controller active"
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
echo "========== 4. Start planning-only MoveIt =========="

python3 "$SERVER" \
    --robot-ip 192.168.1.25 \
    >"$MOVEIT_LOG" 2>&1 &

SERVER_PID=$!

READY=0

for I in $(seq 1 40); do

    if timeout 2 \
       ros2 service list -t \
       2>/dev/null \
       | grep -q \
         '^/compute_cartesian_path \[moveit_msgs/srv/GetCartesianPath\]'
    then
        READY=1
        break
    fi

    echo "[WAIT] planner $I/40"
    sleep 0.5
done

if [ "$READY" -ne 1 ]; then
    echo "[FAIL] Cartesian planner unavailable"
    cleanup
    exit 2
fi

MOVEIT_CHILD="$(
    pgrep -P "$SERVER_PID" \
      -f '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
      || true
)"

echo "[PASS] planning service ready"


echo
echo "========== 5. Generate continuous trajectory =========="

python3 -m \
  rehab_robot_control.rehab_continuous_trajectory_plan \
  --output "$RESULT"

PLAN_RC=$?


echo
echo "========== 6. Cleanup MoveIt =========="

cleanup

pgrep -af \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  || echo "[PASS] no residual move_group"


echo
echo "========== 7. Final controller state =========="

ros2 control list_controllers \
  2>/dev/null || true


echo
echo "============================================================"
echo "CONTINUOUS PLAN RESULT"
echo "============================================================"
echo "plan_result=$PLAN_RC"
echo "trajectory_json=$RESULT"
echo "moveit_log=$MOVEIT_LOG"
echo
echo "NO CONTROLLER ACTIVATION"
echo "NO FOLLOWJOINTTRAJECTORY GOAL"
echo "NO PTP GOAL"
echo "NO ERROR RECOVERY"
echo "NO XHAND"
echo "============================================================"
echo
echo "[DONE] terminal remains open"

exit "$PLAN_RC"
