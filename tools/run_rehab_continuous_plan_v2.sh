#!/usr/bin/env bash

set -o pipefail

WS="$HOME/rehab_robot_ws"
OUTDIR="$WS/validation_logs/runtime"

SERVER="$WS/tools/rehab_move_group_plan_only_server.py"

mkdir -p "$OUTDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

RESULT="$OUTDIR/continuous_plan_v2_current_P2_P1_${STAMP}.json"

MOVEIT_LOG="$OUTDIR/continuous_plan_v2_moveit_${STAMP}.log"

SERVER_PID=""
MOVEIT_CHILDREN=""


cleanup_v2() {

    if [ -n "$SERVER_PID" ]; then
        kill -INT "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1

    MOVEIT_CHILDREN="$(
        pgrep -f \
          '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
          2>/dev/null \
        || true
    )"

    if [ -n "$MOVEIT_CHILDREN" ]; then

        for PID in $MOVEIT_CHILDREN; do
            kill -TERM "$PID" \
                2>/dev/null || true
        done
    fi

    if [ -n "$SERVER_PID" ]; then
        kill -TERM "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1
}


cd "$WS" || exit 2
source "$WS/env.sh"


echo "============================================================"
echo "FR3 CONTINUOUS TRAJECTORY PLAN-ONLY V2"
echo "MOVEIT TIMING + QUINTIC DENSE FK AUDIT"
echo "CURRENT -> P2 -> P1"
echo "============================================================"


echo
echo "========== 1. FR3 safety gate =========="

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
    echo "[BLOCK] motion controller active:"
    printf '%s\n' "$BAD"
    exit 2
fi


echo
echo "========== 3. Process gate =========="

if pgrep -f \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  >/dev/null
then
    echo "[BLOCK] move_group already running"
    exit 2
fi

if pgrep -f \
  'rehab_move_group_plan_only_server.py' \
  >/dev/null
then
    echo "[BLOCK] planning server already running"
    exit 2
fi

echo "[PASS] no existing MoveIt process"


echo
echo "========== 4. Start planning-only MoveIt =========="

python3 "$SERVER" \
    --robot-ip 192.168.1.25 \
    >"$MOVEIT_LOG" 2>&1 &

SERVER_PID=$!

READY=0

for I in $(seq 1 50); do

    SERVICES="$(
        timeout 2 \
        ros2 service list -t \
        2>/dev/null \
        || true
    )"

    HAS_CART=0
    HAS_FK=0

    if printf '%s\n' "$SERVICES" \
       | grep -q \
         '^/compute_cartesian_path \[moveit_msgs/srv/GetCartesianPath\]'
    then
        HAS_CART=1
    fi

    if printf '%s\n' "$SERVICES" \
       | grep -q \
         '^/compute_fk \[moveit_msgs/srv/GetPositionFK\]'
    then
        HAS_FK=1
    fi

    if [ "$HAS_CART" -eq 1 ] && \
       [ "$HAS_FK" -eq 1 ]
    then
        READY=1
        break
    fi

    echo "[WAIT] planner services $I/50"
    sleep 0.5
done

if [ "$READY" -ne 1 ]; then

    echo "[FAIL] MoveIt planning services unavailable"

    echo
    echo "---------- MoveIt log tail ----------"

    tail -n 140 "$MOVEIT_LOG" \
        2>/dev/null || true

    cleanup_v2

    exit 2
fi

echo "[PASS] /compute_cartesian_path ready"
echo "[PASS] /compute_fk ready"


echo
echo "========== 5. Generate + audit V2 trajectory =========="

python3 -m \
  rehab_robot_control.rehab_continuous_trajectory_plan_v2 \
  --output "$RESULT"

PLAN_RC=$?


echo
echo "========== 6. Cleanup planning-only MoveIt =========="

cleanup_v2

if pgrep -f \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  >/dev/null
then
    echo "[WARN] residual move_group detected"
else
    echo "[PASS] no residual move_group"
fi


echo
echo "========== 7. Final FR3 state =========="

MODE2="$(
    timeout 3 \
    ros2 topic echo \
      /franka_robot_state_broadcaster/robot_state \
      --once \
      2>/dev/null \
    | awk '/^robot_mode:/ {print $2; exit}'
)"

echo "robot_mode=$MODE2"

ros2 control list_controllers \
  2>/dev/null || true


echo
echo "============================================================"
echo "CONTINUOUS PLAN V2 RESULT"
echo "============================================================"
echo "plan_result=$PLAN_RC"
echo "trajectory_json=$RESULT"
echo "moveit_log=$MOVEIT_LOG"
echo
echo "NO CONTROLLER LOAD"
echo "NO CONTROLLER ACTIVATION"
echo "NO FOLLOWJOINTTRAJECTORY GOAL"
echo "NO PTP GOAL"
echo "NO ERROR RECOVERY"
echo "NO XHAND"
echo "============================================================"
echo
echo "[DONE] terminal remains open"

exit "$PLAN_RC"
