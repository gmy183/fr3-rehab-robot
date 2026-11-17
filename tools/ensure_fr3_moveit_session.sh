#!/usr/bin/env bash

set +e

WS="$HOME/rehab_robot_ws"
RUNTIME="$WS/.runtime"
LOGDIR="$WS/validation_logs/runtime"

mkdir -p "$RUNTIME" "$LOGDIR"

source "$HOME/robot_env/rehab.sh"

PIDFILE="$RUNTIME/fr3_moveit_session.pid"
LOG="$LOGDIR/fr3_moveit_session_$(date +%Y%m%d_%H%M%S).log"

controllers_ready()
{
    local OUT

    OUT="$(
        timeout 3 \
        ros2 control list_controllers \
        2>/dev/null
    )"

    [ $? -eq 0 ] || return 1

    for NAME in \
        fr3_arm_controller \
        joint_state_broadcaster \
        franka_robot_state_broadcaster
    do
        printf '%s\n' "$OUT" \
        | awk '$NF=="active" {print $1}' \
        | grep -qx "$NAME" \
        || return 1
    done

    return 0
}

moveit_ready()
{
    timeout 3 ros2 node list \
        2>/dev/null \
        | grep -qx '/move_group' \
        || return 1

    timeout 3 ros2 action list \
        2>/dev/null \
        | grep -qx '/execute_trajectory' \
        || return 1

    return 0
}

echo "============================================================"
echo "FR3 OFFICIAL MOVEIT SESSION GATE"
echo "============================================================"

# ------------------------------------------------------------
# 1. 已有完整健康会话：直接复用
# ------------------------------------------------------------

if controllers_ready && moveit_ready; then
    echo "[PASS] existing official MoveIt session healthy"
    exit 0
fi

# ------------------------------------------------------------
# 2. 如果已经有 controller_manager，
#    但不是完整 MoveIt 会话，不做自动 controller switching
# ------------------------------------------------------------

CM="$(
    timeout 3 ros2 node list \
    2>/dev/null \
    | grep -x '/controller_manager' \
    || true
)"

if [ -n "$CM" ]; then
    echo "[INFO] controller_manager exists"
    echo "[INFO] waiting for existing MoveIt session to settle..."

    READY=0

    for SECOND in $(seq 1 8); do

        if controllers_ready && moveit_ready; then
            READY=1
            break
        fi

        printf '\r[WAIT] existing MoveIt readiness... %d/8 s' \
            "$SECOND"

        sleep 1
    done

    echo

    if [ "$READY" -eq 1 ]; then
        echo "[PASS] existing official MoveIt session healthy"
        exit 0
    fi

    echo "[FAIL] existing FR3 session did not become fully ready"
    echo
    echo "[SAFE] no controller switching"
    echo "[SAFE] no process killing"
    echo "[SAFE] no motion sent"
    exit 2
fi

# ------------------------------------------------------------
# 3. 没有 FR3 控制会话：
#    自动启动已经验证使用的官方 MoveIt
# ------------------------------------------------------------

echo "[INFO] no controller_manager"
echo "[INFO] starting official FR3 MoveIt..."
echo "log=$LOG"

setsid bash -lc '
    source /home/gu/robot_env/franka.sh

    exec ros2 launch \
      franka_fr3_moveit_config \
      moveit.launch.py \
      robot_ip:=192.168.1.25 \
      load_gripper:=false
' >"$LOG" 2>&1 &

PID=$!

echo "$PID" > "$PIDFILE"

echo "launch_pid=$PID"

# ------------------------------------------------------------
# 4. 等待官方栈全部 ready
# ------------------------------------------------------------

READY=0

for SECOND in $(seq 1 45); do

    if ! kill -0 "$PID" 2>/dev/null; then
        echo "[FAIL] MoveIt launch exited unexpectedly"
        echo
        tail -n 120 "$LOG"
        exit 2
    fi

    if controllers_ready && moveit_ready; then
        READY=1
        break
    fi

    printf '\r[WAIT] official MoveIt starting... %02d/45 s' \
        "$SECOND"

    sleep 1
done

echo

if [ "$READY" -ne 1 ]; then
    echo "[FAIL] official MoveIt did not become ready"
    echo
    echo "========== LOG TAIL =========="
    tail -n 120 "$LOG"
    exit 2
fi

echo
echo "[PASS] controller_manager available"
echo "[PASS] fr3_arm_controller active"
echo "[PASS] joint_state_broadcaster active"
echo "[PASS] franka_robot_state_broadcaster active"
echo "[PASS] move_group ready"
echo "[PASS] /execute_trajectory ready"
echo
echo "[PASS] OFFICIAL FR3 MOVEIT SESSION READY"

exit 0
