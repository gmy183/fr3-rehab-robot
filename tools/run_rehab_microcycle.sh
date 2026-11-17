#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

LOGDIR="$WS/validation_logs/runtime"
mkdir -p "$LOGDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

EXEC_LOG="$LOGDIR/rehab_microcycle_${STAMP}.log"
FR3_LOG="$LOGDIR/fr3_rehab_microcycle_${STAMP}.log"

FR3_STARTED=0
FR3_PID=""
FR3_PGID=""


cleanup_fr3()
{
    if [ "$FR3_STARTED" -ne 1 ]; then
        return
    fi

    echo
    echo "[SAFE] stopping microcycle-started FR3 bringup"

    if [ -n "$FR3_PGID" ]; then
        kill -INT -- "-$FR3_PGID" \
          2>/dev/null || true

        for _ in 1 2 3 4 5 6 7 8; do
            if ! kill -0 -- "-$FR3_PGID" \
                2>/dev/null; then
                break
            fi

            sleep 0.5
        done

        if kill -0 -- "-$FR3_PGID" \
            2>/dev/null; then

            kill -TERM -- "-$FR3_PGID" \
              2>/dev/null || true

            sleep 1
        fi
    fi

    echo "[PASS] microcycle-started FR3 bringup stopped"
}


echo "============================================================"
echo "FR3 + XHand REHAB MICRO-CYCLE V1"
echo "ONE CYCLE / EMPTY LOAD"
echo "============================================================"

echo
echo "========== 1. 软件合同检查 =========="

python3 -m \
  rehab_robot_control.rehab_microcycle_executor \
  --check

CHECK_RC=$?

if [ "$CHECK_RC" -ne 0 ]; then
    echo
    echo "[FAIL] microcycle software check"
    echo "Real motion NOT started."
    exit 2
fi

echo
echo "[PASS] microcycle software contract"


echo
echo "========== 2. 正式只读 Preflight =========="

"$WS/tools/run_empty_grasp.sh" \
  --preflight

PRE_RC=$?

if [ "$PRE_RC" -ne 0 ]; then
    echo
    echo "[FAIL] live preflight failed"
    echo "Real motion NOT started."
    cleanup_fr3
    exit 2
fi

echo
echo "[PASS] live preflight"


echo
echo "========== 3. sudo =========="

sudo -v

if [ "$?" -ne 0 ]; then
    echo "[FAIL] sudo"
    cleanup_fr3
    exit 2
fi

echo "[PASS] sudo"


echo
echo "========== 4. 保证 FR3 session 在线 =========="

CURRENT_CM="$(
    timeout 4 ros2 node list \
      2>/dev/null \
      | grep -x '/controller_manager' \
      || true
)"

if [ -z "$CURRENT_CM" ]; then

    echo "[INFO] starting official Franka bringup"

    setsid ros2 launch \
      franka_bringup \
      franka.launch.py \
      robot_type:="$FR3_ARM_ID" \
      robot_ip:="$FR3_ROBOT_IP" \
      load_gripper:=false \
      use_fake_hardware:=false \
      fake_sensor_commands:=false \
      load_franka_robot_state_broadcaster:=true \
      >"$FR3_LOG" 2>&1 &

    FR3_PID=$!
    FR3_STARTED=1

    sleep 0.3

    FR3_PGID="$(
        ps -o pgid= -p "$FR3_PID" \
          | tr -d ' '
    )"

    echo "fr3_pid=$FR3_PID"
    echo "fr3_pgid=$FR3_PGID"

    READY=0

    for SECOND in $(seq 1 30); do

        CONTROLLERS="$(
            timeout 3 \
              ros2 control list_controllers \
              2>/dev/null \
              || true
        )"

        HAVE_JS=0
        HAVE_RS=0

        printf '%s\n' "$CONTROLLERS" \
          | awk '$NF=="active" {print $1}' \
          | grep -qx \
              'joint_state_broadcaster' \
          && HAVE_JS=1

        printf '%s\n' "$CONTROLLERS" \
          | awk '$NF=="active" {print $1}' \
          | grep -qx \
              'franka_robot_state_broadcaster' \
          && HAVE_RS=1

        if [ "$HAVE_JS" -eq 1 ] \
           && [ "$HAVE_RS" -eq 1 ]; then

            READY=1
            break
        fi

        echo "[WAIT] FR3 controllers ${SECOND}/30"
        sleep 1
    done

    if [ "$READY" -ne 1 ]; then
        echo "[FAIL] FR3 bringup not ready"
        tail -n 120 "$FR3_LOG" || true
        cleanup_fr3
        exit 2
    fi

else

    echo "[INFO] existing /controller_manager detected"

    CONTROLLERS="$(
        timeout 4 \
          ros2 control list_controllers \
          2>/dev/null \
          || true
    )"

    printf '%s\n' "$CONTROLLERS"

    HAVE_JS=0
    HAVE_RS=0

    printf '%s\n' "$CONTROLLERS" \
      | awk '$NF=="active" {print $1}' \
      | grep -qx \
          'joint_state_broadcaster' \
      && HAVE_JS=1

    printf '%s\n' "$CONTROLLERS" \
      | awk '$NF=="active" {print $1}' \
      | grep -qx \
          'franka_robot_state_broadcaster' \
      && HAVE_RS=1

    if [ "$HAVE_JS" -ne 1 ] \
       || [ "$HAVE_RS" -ne 1 ]; then

        echo "[FAIL] existing FR3 session is not healthy"
        echo "Real motion NOT started."
        exit 2
    fi

    echo "[PASS] using existing healthy FR3 session"
fi


echo
echo "========== 5. 操作员最终确认 =========="
echo
echo "即将执行真实的单次康复微循环。"
echo
echo "必须同时满足："
echo "  1. XHand 完全空载"
echo "  2. FR3 工作空间无人、无障碍物"
echo "  3. 急停按钮在手边"
echo "  4. Desk 中 FCI 已激活"
echo "  5. Desk 中 FR3 Joints 已 UNLOCKED"
echo "  6. 不接触 FR3 与 XHand"
echo
echo "本轮唯一运动范围："
echo "  XHand Δq = +0.005 rad"
echo "  XHand tor_max = 150 mA"
echo "  XHand hold = 20 Hz"
echo "  FR3 Joint1: 当前实时 q0 -> q0 + 0.005 rad"
echo "  FR3 Joint1: q0 + 0.005 rad -> 同一本轮 q0"
echo "  FR3 vmax = 0.020 rad/s"
echo "  循环次数 = 1"
echo
echo "注意："
echo "  正常情况下返回 q0 是计划轨迹的一部分。"
echo "  如果任何阶段发生故障："
echo "    当前 FR3 Action -> cancel"
echo "    XHand -> powerless"
echo "    不发送新的 FR3 回位命令"
echo "    不自动 Recovery"
echo

read -r -p \
  "确认全部满足后输入 EXECUTE-CYCLE ：" \
  CONFIRM

if [ "$CONFIRM" != "EXECUTE-CYCLE" ]; then
    echo
    echo "[SAFE] operator did not authorize motion"
    echo "No motion command was sent."
    cleanup_fr3
    exit 0
fi


echo
echo "========== 6. 启动联合康复微循环 =========="

sudo -n env \
  HOME="$HOME" \
  XHAND_IF="$XHAND_IF" \
  XHAND_SERIAL="$XHAND_SERIAL" \
  PYTHONUNBUFFERED=1 \
  /bin/bash --noprofile --norc -c '
    source "$HOME/rehab_robot_ws/env.sh"

    exec python3 -m \
      rehab_robot_control.rehab_microcycle_executor \
      --execute
  ' 2>&1 | tee "$EXEC_LOG"

RC="${PIPESTATUS[0]}"

echo
echo "============================================================"
echo "MICROCYCLE_RESULT=$RC"
echo "log=$EXEC_LOG"
echo "============================================================"

cleanup_fr3

exit "$RC"
