#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

LOGDIR="$WS/validation_logs/runtime"
mkdir -p "$LOGDIR"

LOG="$LOGDIR/empty_grasp_preflight_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee "$LOG") 2>&1

echo "============================================================"
echo "FR3 + XHand EMPTY-GRASP LIVE PREFLIGHT"
echo "READ-ONLY / NO MOTION"
echo "============================================================"
echo "log=$LOG"

STARTED_FR3=0
FR3_PID=""


cleanup_fr3()
{
    if [ "$STARTED_FR3" -ne 1 ]; then
        return
    fi

    if [ -z "$FR3_PID" ]; then
        return
    fi

    if ! kill -0 "$FR3_PID" 2>/dev/null; then
        wait "$FR3_PID" 2>/dev/null || true
        return
    fi

    echo
    echo "[SAFE] stopping preflight-started FR3 bringup"

    kill -INT "$FR3_PID" 2>/dev/null || true

    for _ in 1 2 3 4 5 6; do
        if ! kill -0 "$FR3_PID" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done

    if kill -0 "$FR3_PID" 2>/dev/null; then
        kill -TERM "$FR3_PID" 2>/dev/null || true
    fi

    wait "$FR3_PID" 2>/dev/null || true

    echo "[PASS] preflight FR3 bringup stopped"
}


finish()
{
    RC="$1"

    cleanup_fr3

    echo
    echo "============================================================"

    if [ "$RC" -eq 0 ]; then
        echo "[PASS] EMPTY-GRASP LIVE PREFLIGHT"
    else
        echo "[FAIL] EMPTY-GRASP LIVE PREFLIGHT"
    fi

    echo "============================================================"
    echo "Real combined motion gate: CLOSED"
    echo "No FR3 PTP goal was sent."
    echo "No XHand send_command() API call was made."
    echo "D435i was not started."
    echo "log=$LOG"

    exit "$RC"
}


echo
echo "========== 0. 软件总检查 =========="

"$WS/tools/run_empty_grasp.sh" --check

RC=$?

if [ "$RC" -ne 0 ]; then
    echo "[FAIL] software check"
    finish 2
fi

echo "[PASS] software check"


echo
echo "========== 1. sudo =========="

sudo -v

if [ "$?" -ne 0 ]; then
    echo "[FAIL] sudo authorization"
    finish 2
fi

echo "[PASS] sudo"


echo
echo "========== 2. FR3 网络 =========="

ip -br link show "$FR3_IF" || {
    echo "[FAIL] FR3 interface missing"
    finish 2
}

if ! ip -4 addr show dev "$FR3_IF" \
    | grep -q "$FR3_PC_IP/24"; then

    echo "[INFO] rehab-fr3 当前未激活，尝试启用已有连接配置"

    sudo -n nmcli connection up rehab-fr3

    if [ "$?" -ne 0 ]; then
        echo "[FAIL] cannot activate existing rehab-fr3 profile"
        finish 2
    fi
fi

ip -br -4 addr show "$FR3_IF"

if ! ip -4 addr show dev "$FR3_IF" \
    | grep -q "$FR3_PC_IP/24"; then

    echo "[FAIL] expected FR3 PC IP not present: $FR3_PC_IP"
    finish 2
fi

ping -I "$FR3_IF" \
    -c 3 \
    -W 1 \
    "$FR3_ROBOT_IP"

if [ "$?" -ne 0 ]; then
    echo "[FAIL] FR3 ping"
    finish 2
fi

echo "[PASS] FR3 network"


echo
echo "========== 3. XHand EtherCAT 网络 =========="

ip -br link show "$XHAND_IF" || {
    echo "[FAIL] XHand interface missing"
    finish 2
}

CARRIER="$(
    cat \
      "/sys/class/net/$XHAND_IF/carrier" \
      2>/dev/null
)"

echo "carrier=$CARRIER"

if [ "$CARRIER" != "1" ]; then
    echo "[FAIL] XHand carrier != 1"
    finish 2
fi

echo "[PASS] XHand physical link"


echo
echo "========== 4. XHand 冲突进程 =========="

CONFLICT="$(
    ps -eo pid,args \
      | grep -E \
        'xhand_control_ros2|test_ethercat|xhand_phase1_validate.py|phase1_xhand.py' \
      | grep -v grep \
      || true
)"

if [ -n "$CONFLICT" ]; then
    echo "$CONFLICT"
    echo "[FAIL] another XHand process is active"
    finish 2
fi

echo "[PASS] no conflicting XHand process"


echo
echo "========== 5. FR3 Bringup =========="

EXISTING_CM="$(
    timeout 3 ros2 node list 2>/dev/null \
      | grep -x '/controller_manager' \
      || true
)"

if [ -n "$EXISTING_CM" ]; then

    echo "[INFO] existing /controller_manager found"
    echo "[INFO] preflight will use existing FR3 session"

else

    FR3_LOG="$LOGDIR/fr3_empty_grasp_preflight_$(date +%Y%m%d_%H%M%S).log"

    echo "starting official franka.launch.py"
    echo "fr3_log=$FR3_LOG"

    ros2 launch \
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
    STARTED_FR3=1

    echo "fr3_launcher_pid=$FR3_PID"

    READY=0

    for _ in $(seq 1 24); do

        if ! kill -0 "$FR3_PID" 2>/dev/null; then
            break
        fi

        if timeout 2 ros2 node list 2>/dev/null \
            | grep -qx '/controller_manager'; then
            READY=1
            break
        fi

        sleep 0.5
    done

    if [ "$READY" -ne 1 ]; then
        echo "[FAIL] FR3 controller_manager did not appear"
        echo
        tail -n 80 "$FR3_LOG"
        finish 2
    fi

    echo "[PASS] FR3 bringup running"
fi


echo
echo "========== 6. 等待 FR3 状态 broadcaster 真正就绪 =========="

CONTROLLERS_READY=0
CONTROLLERS=""

# controller_manager 会先出现，两个 broadcaster 随后由 spawner 加载。
# 因此这里等待“active”，不能只等待 controller_manager 节点存在。
for SECOND in $(seq 1 30); do

    if [ "$STARTED_FR3" -eq 1 ] \
       && ! kill -0 "$FR3_PID" 2>/dev/null; then

        echo "[FAIL] FR3 bringup process exited before controllers became ready"

        echo
        echo "--- FR3 log tail ---"
        tail -n 120 "$FR3_LOG"

        finish 2
    fi

    CONTROLLERS="$(
        timeout 4 \
          ros2 control list_controllers \
          2>&1 \
          || true
    )"

    ACTIVE_CONTROLLERS="$(
        printf '%s\n' "$CONTROLLERS" \
          | awk '$NF=="active" {print $1}' \
          | sort
    )"

    HAVE_JS=0
    HAVE_STATE=0

    if printf '%s\n' "$ACTIVE_CONTROLLERS" \
        | grep -qx 'joint_state_broadcaster'; then
        HAVE_JS=1
    fi

    if printf '%s\n' "$ACTIVE_CONTROLLERS" \
        | grep -qx 'franka_robot_state_broadcaster'; then
        HAVE_STATE=1
    fi

    if [ "$HAVE_JS" -eq 1 ] \
       && [ "$HAVE_STATE" -eq 1 ]; then

        CONTROLLERS_READY=1
        break
    fi

    printf '[WAIT] %02d/30 s  joint_state=%s  franka_state=%s\n' \
      "$SECOND" \
      "$HAVE_JS" \
      "$HAVE_STATE"

    sleep 1
done

echo
echo "--- final controller state ---"
printf '%s\n' "$CONTROLLERS"

if [ "$CONTROLLERS_READY" -ne 1 ]; then

    echo
    echo "[FAIL] required state broadcasters did not become active within 30 s"

    if [ "$STARTED_FR3" -eq 1 ]; then
        echo
        echo "--- FR3 log tail ---"
        tail -n 120 "$FR3_LOG"
    fi

    finish 2
fi

ACTIVE_CONTROLLERS="$(
    printf '%s\n' "$CONTROLLERS" \
      | awk '$NF=="active" {print $1}' \
      | sort
)"

BAD_ACTIVE="$(
    printf '%s\n' "$ACTIVE_CONTROLLERS" \
      | grep -vE \
        '^(joint_state_broadcaster|franka_robot_state_broadcaster)$' \
      || true
)"

if [ -n "$BAD_ACTIVE" ]; then
    echo "[FAIL] unexpected active controller(s):"
    echo "$BAD_ACTIVE"
    finish 2
fi

echo "[PASS] joint_state_broadcaster active"
echo "[PASS] franka_robot_state_broadcaster active"
echo "[PASS] no motion controller is active"


echo
echo "========== 7. FR3 live q + PTP Action + 未来目标检查 =========="

python3 \
  "$WS/tools/empty_grasp_fr3_preflight.py"

RC=$?

if [ "$RC" -ne 0 ]; then
    echo
    echo "[FAIL] FR3 live preflight"

    if [ "$STARTED_FR3" -eq 1 ]; then
        echo
        echo "--- FR3 log tail ---"
        tail -n 80 "$FR3_LOG"
    fi

    finish 2
fi


echo
echo "========== 8. XHand strict read-only =========="

sudo -n env \
  HOME="$HOME" \
  XHAND_IF="$XHAND_IF" \
  XHAND_SERIAL="$XHAND_SERIAL" \
  PYTHONUNBUFFERED=1 \
  /bin/bash --noprofile --norc -c '
      source "$HOME/rehab_robot_ws/env.sh"
      exec python3 \
        "$HOME/rehab_robot_ws/tools/empty_grasp_xhand_readonly_preflight.py"
  '

RC=$?

if [ "$RC" -ne 0 ]; then
    echo "[FAIL] XHand read-only preflight"
    finish 2
fi


echo
echo "========== 9. 最终确认：正式运动门禁仍关闭 =========="

python3 - <<'PY'
from pathlib import Path
import yaml

p = (
    Path.home()
    / "rehab_robot_ws/config/xhand_motion_profile.yaml"
)

g = yaml.safe_load(
    p.read_text(encoding="utf-8")
)["xhand"]["active_gains"]

print("source   =", g["source"])
print("kp       =", g["kp"])
print("ki       =", g["ki"])
print("kd       =", g["kd"])
print("verified =", g["verified"])

assert g["verified"] is False

print("[PASS] production combined-motion gate remains CLOSED")
PY

RC=$?

if [ "$RC" -ne 0 ]; then
    finish 2
fi


finish 0
