#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"
LOGDIR="$WS/validation_logs/runtime"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

mkdir -p "$LOGDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"

LOG="$LOGDIR/fr3_robot_description_diag_${STAMP}.log"
FR3_LOG="$LOGDIR/fr3_robot_description_bringup_${STAMP}.log"

PID=""
PGID=""
STARTED=0

exec > >(tee "$LOG") 2>&1


cleanup()
{
    if [ "$STARTED" -ne 1 ]; then
        return
    fi

    echo
    echo "========== SAFE CLEANUP =========="

    if [ -n "$PGID" ]; then
        kill -INT -- "-$PGID" \
          2>/dev/null || true

        for _ in 1 2 3 4 5 6 7 8; do
            if ! kill -0 -- "-$PGID" \
                2>/dev/null; then
                break
            fi

            sleep 0.5
        done

        if kill -0 -- "-$PGID" \
            2>/dev/null; then

            kill -TERM -- "-$PGID" \
              2>/dev/null || true

            sleep 1
        fi
    fi

    if [ -n "$PID" ]; then
        wait "$PID" \
          2>/dev/null || true
    fi

    echo "[PASS] 本轮 FR3 诊断 Bringup 已关闭"
}


trap cleanup EXIT INT TERM


echo "============================================================"
echo "FR3 robot_description DIAGNOSTIC"
echo "READ-ONLY / NO MOTION"
echo "============================================================"

echo "log=$LOG"
echo "fr3_log=$FR3_LOG"


echo
echo "========== A. 基础环境 =========="

echo "kernel=$(uname -r)"
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "RMW=${RMW_IMPLEMENTATION:-unknown}"

echo
echo "franka_bringup:"
ros2 pkg prefix franka_bringup

echo
echo "franka_description:"
ros2 pkg prefix franka_description

echo
echo "controller_manager:"
ros2 pkg prefix controller_manager


echo
echo "========== B. 当前官方 launch 中 robot_description 相关代码 =========="

PREFIX="$(
    ros2 pkg prefix franka_bringup
)"

LAUNCH="$PREFIX/share/franka_bringup/launch/franka.launch.py"

echo "launch=$LAUNCH"

if [ ! -f "$LAUNCH" ]; then
    echo "[FAIL] franka.launch.py missing"
    exit 2
fi

grep -nE \
  'robot_description|robot_state_publisher|ros2_control|remap|xacro|franka_description' \
  "$LAUNCH" \
  || true


echo
echo "========== C. 启动前确认无残留 FR3 =========="

ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

NODES="$(
    timeout 4 ros2 node list \
      2>/dev/null \
      || true
)"

printf '%s\n' "$NODES"

if printf '%s\n' "$NODES" \
    | grep -qx '/controller_manager'; then

    echo
    echo "[FAIL] 已存在 /controller_manager"
    echo "本脚本不会自动杀未知会话"
    exit 2
fi

echo "[PASS] no existing controller_manager"


echo
echo "========== D. 网络只读确认 =========="

ip -br -4 addr show "$FR3_IF"

ping \
  -I "$FR3_IF" \
  -c 2 \
  -W 1 \
  "$FR3_ROBOT_IP"

if [ "$?" -ne 0 ]; then
    echo "[FAIL] FR3 network"
    exit 2
fi

echo "[PASS] FR3 network"


echo
echo "========== E. 启动官方 Franka Bringup =========="

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

PID=$!
STARTED=1

sleep 0.3

PGID="$(
    ps -o pgid= -p "$PID" \
      | tr -d ' '
)"

echo "pid=$PID"
echo "pgid=$PGID"

if [ -z "$PGID" ]; then
    echo "[FAIL] cannot determine process group"
    exit 2
fi


echo
echo "========== F. 等待 12 秒观察初始化 =========="

for SECOND in $(seq 1 12); do

    if ! kill -0 "$PID" \
        2>/dev/null; then

        echo "[FAIL] launch exited early"
        break
    fi

    printf '[WAIT] %02d/12\n' "$SECOND"

    sleep 1
done


echo
echo "========== G. 当前 ROS nodes =========="

timeout 5 ros2 node list \
  2>/dev/null \
  | sort \
  || true


echo
echo "========== H. 当前 robot_description topic =========="

timeout 5 ros2 topic list -t \
  2>/dev/null \
  | grep -E \
      'robot_description|joint_states|franka' \
  || true

echo

timeout 5 ros2 topic info \
  /robot_description \
  --verbose \
  2>&1 \
  || true


echo
echo "========== I. robot_description 实际消息检查 =========="

python3 - <<'PY'
import time

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class Reader(Node):

    def __init__(self):
        super().__init__(
            "rehab_robot_description_probe"
        )

        self.message = None

        qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.create_subscription(
            String,
            "/robot_description",
            self.cb,
            qos,
        )

    def cb(self, msg):
        self.message = msg.data


rclpy.init()

node = Reader()

deadline = time.monotonic() + 4.0

while (
    time.monotonic() < deadline
    and node.message is None
):
    rclpy.spin_once(
        node,
        timeout_sec=0.2,
    )

if node.message is None:

    print(
        "[FAIL] /robot_description "
        "message NOT received"
    )

else:

    print(
        "[PASS] /robot_description "
        "message received"
    )

    print(
        "URDF length =",
        len(node.message),
        "characters",
    )

    print(
        "starts_with =",
        repr(
            node.message[:80]
        ),
    )

node.destroy_node()
rclpy.shutdown()
PY


echo
echo "========== J. robot_state_publisher 参数 =========="

if timeout 4 ros2 node list \
    2>/dev/null \
    | grep -qx '/robot_state_publisher'; then

    echo "[PASS] /robot_state_publisher exists"

    echo
    echo "--- parameters ---"

    timeout 5 ros2 param list \
      /robot_state_publisher \
      2>&1 \
      | grep -E \
          'robot_description|use_sim_time' \
      || true

    echo
    echo "--- robot_description parameter byte count ---"

    TMP="/tmp/rehab_robot_description_param_${STAMP}.txt"

    timeout 6 ros2 param get \
      /robot_state_publisher \
      robot_description \
      >"$TMP" \
      2>&1 \
      || true

    wc -c "$TMP"

    head -c 180 "$TMP" || true
    echo

    rm -f "$TMP"

else

    echo "[FAIL] /robot_state_publisher node NOT present"
fi


echo
echo "========== K. controller_manager 状态 =========="

timeout 5 ros2 service list \
  2>/dev/null \
  | grep '/controller_manager' \
  || true

echo

timeout 5 ros2 control list_controllers \
  2>&1 \
  || true


echo
echo "========== L. PTP Action 当前状态 =========="

timeout 5 ros2 action list -t \
  2>/dev/null \
  | grep -E \
      'PTPMotion|franka' \
  || true


echo
echo "========== M. 关键进程 =========="

ps -eo pid,ppid,pgid,args \
  | grep -E \
      'franka.launch.py|robot_state_publisher|ros2_control_node|spawner|action_server' \
  | grep -v grep \
  || true


echo
echo "========== N. FR3 日志中的关键行 =========="

grep -nEi \
  'robot_description|robot_state_publisher|xacro|error|fatal|exception|action server|action_server|franka_hardware|controller_manager' \
  "$FR3_LOG" \
  | tail -n 260 \
  || true


echo
echo "========== O. FR3 日志开头 =========="

head -n 180 "$FR3_LOG" \
  || true


echo
echo "============================================================"
echo "诊断完成"
echo "============================================================"
echo "本轮："
echo "  没有发送 PTP goal"
echo "  没有打开 XHand EtherCAT"
echo "  没有发送 XHand command"
echo "  没有启动 D435i"
echo
echo "diag_log=$LOG"
echo "fr3_log=$FR3_LOG"
echo "============================================================"
