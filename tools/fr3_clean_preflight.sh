#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"
LOGDIR="$WS/validation_logs/runtime"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

mkdir -p "$LOGDIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$LOGDIR/fr3_clean_preflight_${STAMP}.log"
FR3_LOG="$LOGDIR/fr3_clean_bringup_${STAMP}.log"

STARTED_FR3=0
FR3_PID=""
FR3_PGID=""

exec > >(tee "$LOG") 2>&1


cleanup_fr3()
{
    if [ "$STARTED_FR3" -ne 1 ]; then
        return
    fi

    echo
    echo "========== SAFE CLEANUP =========="

    if [ -n "$FR3_PGID" ]; then
        echo "停止本次 Preflight 启动的 FR3 process group: $FR3_PGID"

        kill -INT -- "-$FR3_PGID" 2>/dev/null || true

        for _ in 1 2 3 4 5 6 7 8; do
            if ! kill -0 -- "-$FR3_PGID" 2>/dev/null; then
                break
            fi
            sleep 0.5
        done

        if kill -0 -- "-$FR3_PGID" 2>/dev/null; then
            kill -TERM -- "-$FR3_PGID" 2>/dev/null || true
            sleep 1
        fi
    fi

    if [ -n "$FR3_PID" ]; then
        wait "$FR3_PID" 2>/dev/null || true
    fi

    echo "[PASS] 本次 FR3 Bringup 已关闭"
}


finish()
{
    RC="$1"

    cleanup_fr3

    echo
    echo "============================================================"

    if [ "$RC" -eq 0 ]; then
        echo "[PASS] FR3 CLEAN LIVE PREFLIGHT"
    else
        echo "[FAIL] FR3 CLEAN LIVE PREFLIGHT"
    fi

    echo "============================================================"
    echo "本轮没有发送任何 PTP goal"
    echo "本轮没有发送任何 XHand command"
    echo "XHand EtherCAT 本轮没有打开"
    echo "D435i 未参与"
    echo "真实联合运动门禁仍然 CLOSED"
    echo
    echo "main_log=$LOG"
    echo "fr3_log=$FR3_LOG"
    echo "============================================================"

    exit "$RC"
}


echo "============================================================"
echo "FR3 CLEAN LIVE PREFLIGHT"
echo "NO MOTION"
echo "============================================================"
echo "main_log=$LOG"
echo "fr3_log=$FR3_LOG"


echo
echo "========== 1. 当前软件环境 =========="

echo "kernel=$(uname -r)"

if [ -r /sys/kernel/realtime ]; then
    echo -n "realtime="
    cat /sys/kernel/realtime
else
    echo "realtime=unknown"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unknown}"

ros2 pkg prefix franka_bringup || {
    echo "[FAIL] franka_bringup missing"
    finish 2
    return 2
}

ros2 pkg prefix franka_hardware || {
    echo "[FAIL] franka_hardware missing"
    finish 2
    return 2
}

ros2 interface show \
    franka_msgs/action/PTPMotion \
    >/dev/null || {
        echo "[FAIL] franka_msgs/action/PTPMotion missing"
        finish 2
        return 2
    }

echo "[PASS] Franka ROS2 software"


echo
echo "========== 2. 当前项目关键配置 =========="

python3 - <<'PY'
from pathlib import Path
import yaml

ws = Path.home() / "rehab_robot_ws"

hp = {}
for line in (
    ws / "config/hardware_profile.env"
).read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#") or "=" not in line:
        continue
    k, v = line.split("=", 1)
    hp[k] = v.strip().strip('"').strip("'")

required = {
    "FR3_IF": "enp4s0",
    "FR3_PC_IP": "192.168.1.100",
    "FR3_ROBOT_IP": "192.168.1.25",
    "FR3_ARM_ID": "fr3",
    "XHAND_IF": "enx6c1ff7641594",
    "XHAND_SERIAL": "012R3201B1217006",
}

for key, expected in required.items():
    actual = hp.get(key)
    print(f"{key}={actual}")
    assert actual == expected, (
        f"{key}: expected {expected}, got {actual}"
    )

profile = yaml.safe_load(
    (
        ws / "config/xhand_motion_profile.yaml"
    ).read_text()
)["xhand"]

g = profile["active_gains"]

print()
print("XHand candidate:")
print("  source   =", g["source"])
print("  kp       =", g["kp"])
print("  ki       =", g["ki"])
print("  kd       =", g["kd"])
print("  verified =", g["verified"])

assert int(g["kp"]) == 225
assert int(g["ki"]) == 0
assert int(g["kd"]) == 0

# 联合执行仍然不能提前开放
assert g["verified"] is False

print()
print("[PASS] hardware profile")
print("[PASS] XHand candidate retained")
print("[PASS] combined motion gate remains CLOSED")
PY

if [ "$?" -ne 0 ]; then
    echo "[FAIL] project configuration mismatch"
    finish 2
    return 2
fi


echo
echo "========== 3. 检查残留 ROS/FR3 会话 =========="

# 关闭 ROS CLI daemon 只会清理 CLI discovery cache，
# 不会停止任何机器人节点。
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

EXISTING_CM="$(
    timeout 3 ros2 node list 2>/dev/null \
      | grep -x '/controller_manager' \
      || true
)"

EXISTING_ACTION="$(
    timeout 3 ros2 action list -t 2>/dev/null \
      | grep 'franka_msgs/action/PTPMotion' \
      || true
)"

if [ -n "$EXISTING_CM" ] || [ -n "$EXISTING_ACTION" ]; then
    echo "[FAIL] 检测到已有 FR3 ROS 会话"
    echo
    echo "--- nodes ---"
    timeout 3 ros2 node list 2>/dev/null | sort || true
    echo
    echo "--- actions ---"
    timeout 3 ros2 action list -t 2>/dev/null | sort || true
    echo
    echo "为避免误杀未知进程，本脚本不会自动清理。"
    echo "把这一段输出发给我即可。"

    finish 2
    return 2
fi

echo "[PASS] 当前没有残留 controller_manager / PTPMotion"


echo
echo "========== 4. FR3 网络 =========="

sudo -v || {
    echo "[FAIL] sudo authorization"
    finish 2
    return 2
}

ip -br link show "$FR3_IF" || {
    echo "[FAIL] FR3 interface missing: $FR3_IF"
    finish 2
    return 2
}

if ! ip -4 addr show dev "$FR3_IF" \
    | grep -q "$FR3_PC_IP/24"; then

    echo "[INFO] 激活现有 rehab-fr3 NetworkManager profile"

    sudo -n nmcli connection up rehab-fr3 || {
        echo "[FAIL] rehab-fr3 profile activation failed"
        finish 2
        return 2
    }
fi

ip -br -4 addr show "$FR3_IF"

if ! ip -4 addr show dev "$FR3_IF" \
    | grep -q "$FR3_PC_IP/24"; then

    echo "[FAIL] expected FR3 PC IP missing: $FR3_PC_IP/24"
    finish 2
    return 2
fi

ping \
    -I "$FR3_IF" \
    -c 3 \
    -W 1 \
    "$FR3_ROBOT_IP"

if [ "$?" -ne 0 ]; then
    echo "[FAIL] FR3 ping"
    finish 2
    return 2
fi

echo "[PASS] FR3 network"


echo
echo "========== 5. XHand 只检查物理链路 =========="

ip -br link show "$XHAND_IF" || {
    echo "[FAIL] XHand interface missing"
    finish 2
    return 2
}

CARRIER="$(
    cat "/sys/class/net/$XHAND_IF/carrier" \
      2>/dev/null
)"

echo "carrier=$CARRIER"

if [ "$CARRIER" != "1" ]; then
    echo "[FAIL] XHand carrier != 1"
    finish 2
    return 2
fi

echo "[PASS] XHand physical link"
echo "[INFO] 本轮不打开 EtherCAT"


echo
echo "========== 6. 启动干净 FR3 Bringup =========="

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
STARTED_FR3=1

sleep 0.3

FR3_PGID="$(
    ps -o pgid= -p "$FR3_PID" \
      | tr -d ' '
)"

echo "fr3_pid=$FR3_PID"
echo "fr3_pgid=$FR3_PGID"

if [ -z "$FR3_PGID" ]; then
    echo "[FAIL] cannot determine FR3 process group"

    tail -n 100 "$FR3_LOG" || true

    finish 2
    return 2
fi


echo
echo "========== 7. 等待完整 FR3 会话 =========="

READY=0
CONTROLLERS=""
ACTIONS=""

for SECOND in $(seq 1 35); do

    if ! kill -0 "$FR3_PID" 2>/dev/null; then
        echo "[FAIL] FR3 launch exited early"
        echo
        tail -n 140 "$FR3_LOG" || true

        finish 2
        return 2
    fi

    CONTROLLERS="$(
        timeout 3 ros2 control list_controllers \
          2>/dev/null \
          || true
    )"

    ACTIONS="$(
        timeout 3 ros2 action list -t \
          2>/dev/null \
          || true
    )"

    HAVE_JS=0
    HAVE_RS=0
    HAVE_PTP=0

    printf '%s\n' "$CONTROLLERS" \
      | awk '$NF=="active" {print $1}' \
      | grep -qx 'joint_state_broadcaster' \
      && HAVE_JS=1

    printf '%s\n' "$CONTROLLERS" \
      | awk '$NF=="active" {print $1}' \
      | grep -qx 'franka_robot_state_broadcaster' \
      && HAVE_RS=1

    printf '%s\n' "$ACTIONS" \
      | grep -q \
        '\[franka_msgs/action/PTPMotion\]' \
      && HAVE_PTP=1

    printf \
      '[WAIT] %02d/35  joint_state=%d  franka_state=%d  PTPMotion=%d\n' \
      "$SECOND" \
      "$HAVE_JS" \
      "$HAVE_RS" \
      "$HAVE_PTP"

    if [ "$HAVE_JS" -eq 1 ] \
       && [ "$HAVE_RS" -eq 1 ] \
       && [ "$HAVE_PTP" -eq 1 ]; then

        READY=1
        break
    fi

    sleep 1
done


echo
echo "--- controllers ---"
printf '%s\n' "$CONTROLLERS"

echo
echo "--- actions ---"
printf '%s\n' "$ACTIONS"


if [ "$READY" -ne 1 ]; then
    echo
    echo "[FAIL] FR3 完整会话 35 秒内未就绪"
    echo
    echo "需要同时存在："
    echo "  joint_state_broadcaster active"
    echo "  franka_robot_state_broadcaster active"
    echo "  franka_msgs/action/PTPMotion"
    echo
    echo "--- FR3 log tail ---"

    tail -n 180 "$FR3_LOG" || true

    finish 2
    return 2
fi

echo
echo "[PASS] joint_state_broadcaster active"
echo "[PASS] franka_robot_state_broadcaster active"
echo "[PASS] PTPMotion Action discovered"


echo
echo "========== 8. 检查没有运动 Controller =========="

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
    echo "[FAIL] unexpected active controller:"
    echo "$BAD_ACTIVE"

    finish 2
    return 2
fi

echo "[PASS] no motion controller active"


echo
echo "========== 9. 读取真实 FR3 当前关节状态 =========="

python3 - <<'PY'
from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

from rehab_robot_control.fr3_ptp_contract import (
    JOINT_NAMES,
    relative_target,
    validate_target,
    load_joint_limits,
)

WS = Path.home() / "rehab_robot_ws"

seq = yaml.safe_load(
    (
        WS / "config/empty_grasp_sequence.yaml"
    ).read_text(encoding="utf-8")
)

fr3 = seq["fr3"]


class Reader(Node):
    def __init__(self):
        super().__init__(
            "rehab_fr3_clean_preflight_reader"
        )

        self.msg = None
        self.times = []

        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self.cb,
            qos_profile_sensor_data,
        )

    def cb(self, msg):
        self.msg = msg
        self.times.append(time.monotonic())

        if len(self.times) > 500:
            self.times = self.times[-500:]


rclpy.init()
node = Reader()

deadline = time.monotonic() + 6.0

while time.monotonic() < deadline:
    rclpy.spin_once(
        node,
        timeout_sec=0.05,
    )

    if node.msg is not None \
       and len(node.times) >= 100:
        break

if node.msg is None:
    raise RuntimeError(
        "/franka/joint_states not received"
    )

msg = node.msg

lookup = {
    name: i
    for i, name in enumerate(msg.name)
}

missing = [
    name
    for name in JOINT_NAMES
    if name not in lookup
]

if missing:
    raise RuntimeError(
        "missing joints: "
        + ", ".join(missing)
    )

q = tuple(
    float(msg.position[lookup[name]])
    for name in JOINT_NAMES
)

if len(msg.velocity) == len(msg.name):
    dq = tuple(
        float(msg.velocity[lookup[name]])
        for name in JOINT_NAMES
    )
else:
    dq = tuple(0.0 for _ in JOINT_NAMES)

if not all(
    math.isfinite(v)
    for v in q + dq
):
    raise RuntimeError(
        "non-finite FR3 state"
    )

q = validate_target(q)

print("live FR3 state:")

for name, pos, vel in zip(
    JOINT_NAMES,
    q,
    dq,
):
    print(
        f"  {name:10s} "
        f"q={pos:+.6f} "
        f"dq={vel:+.6f}"
    )

max_dq = max(abs(v) for v in dq)

print(
    f"max |dq| = {max_dq:.6f} rad/s"
)

# 这里只检查机器人基本静止，
# 不做高频伺服性能判定。
if max_dq > 0.020:
    raise RuntimeError(
        "FR3 is moving too fast for preflight"
    )

if len(node.times) >= 20:
    span = node.times[-1] - node.times[0]

    if span > 0:
        rate = (
            len(node.times) - 1
        ) / span

        print(
            f"receive rate ≈ {rate:.1f} Hz"
        )

# ------------------------------------------------------------
# 未来三个目标只根据实时 q 计算并检查；
# 绝不发送。
# ------------------------------------------------------------

d1 = float(
    fr3["pregrasp_delta_joint1_rad"]
)

d2 = float(
    fr3["grasp_delta_joint1_rad"]
)

d3 = float(
    fr3["rehab_stage_1"][
        "delta_joint1_rad"
    ]
)

q1 = relative_target(
    q,
    joint_index=0,
    delta_rad=d1,
)

q2 = relative_target(
    q1,
    joint_index=0,
    delta_rad=d2,
)

q3 = relative_target(
    q2,
    joint_index=0,
    delta_rad=d3,
)

print()
print("projected targets — NOT SENT:")

print(
    "  q_live      =",
    [round(v, 6) for v in q],
)

print(
    "  q_pregrasp  =",
    [round(v, 6) for v in q1],
)

print(
    "  q_grasp     =",
    [round(v, 6) for v in q2],
)

print(
    "  q_rehab     =",
    [round(v, 6) for v in q3],
)

limits = load_joint_limits()

minimum = None

for name, value in zip(
    JOINT_NAMES,
    q3,
):
    item = limits[name]

    lo = float(item["safe_lower_rad"])
    hi = float(item["safe_upper_rad"])

    margin = min(
        value - lo,
        hi - value,
    )

    if minimum is None \
       or margin < minimum[0]:
        minimum = (
            margin,
            name,
        )

print()
print(
    "minimum projected safe-limit margin = "
    f"{minimum[0]:.6f} rad "
    f"({minimum[1]})"
)

if minimum[0] <= 0:
    raise RuntimeError(
        "projected target outside safe limits"
    )

node.destroy_node()
rclpy.shutdown()

print()
print("[PASS] live joint-state / projected-target check")
print("[PASS] no PTP goal was sent")
PY

RC=$?

if [ "$RC" -ne 0 ]; then
    echo "[FAIL] FR3 live state check"
    finish 2
    return 2
fi


echo
echo "========== 10. 最终状态 =========="

echo "[PASS] FR3 network"
echo "[PASS] FR3 state broadcasters"
echo "[PASS] PTPMotion Action"
echo "[PASS] real joint state"
echo "[PASS] projected targets inside software safe limits"
echo "[PASS] XHand physical link"
echo
echo "XHand 本轮没有打开 EtherCAT。"
echo "FR3 本轮没有发送 PTP goal。"
echo "Production combined-motion gate remains CLOSED."

finish 0
