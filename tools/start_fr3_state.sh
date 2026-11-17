#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_interface "$FR3_IF"
require_ros_package franka_bringup

if [ -z "${FR3_ROBOT_IP:-}" ]; then
    echo "[FAIL] FR3_ROBOT_IP 尚未确认。"
    echo
    echo "请先连接 FR3 后运行："
    echo "  bash tools/detect_three_devices.sh"
    echo
    echo "确认真实机器人 IP 后写入："
    echo "  config/hardware_profile.env"
    exit 1
fi

PREFIX="$(ros2 pkg prefix franka_bringup)"
LAUNCH_DIR="$PREFIX/share/franka_bringup/launch"

SELECTED=""
SELECTED_ARGS=""

while IFS= read -r path
do
    name="$(basename "$path")"

    args="$(
        ros2 launch \
            franka_bringup \
            "$name" \
            --show-args \
            2>&1 || true
    )"

    if grep -q 'robot_ip' <<<"$args"; then

        if [ "$name" = "franka.launch.py" ]; then
            SELECTED="$name"
            SELECTED_ARGS="$args"
            break
        fi

        if [ -z "$SELECTED" ]; then
            SELECTED="$name"
            SELECTED_ARGS="$args"
        fi
    fi

done < <(
    find "$LAUNCH_DIR" \
        -maxdepth 1 \
        -type f \
        -name '*.launch.py' \
        | sort
)

if [ -z "$SELECTED" ]; then
    echo "[FAIL] 未找到包含 robot_ip 参数的 Franka 官方 launch"
    echo
    echo "当前 launch 文件："
    find "$LAUNCH_DIR" -maxdepth 1 -name '*.launch.py' -printf '%f\n'
    exit 1
fi

CMD=(
    ros2 launch
    franka_bringup
    "$SELECTED"
    "robot_ip:=$FR3_ROBOT_IP"
)

if grep -q 'arm_id' <<<"$SELECTED_ARGS"; then
    CMD+=("arm_id:=$FR3_ARM_ID")
fi

if grep -q 'robot_type' <<<"$SELECTED_ARGS"; then
    CMD+=("robot_type:=fr3")
fi

if grep -q 'load_gripper' <<<"$SELECTED_ARGS"; then
    CMD+=("load_gripper:=false")
fi

if grep -q 'use_rviz' <<<"$SELECTED_ARGS"; then
    CMD+=("use_rviz:=false")
fi

echo "============================================================"
echo "启动 FR3 官方 Bringup"
echo "============================================================"
echo "interface=$FR3_IF"
echo "robot_ip=$FR3_ROBOT_IP"
echo "launch=$SELECTED"
echo
echo "本脚本不发送 PTPMotion goal"
echo "============================================================"

printf ' %q' "${CMD[@]}"
echo

exec "${CMD[@]}"
