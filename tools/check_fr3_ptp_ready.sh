#!/usr/bin/env bash

source "$HOME/rehab_robot_ws/env.sh"

echo "============================================================"
echo "FR3 PTP Action 检查"
echo "本脚本绝不发送 goal"
echo "============================================================"

ros2 interface show \
    franka_msgs/action/PTPMotion \
    2>/dev/null || true

echo
echo "当前 action："

ros2 action list -t 2>/dev/null \
    | grep -Ei 'ptp|motion|franka' \
    || echo "[WAIT] 当前没有 FR3 PTP action server"

echo
echo "============================================================"
echo "没有发送任何 PTPMotion goal"
echo "============================================================"
