#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_interface "$XHAND_IF"

echo "============================================================"
echo "启动 XHand：REAL READ-ONLY"
echo "============================================================"
echo "interface=$XHAND_IF"
echo "state rate=$XHAND_STATE_RATE_HZ Hz"
echo
echo "本模式没有 command subscriber"
echo "本模式没有 send_command API"
echo "============================================================"

exec ros2 run \
    rehab_robot_control \
    xhand_control_ros2_node \
    --ros-args \
    -p hardware_enable:=true \
    -p state_source:=vendor_readonly \
    -p interface:="$XHAND_IF" \
    -p state_rate_hz:="$XHAND_STATE_RATE_HZ" \
    -p watchdog_s:="$XHAND_WATCHDOG_S"
