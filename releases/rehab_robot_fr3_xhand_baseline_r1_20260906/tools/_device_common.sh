#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

require_hardware_flag()
{
    if [ "${1:-}" != "--hardware" ]; then
        echo
        echo "[BLOCKED] 这是实际硬件启动脚本。"
        echo
        echo "只有真正连接设备后才允许："
        echo "  $0 --hardware"
        echo
        return 2
    fi
}

require_interface()
{
    local ifname="$1"

    if ! ip link show "$ifname" >/dev/null 2>&1; then
        echo "[FAIL] 找不到网卡：$ifname"
        return 1
    fi
}

require_ros_package()
{
    local package="$1"

    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
        echo "[FAIL] ROS2 包不存在：$package"
        return 1
    fi
}
