#!/usr/bin/env bash

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

MODE="${1:-software}"

OK=1

echo "============================================================"
echo "三设备 Preflight：$MODE"
echo "============================================================"

for pkg in \
    franka_bringup \
    franka_hardware \
    franka_msgs \
    realsense2_camera \
    rehab_robot_control \
    rehab_robot_interfaces
do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "[PASS] ROS package: $pkg"
    else
        echo "[FAIL] ROS package: $pkg"
        OK=0
    fi
done

python3 - <<'PY'
from xhand_controller.xhand_control import XHandControl
from rehab_robot_control.xhand_readonly_session import XHandReadOnlySession
from rehab_robot_interfaces.msg import XHandState

print("[PASS] XHand vendor SDK import")
print("[PASS] XHandReadOnlySession import")
print("[PASS] XHandState import")
PY

if [ "$MODE" = "--hardware" ]; then
    echo
    echo "========== hardware =========="

    if ip link show "$FR3_IF" >/dev/null 2>&1; then
        echo "[PASS] FR3 interface: $FR3_IF"
    else
        echo "[FAIL] FR3 interface missing: $FR3_IF"
        OK=0
    fi

    if ip link show "$XHAND_IF" >/dev/null 2>&1; then
        echo "[PASS] XHand interface: $XHAND_IF"
    else
        echo "[FAIL] XHand interface missing: $XHAND_IF"
        OK=0
    fi

    if [ -n "${FR3_ROBOT_IP:-}" ]; then
        echo "[PASS] FR3_ROBOT_IP=$FR3_ROBOT_IP"
    else
        echo "[FAIL] FR3_ROBOT_IP 尚未写入 hardware_profile.env"
        OK=0
    fi
fi

echo
echo "============================================================"

if [ "$OK" -eq 1 ]; then
    echo "[PASS] preflight"
else
    echo "[FAIL] preflight"
fi

echo "============================================================"

[ "$OK" -eq 1 ]
