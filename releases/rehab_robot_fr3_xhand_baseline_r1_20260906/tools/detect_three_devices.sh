#!/usr/bin/env bash

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

echo "============================================================"
echo "康复机械臂：三设备识别"
echo "============================================================"

echo
echo "========== FR3 =========="

echo "预设网卡：$FR3_IF"

ip -brief link show "$FR3_IF" 2>/dev/null \
    || echo "[WAIT] FR3 网卡当前不存在"

ip -brief address show "$FR3_IF" 2>/dev/null \
    || true

if [ -n "${FR3_ROBOT_IP:-}" ]; then
    echo "FR3_ROBOT_IP=$FR3_ROBOT_IP"
else
    echo "[WAIT] FR3_ROBOT_IP 尚未确认"
fi

echo
echo "Franka ROS2："

ros2 pkg prefix franka_bringup 2>/dev/null \
    || echo "[FAIL] franka_bringup"

ros2 pkg prefix franka_hardware 2>/dev/null \
    || echo "[FAIL] franka_hardware"

ros2 interface show franka_msgs/action/PTPMotion \
    >/dev/null 2>&1 \
    && echo "[PASS] franka_msgs/action/PTPMotion" \
    || echo "[FAIL] PTPMotion"


echo
echo "========== XHand =========="

echo "预设网卡：$XHAND_IF"

ip -brief link show "$XHAND_IF" 2>/dev/null \
    || echo "[WAIT] XHand USB/EtherCAT 网卡当前不存在"

python3 - <<'PY'
try:
    import xhand_controller
    from xhand_controller.xhand_control import XHandControl
    print("[PASS] xhand_controller")
    print("       ", xhand_controller.__file__)
except Exception as exc:
    print("[FAIL] XHand SDK:", exc)
PY


echo
echo "========== D435i =========="

ros2 pkg prefix realsense2_camera 2>/dev/null \
    || echo "[FAIL] realsense2_camera"

if command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo
    echo "RealSense 设备："
    rs-enumerate-devices -s 2>/dev/null \
        || echo "[WAIT] 当前没有检测到 RealSense"
else
    echo "[INFO] rs-enumerate-devices 命令不存在"
fi

if [ -n "${D435I_SERIAL:-}" ]; then
    echo "D435I_SERIAL=$D435I_SERIAL"
else
    echo "[WAIT] D435I_SERIAL 尚未确认"
fi


echo
echo "============================================================"
echo "本脚本只读取设备状态"
echo "没有启动 FR3"
echo "没有 open_ethercat"
echo "没有 XHand send_command"
echo "没有启动 D435i ROS 节点"
echo "============================================================"
