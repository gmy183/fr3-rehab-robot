#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

WS="$HOME/rehab_robot_ws"

echo "============================================================"
echo "康复机械臂三设备 VSCode Bundle 软件验收"
echo "============================================================"

echo
echo "========== Bash syntax =========="

for f in \
    "$WS/tools/detect_three_devices.sh" \
    "$WS/tools/preflight_three_devices.sh" \
    "$WS/tools/start_xhand_readonly.sh" \
    "$WS/tools/start_d435i.sh" \
    "$WS/tools/start_fr3_state.sh" \
    "$WS/tools/start_all_readonly.sh" \
    "$WS/tools/stop_all_devices.sh" \
    "$WS/tools/status_all_devices.sh" \
    "$WS/tools/check_fr3_ptp_ready.sh"
do
    bash -n "$f"
    echo "[PASS] $(basename "$f")"
done


echo
echo "========== ROS packages =========="

for pkg in \
    franka_bringup \
    franka_hardware \
    franka_msgs \
    realsense2_camera \
    rehab_robot_bringup \
    rehab_robot_control \
    rehab_robot_interfaces
do
    ros2 pkg prefix "$pkg" >/dev/null
    echo "[PASS] $pkg"
done


echo
echo "========== FR3 PTP interface =========="

ros2 interface show \
    franka_msgs/action/PTPMotion \
    >/dev/null

echo "[PASS] franka_msgs/action/PTPMotion"


echo
echo "========== XHand regression =========="

bash "$WS/tools/verify_xhand_sdk_adapter.sh"
bash "$WS/tools/verify_xhand_driver_core.sh"
bash "$WS/tools/verify_xhand_ros_contract.sh"
bash "$WS/tools/verify_xhand_fake_state_publish.sh"


echo
echo "========== three-device detection =========="

bash "$WS/tools/detect_three_devices.sh"


echo
echo "============================================================"
echo "[PASS] 三设备 VSCode 软件包验收完成"
echo
echo "没有启动真实 FR3"
echo "没有启动真实 XHand EtherCAT"
echo "没有启动真实 D435i"
echo "没有发送任何运动命令"
echo "============================================================"
