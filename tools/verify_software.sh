#!/usr/bin/env bash
set -Eeuo pipefail
source "$HOME/rehab_robot_ws/env.sh"

ros2 run rehab_robot_control rehab_config_validator
ros2 run rehab_robot_control rehab_runtime_probe

ros2 pkg prefix franka_bringup
ros2 pkg prefix franka_hardware
ros2 pkg prefix franka_fr3_moveit_config
ros2 pkg prefix rehab_robot_control
ros2 pkg prefix rehab_robot_bringup
ros2 pkg prefix realsense2_camera
ros2 interface show franka_msgs/action/PTPMotion >/dev/null

echo "[PASS] software-only verification complete"
echo "No hardware node was started and no motion command was sent."
