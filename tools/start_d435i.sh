#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_ros_package realsense2_camera

echo "============================================================"
echo "D435i"
echo "============================================================"
echo "serial=$D435I_SERIAL"
echo "color=640x480x30"
echo "depth=848x480x30"
echo "gyro=false"
echo "accel=false"
echo "============================================================"

exec ros2 launch \
    realsense2_camera \
    rs_launch.py \
    serial_no:="_${D435I_SERIAL}" \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=848x480x30 \
    align_depth.enable:=true \
    enable_sync:=true \
    enable_gyro:=false \
    enable_accel:=false
