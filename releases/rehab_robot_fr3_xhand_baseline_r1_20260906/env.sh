#!/usr/bin/env bash

# ============================================================
# Rehab Robot ROS 2 environment
#
# Software environment only.
# This file does NOT:
#   - configure network interfaces
#   - start FR3
#   - start XHand
#   - open EtherCAT
#   - start D435i
#   - send any motion command
# ============================================================

export REHAB_ROBOT_WS="$HOME/rehab_robot_ws"

# ROS/ament setup scripts are not guaranteed to be compatible with
# Bash nounset ("set -u"). If the caller enabled nounset, temporarily
# disable it while loading ROS overlays, then restore it.
_REHAB_RESTORE_NOUNSET=0

case $- in
    *u*)
        _REHAB_RESTORE_NOUNSET=1
        set +u
        ;;
esac

# ROS 2 Jazzy base
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Franka ROS 2 overlay
if [ -f "$HOME/franka_ros2_ws/install/setup.bash" ]; then
    source "$HOME/franka_ros2_ws/install/setup.bash"
fi

# Rehab robot overlay
if [ -f "$HOME/rehab_robot_ws/install/setup.bash" ]; then
    source "$HOME/rehab_robot_ws/install/setup.bash"
fi

# Project-validated ROS 2 middleware.
# CycloneDDS is required for the XHand root -> normal-user ROS 2 path
# and has been validated simultaneously with FR3 and XHand hardware.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ "$_REHAB_RESTORE_NOUNSET" -eq 1 ]; then
    set -u
fi

unset _REHAB_RESTORE_NOUNSET
