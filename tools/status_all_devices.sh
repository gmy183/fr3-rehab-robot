#!/usr/bin/env bash

source "$HOME/rehab_robot_ws/env.sh"

RUNTIME="$HOME/rehab_robot_ws/.runtime"

echo "============================================================"
echo "三设备运行状态"
echo "============================================================"

for name in xhand d435i fr3
do
    PIDFILE="$RUNTIME/$name.pid"

    if [ -f "$PIDFILE" ]; then
        PID="$(cat "$PIDFILE")"

        if kill -0 "$PID" 2>/dev/null; then
            echo "[RUNNING] $name pid=$PID"
        else
            echo "[STOPPED] $name"
        fi
    else
        echo "[NONE] $name"
    fi
done

echo
echo "========== ROS nodes =========="

ros2 node list 2>/dev/null \
    | grep -Ei \
      'franka|xhand|camera|realsense|controller' \
    || true

echo
echo "========== ROS topics =========="

ros2 topic list 2>/dev/null \
    | grep -Ei \
      'franka|joint_states|xhand|camera|color|depth' \
    || true

echo
echo "========== XHand =========="

TYPE="$(
    ros2 topic type \
      /xhand_control/xhand_state \
      2>/dev/null || true
)"

if [ -n "$TYPE" ]; then
    echo "/xhand_control/xhand_state : $TYPE"
else
    echo "[WAIT] /xhand_control/xhand_state"
fi

echo
echo "============================================================"
