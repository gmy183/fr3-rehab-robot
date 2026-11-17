#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"

WS="$HOME/rehab_robot_ws"
RUNTIME="$WS/.runtime"

mkdir -p "$RUNTIME"

echo "============================================================"
echo "三设备统一启动"
echo "FR3     : official bringup / no motion goal"
echo "XHand   : vendor_readonly / no command"
echo "D435i   : color + depth / IMU off"
echo "============================================================"

bash "$WS/tools/preflight_three_devices.sh" --hardware

bash "$WS/tools/stop_all_devices.sh"

echo
echo "[START] XHand"

setsid bash "$WS/tools/start_xhand_readonly.sh" --hardware \
    >"$RUNTIME/xhand.log" 2>&1 &

echo $! > "$RUNTIME/xhand.pid"


echo "[START] D435i"

setsid bash "$WS/tools/start_d435i.sh" --hardware \
    >"$RUNTIME/d435i.log" 2>&1 &

echo $! > "$RUNTIME/d435i.pid"


echo "[START] FR3"

setsid bash "$WS/tools/start_fr3_state.sh" --hardware \
    >"$RUNTIME/fr3.log" 2>&1 &

echo $! > "$RUNTIME/fr3.pid"


sleep 3

OK=1

for name in xhand d435i fr3
do
    PID="$(cat "$RUNTIME/$name.pid")"

    if kill -0 "$PID" 2>/dev/null; then
        echo "[PASS] $name running pid=$PID"
    else
        echo "[FAIL] $name 启动失败"
        echo "------ $name.log ------"
        cat "$RUNTIME/$name.log" 2>/dev/null || true
        OK=0
    fi
done

if [ "$OK" -ne 1 ]; then
    echo
    echo "[FAIL] 三设备没有全部启动成功，执行统一停止"
    bash "$WS/tools/stop_all_devices.sh"
    exit 1
fi

echo
echo "============================================================"
echo "[PASS] 三设备 ROS 进程均已启动"
echo
echo "状态："
echo "  bash tools/status_all_devices.sh"
echo
echo "停止："
echo "  bash tools/stop_all_devices.sh"
echo "============================================================"
