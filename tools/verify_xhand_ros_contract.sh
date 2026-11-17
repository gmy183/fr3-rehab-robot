#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

echo "============================================================"
echo "XHand ROS2 compatibility layer software-only verification"
echo "============================================================"

echo
echo "========== interface =========="
ros2 interface show rehab_robot_interfaces/msg/XHandState

echo
echo "========== executable =========="
ros2 pkg executables rehab_robot_control \
  | grep -F "xhand_control_ros2_node"

echo
echo "========== pytest =========="
python3 -m pytest -q \
  "$HOME/rehab_robot_ws/src/rehab_robot_control/test/test_xhand_driver_core.py" \
  "$HOME/rehab_robot_ws/src/rehab_robot_control/test/test_xhand_ros_contract.py"

echo
echo "========== software-only node =========="

LOG="$(mktemp)"
PID=""

cleanup() {
    if [[ -n "${PID:-}" ]] && kill -0 "$PID" 2>/dev/null; then
        # 节点在独立进程组中运行，结束整个软件测试进程组
        kill -TERM -- "-$PID" 2>/dev/null || true

        for _ in $(seq 1 20); do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$PID" 2>/dev/null; then
            kill -KILL -- "-$PID" 2>/dev/null || true
        fi

        wait "$PID" 2>/dev/null || true
    fi

    rm -f "$LOG"
}

trap cleanup EXIT INT TERM

# setsid 创建独立进程组，避免 ros2 run 子进程退出时无限 wait
setsid ros2 run rehab_robot_control xhand_control_ros2_node \
  --ros-args \
  -p hardware_enable:=false \
  -p interface:=enx6c1ff7641594 \
  -p state_rate_hz:=100.0 \
  -p watchdog_s:=0.5 \
  >"$LOG" 2>&1 &

PID=$!

READY=0

for _ in $(seq 1 40); do
    if ros2 node list 2>/dev/null \
        | grep -Fxq "/xhand_control/xhand_control_ros2"; then
        READY=1
        break
    fi

    if ! kill -0 "$PID" 2>/dev/null; then
        break
    fi

    sleep 0.1
done

if [[ "$READY" -ne 1 ]]; then
    cat "$LOG"
    echo "[FAIL] XHand ROS2 software-only node did not appear"
    exit 1
fi

echo "[PASS] node=/xhand_control/xhand_control_ros2"

TYPE="$(ros2 topic type /xhand_control/xhand_state 2>/dev/null || true)"

if [[ "$TYPE" != "rehab_robot_interfaces/msg/XHandState" ]]; then
    cat "$LOG"
    echo "[FAIL] unexpected topic type: $TYPE"
    exit 1
fi

echo "[PASS] topic=/xhand_control/xhand_state"
echo "[PASS] type=$TYPE"

GUARD_OK=1

if ! grep -Fq "hardware_enable=false" "$LOG"; then
    echo "[FAIL] hardware_enable=false log missing"
    GUARD_OK=0
fi

if ! grep -Fq "state_source=disabled" "$LOG"; then
    echo "[FAIL] state_source=disabled log missing"
    GUARD_OK=0
fi

if ! grep -Fq "command interface is disabled in this node" "$LOG"; then
    echo "[FAIL] command-disabled log missing"
    GUARD_OK=0
fi

if grep -Fq "REAL XHand READ-ONLY state source enabled" "$LOG"; then
    echo "[FAIL] disabled test unexpectedly entered real hardware path"
    GUARD_OK=0
fi

if [[ "$GUARD_OK" -ne 1 ]]; then
    cat "$LOG"
    exit 1
fi

echo "[PASS] hardware_enable=false"
echo "[PASS] state_source=disabled"
echo "[PASS] command interface disabled"
echo "[PASS] real vendor_readonly path not entered"

echo
cat "$LOG"

echo
echo "========== bounded shutdown =========="

kill -TERM -- "-$PID" 2>/dev/null || true

STOPPED=0

for _ in $(seq 1 20); do
    if ! kill -0 "$PID" 2>/dev/null; then
        STOPPED=1
        break
    fi
    sleep 0.1
done

if [[ "$STOPPED" -ne 1 ]]; then
    echo "[WARN] normal shutdown timeout; forcing test process exit"
    kill -KILL -- "-$PID" 2>/dev/null || true
fi

wait "$PID" 2>/dev/null || true
PID=""

echo "[PASS] software-only node stopped"

echo
echo "============================================================"
echo "[PASS] XHand ROS2 compatibility layer verified"
echo
echo "No EtherCAT interface was opened."
echo "No XHand command was sent."
echo "No FR3 command was sent."
echo "No D435i node was started."
echo "============================================================"
