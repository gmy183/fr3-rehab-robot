#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/env.sh"

WS="$HOME/rehab_robot_ws"
TOOLS="$WS/tools"
CONFIG="$WS/config"
VSCODE="$WS/.vscode"
RUNTIME="$WS/.runtime"

mkdir -p "$TOOLS" "$CONFIG" "$VSCODE" "$RUNTIME"


echo "============================================================"
echo "1. 创建统一硬件配置文件"
echo "============================================================"

if [ ! -f "$CONFIG/hardware_profile.env" ]; then
cat > "$CONFIG/hardware_profile.env" <<'EOF'
# ============================================================
# 康复机械臂三设备唯一硬件配置
#
# 注意：
# FR3_ROBOT_IP 和 D435I_SERIAL 在真实设备连接后确认。
# 不在无设备状态下猜测。
# ============================================================

FR3_IF=enp4s0
FR3_ROBOT_IP=
FR3_ARM_ID=fr3

XHAND_IF=enx6c1ff7641594

D435I_SERIAL=

XHAND_STATE_RATE_HZ=100.0
XHAND_WATCHDOG_S=0.5

D435I_ENABLE_COLOR=true
D435I_ENABLE_DEPTH=true
D435I_ENABLE_IMU=false
EOF
    echo "[PASS] 新建 hardware_profile.env"
else
    echo "[PASS] hardware_profile.env 已存在，未覆盖"
fi


echo
echo "============================================================"
echo "2. 创建公共加载脚本"
echo "============================================================"

cat > "$TOOLS/_device_common.sh" <<'SH'
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
SH

chmod +x "$TOOLS/_device_common.sh"


echo
echo "============================================================"
echo "3. 创建设备识别脚本（完全只读）"
echo "============================================================"

cat > "$TOOLS/detect_three_devices.sh" <<'SH'
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
SH

chmod +x "$TOOLS/detect_three_devices.sh"


echo
echo "============================================================"
echo "4. 创建统一硬件预检"
echo "============================================================"

cat > "$TOOLS/preflight_three_devices.sh" <<'SH'
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
SH

chmod +x "$TOOLS/preflight_three_devices.sh"


echo
echo "============================================================"
echo "5. 创建 XHand 真实只读启动器"
echo "============================================================"

cat > "$TOOLS/start_xhand_readonly.sh" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_interface "$XHAND_IF"

echo "============================================================"
echo "启动 XHand：REAL READ-ONLY"
echo "============================================================"
echo "interface=$XHAND_IF"
echo "state rate=$XHAND_STATE_RATE_HZ Hz"
echo
echo "本模式没有 command subscriber"
echo "本模式没有 send_command API"
echo "============================================================"

exec ros2 run \
    rehab_robot_control \
    xhand_control_ros2_node \
    --ros-args \
    -p hardware_enable:=true \
    -p state_source:=vendor_readonly \
    -p interface:="$XHAND_IF" \
    -p state_rate_hz:="$XHAND_STATE_RATE_HZ" \
    -p watchdog_s:="$XHAND_WATCHDOG_S"
SH

chmod +x "$TOOLS/start_xhand_readonly.sh"


echo
echo "============================================================"
echo "6. 创建 D435i 启动器"
echo "============================================================"

cat > "$TOOLS/start_d435i.sh" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_ros_package realsense2_camera

PREFIX="$(ros2 pkg prefix realsense2_camera)"
LAUNCH_DIR="$PREFIX/share/realsense2_camera/launch"

if [ -f "$LAUNCH_DIR/rs_launch.py" ]; then
    LAUNCH_FILE="rs_launch.py"
else
    LAUNCH_FILE="$(
        find "$LAUNCH_DIR" \
            -maxdepth 1 \
            -type f \
            -name '*.launch.py' \
            -printf '%f\n' \
            | head -n 1
    )"
fi

if [ -z "${LAUNCH_FILE:-}" ]; then
    echo "[FAIL] 找不到 realsense2_camera launch 文件"
    exit 1
fi

SHOW_ARGS="$(
    ros2 launch \
        realsense2_camera \
        "$LAUNCH_FILE" \
        --show-args \
        2>&1 || true
)"

CMD=(
    ros2 launch
    realsense2_camera
    "$LAUNCH_FILE"
)

if [ -n "${D435I_SERIAL:-}" ] \
    && grep -q 'serial_no' <<<"$SHOW_ARGS"; then
    CMD+=("serial_no:=$D435I_SERIAL")
fi

if grep -q 'enable_color' <<<"$SHOW_ARGS"; then
    CMD+=("enable_color:=true")
fi

if grep -q 'enable_depth' <<<"$SHOW_ARGS"; then
    CMD+=("enable_depth:=true")
fi

# 当前恢复阶段 D435i IMU 明确保持关闭。
if grep -q 'enable_gyro' <<<"$SHOW_ARGS"; then
    CMD+=("enable_gyro:=false")
fi

if grep -q 'enable_accel' <<<"$SHOW_ARGS"; then
    CMD+=("enable_accel:=false")
fi

echo "============================================================"
echo "启动 D435i"
echo "============================================================"
echo "launch=$LAUNCH_FILE"
echo "color=true"
echo "depth=true"
echo "IMU=false"

if [ -n "${D435I_SERIAL:-}" ]; then
    echo "serial=$D435I_SERIAL"
else
    echo "serial=未固定（只连接一个 RealSense 时可直接启动）"
fi

echo "============================================================"

printf ' %q' "${CMD[@]}"
echo

exec "${CMD[@]}"
SH

chmod +x "$TOOLS/start_d435i.sh"


echo
echo "============================================================"
echo "7. 创建 FR3 官方状态 Bringup 启动器"
echo "============================================================"

cat > "$TOOLS/start_fr3_state.sh" <<'SH'
#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/tools/_device_common.sh"

require_hardware_flag "${1:-}"
require_interface "$FR3_IF"
require_ros_package franka_bringup

if [ -z "${FR3_ROBOT_IP:-}" ]; then
    echo "[FAIL] FR3_ROBOT_IP 尚未确认。"
    echo
    echo "请先连接 FR3 后运行："
    echo "  bash tools/detect_three_devices.sh"
    echo
    echo "确认真实机器人 IP 后写入："
    echo "  config/hardware_profile.env"
    exit 1
fi

PREFIX="$(ros2 pkg prefix franka_bringup)"
LAUNCH_DIR="$PREFIX/share/franka_bringup/launch"

SELECTED=""
SELECTED_ARGS=""

while IFS= read -r path
do
    name="$(basename "$path")"

    args="$(
        ros2 launch \
            franka_bringup \
            "$name" \
            --show-args \
            2>&1 || true
    )"

    if grep -q 'robot_ip' <<<"$args"; then

        if [ "$name" = "franka.launch.py" ]; then
            SELECTED="$name"
            SELECTED_ARGS="$args"
            break
        fi

        if [ -z "$SELECTED" ]; then
            SELECTED="$name"
            SELECTED_ARGS="$args"
        fi
    fi

done < <(
    find "$LAUNCH_DIR" \
        -maxdepth 1 \
        -type f \
        -name '*.launch.py' \
        | sort
)

if [ -z "$SELECTED" ]; then
    echo "[FAIL] 未找到包含 robot_ip 参数的 Franka 官方 launch"
    echo
    echo "当前 launch 文件："
    find "$LAUNCH_DIR" -maxdepth 1 -name '*.launch.py' -printf '%f\n'
    exit 1
fi

CMD=(
    ros2 launch
    franka_bringup
    "$SELECTED"
    "robot_ip:=$FR3_ROBOT_IP"
)

if grep -q 'arm_id' <<<"$SELECTED_ARGS"; then
    CMD+=("arm_id:=$FR3_ARM_ID")
fi

if grep -q 'robot_type' <<<"$SELECTED_ARGS"; then
    CMD+=("robot_type:=fr3")
fi

if grep -q 'load_gripper' <<<"$SELECTED_ARGS"; then
    CMD+=("load_gripper:=false")
fi

if grep -q 'use_rviz' <<<"$SELECTED_ARGS"; then
    CMD+=("use_rviz:=false")
fi

echo "============================================================"
echo "启动 FR3 官方 Bringup"
echo "============================================================"
echo "interface=$FR3_IF"
echo "robot_ip=$FR3_ROBOT_IP"
echo "launch=$SELECTED"
echo
echo "本脚本不发送 PTPMotion goal"
echo "============================================================"

printf ' %q' "${CMD[@]}"
echo

exec "${CMD[@]}"
SH

chmod +x "$TOOLS/start_fr3_state.sh"


echo
echo "============================================================"
echo "8. 创建三设备统一停止器"
echo "============================================================"

cat > "$TOOLS/stop_all_devices.sh" <<'SH'
#!/usr/bin/env bash

RUNTIME="$HOME/rehab_robot_ws/.runtime"

echo "============================================================"
echo "停止康复机械臂三设备 ROS 进程"
echo "============================================================"

for name in xhand d435i fr3
do
    PIDFILE="$RUNTIME/$name.pid"

    if [ ! -f "$PIDFILE" ]; then
        echo "[INFO] $name：没有 pid 文件"
        continue
    fi

    PID="$(cat "$PIDFILE" 2>/dev/null)"

    if [ -z "$PID" ]; then
        rm -f "$PIDFILE"
        continue
    fi

    if kill -0 "$PID" 2>/dev/null; then
        echo "[STOP] $name pid=$PID"

        kill -TERM -- "-$PID" 2>/dev/null || \
        kill -TERM "$PID" 2>/dev/null || true

        for _ in $(seq 1 30)
        do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$PID" 2>/dev/null; then
            echo "[WARN] $name 强制结束"
            kill -KILL -- "-$PID" 2>/dev/null || \
            kill -KILL "$PID" 2>/dev/null || true
        fi
    else
        echo "[INFO] $name 已停止"
    fi

    rm -f "$PIDFILE"
done

echo
echo "[PASS] stop_all_devices 完成"
SH

chmod +x "$TOOLS/stop_all_devices.sh"


echo
echo "============================================================"
echo "9. 创建三设备一起启动器"
echo "============================================================"

cat > "$TOOLS/start_all_readonly.sh" <<'SH'
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
SH

chmod +x "$TOOLS/start_all_readonly.sh"


echo
echo "============================================================"
echo "10. 创建统一状态查看器"
echo "============================================================"

cat > "$TOOLS/status_all_devices.sh" <<'SH'
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
SH

chmod +x "$TOOLS/status_all_devices.sh"


echo
echo "============================================================"
echo "11. 创建 FR3 PTP 只读检查器"
echo "============================================================"

cat > "$TOOLS/check_fr3_ptp_ready.sh" <<'SH'
#!/usr/bin/env bash

source "$HOME/rehab_robot_ws/env.sh"

echo "============================================================"
echo "FR3 PTP Action 检查"
echo "本脚本绝不发送 goal"
echo "============================================================"

ros2 interface show \
    franka_msgs/action/PTPMotion \
    2>/dev/null || true

echo
echo "当前 action："

ros2 action list -t 2>/dev/null \
    | grep -Ei 'ptp|motion|franka' \
    || echo "[WAIT] 当前没有 FR3 PTP action server"

echo
echo "============================================================"
echo "没有发送任何 PTPMotion goal"
echo "============================================================"
SH

chmod +x "$TOOLS/check_fr3_ptp_ready.sh"


echo
echo "============================================================"
echo "12. 创建三设备软件总验收"
echo "============================================================"

cat > "$TOOLS/verify_three_device_bundle.sh" <<'SH'
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
SH

chmod +x "$TOOLS/verify_three_device_bundle.sh"


echo
echo "============================================================"
echo "13. 创建 VSCode Tasks"
echo "============================================================"

python3 - <<'PY'
import json
from pathlib import Path

ws = Path.home() / "rehab_robot_ws"
path = ws / ".vscode/tasks.json"

if path.exists():
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        data = {"version": "2.0.0", "tasks": []}
else:
    data = {"version": "2.0.0", "tasks": []}

data["version"] = "2.0.0"

new_tasks = [
    {
        "label": "00｜三设备软件总验收",
        "type": "shell",
        "command": "bash tools/verify_three_device_bundle.sh",
    },
    {
        "label": "01｜识别三个设备（只读）",
        "type": "shell",
        "command": "bash tools/detect_three_devices.sh",
    },
    {
        "label": "10｜XHand｜真实只读启动",
        "type": "shell",
        "command": "bash tools/start_xhand_readonly.sh --hardware",
    },
    {
        "label": "11｜D435i｜Color+Depth",
        "type": "shell",
        "command": "bash tools/start_d435i.sh --hardware",
    },
    {
        "label": "12｜FR3｜官方状态 Bringup",
        "type": "shell",
        "command": "bash tools/start_fr3_state.sh --hardware",
    },
    {
        "label": "20｜三设备｜一起启动（无运动）",
        "type": "shell",
        "command": "bash tools/start_all_readonly.sh --hardware",
    },
    {
        "label": "21｜三设备｜查看状态",
        "type": "shell",
        "command": "bash tools/status_all_devices.sh",
    },
    {
        "label": "22｜FR3｜检查 PTP Action（不发送Goal）",
        "type": "shell",
        "command": "bash tools/check_fr3_ptp_ready.sh",
    },
    {
        "label": "90｜停止全部设备",
        "type": "shell",
        "command": "bash tools/stop_all_devices.sh",
    },
]

labels = {x["label"] for x in new_tasks}

old_tasks = [
    x for x in data.get("tasks", [])
    if x.get("label") not in labels
]

for task in new_tasks:
    task["options"] = {
        "cwd": "${workspaceFolder}"
    }
    task["presentation"] = {
        "reveal": "always",
        "panel": "dedicated",
        "clear": False,
    }
    task["problemMatcher"] = []

data["tasks"] = old_tasks + new_tasks

path.parent.mkdir(parents=True, exist_ok=True)

path.write_text(
    json.dumps(
        data,
        ensure_ascii=False,
        indent=2,
    ) + "\n",
    encoding="utf-8",
)

print("[PASS]", path)
PY


echo
echo "============================================================"
echo "14. 创建 VSCode Workspace"
echo "============================================================"

cat > "$WS/rehab_robot.code-workspace" <<'JSON'
{
    "folders": [
        {
            "name": "康复机械臂",
            "path": "."
        }
    ],
    "settings": {
        "terminal.integrated.cwd": "${workspaceFolder}"
    }
}
JSON


echo
echo "============================================================"
echo "15. 创建 VSCode 首页说明"
echo "============================================================"

cat > "$WS/README_START_HERE.md" <<'MD'
# 康复机械臂

当前统一工作空间：

`~/rehab_robot_ws`

## 三设备

FR3  
XHand  
Intel RealSense D435i

## VSCode 操作

按：

`Ctrl + Shift + P`

输入：

`Tasks: Run Task`

主要任务：

- `00｜三设备软件总验收`
- `01｜识别三个设备（只读）`
- `10｜XHand｜真实只读启动`
- `11｜D435i｜Color+Depth`
- `12｜FR3｜官方状态 Bringup`
- `20｜三设备｜一起启动（无运动）`
- `21｜三设备｜查看状态`
- `22｜FR3｜检查 PTP Action（不发送Goal）`
- `90｜停止全部设备`

## 当前硬件安全阶段

XHand：

`vendor_readonly`

不提供 command subscriber。

FR3：

只运行官方 bringup，不发送 PTPMotion goal。

D435i：

Color + Depth。

IMU 暂时关闭。

## 唯一硬件配置文件

`config/hardware_profile.env`

真实设备连接后确认：

`FR3_ROBOT_IP`

`D435I_SERIAL`

在确认前不要猜测。

## 后续运动恢复顺序

三设备只读稳定

→ XHand 单关节 0.005 rad

→ XHand 整手 0.005 rad

→ FR3 PTP 小运动

→ FR3 + XHand empty grasp

→ D435i 参与感知
MD


echo
echo "============================================================"
echo "16. 最终软件验收"
echo "============================================================"

bash "$TOOLS/verify_three_device_bundle.sh"


echo
echo "============================================================"
echo "[PASS] 三设备 VSCode 工程封装完成"
echo
echo "Workspace:"
echo "  $WS/rehab_robot.code-workspace"
echo
echo "硬件配置:"
echo "  $CONFIG/hardware_profile.env"
echo
echo "当前没有启动任何真实设备"
echo "============================================================"
