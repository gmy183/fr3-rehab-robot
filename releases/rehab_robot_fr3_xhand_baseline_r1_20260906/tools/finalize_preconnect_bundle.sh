#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/env.sh"
source "$HOME/rehab_robot_ws/config/hardware_profile.env"

WS="$HOME/rehab_robot_ws"
TOOLS="$WS/tools"
VSCODE="$WS/.vscode"

echo "============================================================"
echo "1. 修正 Motion Gate 过期提示"
echo "============================================================"

python3 - <<'PY'
from pathlib import Path

path = Path.home() / "rehab_robot_ws/tools/verify_motion_config_gate.sh"
text = path.read_text(encoding="utf-8")

old = '''echo "运动层当前仍被两个条件锁住："
echo "  1. FR3 官方 joint limits / Action server 名称尚未运行时确认"
echo "  2. XHand 项目主动 kp/ki/kd 数值尚未可靠恢复"
echo
echo "因此当前没有任何真实运动入口。"
'''

new = '''echo "[PASS] FR3 官方 joint limits 已从当前 franka_description 解析"
echo "[PASS] FR3 PTPMotion Goal 软件契约已确认"
echo "[INFO] PTP Action server 名称将在 FR3 启动后按"
echo "       franka_msgs/action/PTPMotion 唯一发现"
echo "[BLOCKED] XHand 真实运动仍由 active-control gate 锁定"
echo
echo "因此当前仍不会发送任何真实运动命令。"
'''

if old in text:
    text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print("[PASS] Motion Gate 状态文字已更新")
else:
    print("[INFO] 原提示块已经不存在，不重复修改")
PY


echo
echo "============================================================"
echo "2. 保存双网口 NetworkManager 模板"
echo "   注意：只保存，不激活"
echo "============================================================"

sudo -v

if sudo nmcli -t -f NAME connection show \
    | grep -Fxq "rehab-fr3"
then
    sudo nmcli connection modify \
        rehab-fr3 \
        connection.interface-name "$FR3_IF" \
        connection.autoconnect no \
        ipv4.method manual \
        ipv4.addresses "$FR3_PC_IP/24" \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.never-default yes \
        ipv6.method disabled
else
    sudo nmcli connection add \
        type ethernet \
        ifname "$FR3_IF" \
        con-name rehab-fr3 \
        connection.autoconnect no \
        ipv4.method manual \
        ipv4.addresses "$FR3_PC_IP/24" \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.never-default yes \
        ipv6.method disabled
fi

sudo nmcli connection modify \
    rehab-fr3 \
    802-3-ethernet.mtu 1500

if sudo nmcli -t -f NAME connection show \
    | grep -Fxq "rehab-xhand-ethercat"
then
    sudo nmcli connection modify \
        rehab-xhand-ethercat \
        connection.interface-name "$XHAND_IF" \
        connection.autoconnect no \
        ipv4.method disabled \
        ipv6.method disabled
else
    sudo nmcli connection add \
        type ethernet \
        ifname "$XHAND_IF" \
        con-name rehab-xhand-ethercat \
        connection.autoconnect no \
        ipv4.method disabled \
        ipv6.method disabled
fi

# 如果旧恢复过程中还残留 rehab-xhand，
# 只关闭它的自动连接，不删除。
if sudo nmcli -t -f NAME connection show \
    | grep -Fxq "rehab-xhand"
then
    sudo nmcli connection modify \
        rehab-xhand \
        connection.autoconnect no
fi

echo "[PASS] rehab-fr3 配置已保存"
echo "[PASS] rehab-xhand-ethercat 配置已保存"
echo "[PASS] 两个连接均未激活"


echo
echo "============================================================"
echo "3. 固定 D435i 最终启动器"
echo "============================================================"

cat > "$TOOLS/start_d435i.sh" <<'SH'
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
    serial_no:="$D435I_SERIAL" \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=848x480x30 \
    enable_gyro:=false \
    enable_accel:=false
SH

chmod +x "$TOOLS/start_d435i.sh"


echo
echo "============================================================"
echo "4. 创建离线上机前最终验收"
echo "============================================================"

cat > "$TOOLS/verify_preconnect_ready.sh" <<'SH'
#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"
source "$HOME/rehab_robot_ws/config/hardware_profile.env"

WS="$HOME/rehab_robot_ws"

echo "============================================================"
echo "康复机械臂：上机前离线总验收"
echo "============================================================"

echo
echo "========== Hardware identity =========="

[[ "$FR3_IF" == "enp4s0" ]]
[[ "$FR3_PC_IP" == "192.168.1.100" ]]
[[ "$FR3_ROBOT_IP" == "192.168.1.25" ]]

[[ "$XHAND_IF" == "enx6c1ff7641594" ]]

[[ "$D435I_SERIAL" == "244222072086" ]]

echo "[PASS] FR3 interface=$FR3_IF"
echo "[PASS] FR3 PC=$FR3_PC_IP"
echo "[PASS] FR3 robot=$FR3_ROBOT_IP"
echo "[PASS] XHand interface=$XHAND_IF"
echo "[PASS] D435i serial=$D435I_SERIAL"


echo
echo "========== Saved network profiles =========="

FR3_IF_SAVED="$(
    nmcli -g connection.interface-name \
        connection show rehab-fr3
)"

FR3_METHOD="$(
    nmcli -g ipv4.method \
        connection show rehab-fr3
)"

FR3_ADDR="$(
    nmcli -g ipv4.addresses \
        connection show rehab-fr3
)"

FR3_AUTO="$(
    nmcli -g connection.autoconnect \
        connection show rehab-fr3
)"

XH_IF_SAVED="$(
    nmcli -g connection.interface-name \
        connection show rehab-xhand-ethercat
)"

XH_IPV4="$(
    nmcli -g ipv4.method \
        connection show rehab-xhand-ethercat
)"

XH_AUTO="$(
    nmcli -g connection.autoconnect \
        connection show rehab-xhand-ethercat
)"

[[ "$FR3_IF_SAVED" == "$FR3_IF" ]]
[[ "$FR3_METHOD" == "manual" ]]
[[ "$FR3_ADDR" == *"$FR3_PC_IP/24"* ]]
[[ "$FR3_AUTO" == "no" ]]

[[ "$XH_IF_SAVED" == "$XHAND_IF" ]]
[[ "$XH_IPV4" == "disabled" ]]
[[ "$XH_AUTO" == "no" ]]

echo "[PASS] rehab-fr3 = $FR3_PC_IP/24"
echo "[PASS] rehab-fr3 autoconnect=no"
echo "[PASS] rehab-xhand-ethercat = no IP"
echo "[PASS] rehab-xhand-ethercat autoconnect=no"


echo
echo "========== RealSense launch contract =========="

ARGS="$(
    ros2 launch \
        realsense2_camera \
        rs_launch.py \
        --show-args \
        2>&1
)"

for arg in \
    serial_no \
    enable_color \
    enable_depth \
    rgb_camera.color_profile \
    depth_module.depth_profile \
    enable_gyro \
    enable_accel
do
    if grep -Fq "$arg" <<<"$ARGS"; then
        echo "[PASS] $arg"
    else
        echo "[FAIL] RealSense launch arg missing: $arg"
        exit 1
    fi
done


echo
echo "========== Runtime config =========="

bash "$WS/tools/verify_project_runtime_config.sh"


echo
echo "========== Motion gate =========="

bash "$WS/tools/verify_motion_config_gate.sh"


echo
echo "========== XHand software =========="

bash "$WS/tools/verify_xhand_ros_contract.sh"
bash "$WS/tools/verify_xhand_fake_state_publish.sh"


echo
echo "========== Motion dry-run =========="

bash "$WS/tools/verify_motion_dryrun_stack.sh"


echo
echo "============================================================"
echo "[PASS] 上机前离线配置全部通过"
echo
echo "FR3 网络模板：已保存，未激活"
echo "XHand EtherCAT模板：已保存，未激活"
echo "D435i：启动参数已固定"
echo "FR3 PTP：软件契约已完成"
echo "XHand：真实执行仍 fail-closed"
echo
echo "没有启动 FR3"
echo "没有打开 XHand EtherCAT"
echo "没有启动 D435i"
echo "没有发送任何运动命令"
echo "============================================================"
SH

chmod +x "$TOOLS/verify_preconnect_ready.sh"


echo
echo "============================================================"
echo "5. 加入 VSCode 最终任务"
echo "============================================================"

python3 - <<'PY'
import json
from pathlib import Path

path = Path.home() / "rehab_robot_ws/.vscode/tasks.json"
data = json.loads(path.read_text(encoding="utf-8"))

new_tasks = [
    {
        "label": "40｜上机前离线总验收",
        "type": "shell",
        "command": "bash tools/verify_preconnect_ready.sh",
    },
    {
        "label": "41｜查看双网口模板",
        "type": "shell",
        "command": (
            "nmcli connection show rehab-fr3; "
            "echo; "
            "nmcli connection show rehab-xhand-ethercat"
        ),
    },
]

labels = {x["label"] for x in new_tasks}

tasks = [
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

tasks.extend(new_tasks)

data["tasks"] = tasks

path.write_text(
    json.dumps(
        data,
        ensure_ascii=False,
        indent=2,
    ) + "\n",
    encoding="utf-8",
)

print("[PASS] VSCode tasks 40/41")
PY


echo
echo "============================================================"
echo "6. Shell syntax"
echo "============================================================"

bash -n "$TOOLS/start_d435i.sh"
bash -n "$TOOLS/verify_preconnect_ready.sh"

echo "[PASS] shell syntax"


echo
echo "============================================================"
echo "7. 执行最终离线验收"
echo "============================================================"

bash "$TOOLS/verify_preconnect_ready.sh"

echo
echo "============================================================"
echo "[PASS] 三设备上机前配置封装完成"
echo "============================================================"
