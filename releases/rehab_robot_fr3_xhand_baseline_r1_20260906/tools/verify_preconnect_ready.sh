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
