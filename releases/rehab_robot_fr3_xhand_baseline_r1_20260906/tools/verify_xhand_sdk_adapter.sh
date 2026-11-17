#!/usr/bin/env bash
set -Eeuo pipefail

cd "$HOME/rehab_robot_ws"
source "$HOME/rehab_robot_ws/env.sh"

echo "============================================================"
echo "XHand 项目层 SDK 适配器静态验收"
echo "============================================================"

python3 -m py_compile \
  "$HOME/rehab_robot_ws/src/rehab_robot_control/rehab_robot_control/xhand_sdk_adapter.py"

python3 \
  "$HOME/rehab_robot_ws/src/rehab_robot_control/rehab_robot_control/xhand_sdk_adapter.py"

echo
echo "========== vendor version =========="
dpkg-query -W -f='xhand=${Version}\n' xhand

python3 - <<'PY'
import xhand_controller
import xhand_controller.xhand_control as xc

print("python package =", xhand_controller.__file__)
print("binary module  =", xc.__file__)
PY

echo
echo "============================================================"
echo "[PASS] XHand SDK 项目层适配器验收完成"
echo "没有打开 EtherCAT，没有启动设备，没有发送运动命令。"
echo "============================================================"
