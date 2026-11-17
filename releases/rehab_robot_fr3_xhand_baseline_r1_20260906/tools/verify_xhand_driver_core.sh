#!/usr/bin/env bash
set -Eeuo pipefail

cd "$HOME/rehab_robot_ws"
source "$HOME/rehab_robot_ws/env.sh"

CORE="$HOME/rehab_robot_ws/src/rehab_robot_control/rehab_robot_control/xhand_driver_core.py"
TEST="$HOME/rehab_robot_ws/src/rehab_robot_control/test/test_xhand_driver_core.py"

echo "============================================================"
echo "XHand 驱动核心软件静态验收"
echo "============================================================"

python3 -m py_compile "$CORE"
python3 -m py_compile "$TEST"

echo
echo "========== pytest =========="
python3 -m pytest -q "$TEST"

echo
echo "========== import =========="
python3 - <<'PY'
from rehab_robot_control.xhand_driver_core import (
    DEFAULT_COMMAND_TIMEOUT_S,
    DEFAULT_DISABLE_FRAMES,
    VendorXHandBackend,
    XHandDriverCore,
)

assert DEFAULT_COMMAND_TIMEOUT_S == 0.5
assert DEFAULT_DISABLE_FRAMES >= 3

print("[PASS] XHandDriverCore import")
print("[PASS] watchdog =", DEFAULT_COMMAND_TIMEOUT_S, "s")
print("[PASS] disable_frames =", DEFAULT_DISABLE_FRAMES)
print("[PASS] VendorXHandBackend class available")
PY

echo
echo "============================================================"
echo "[PASS] XHand 驱动核心软件验收完成"
echo
echo "本轮只使用 FakeBackend。"
echo "未调用 open_ethercat()。"
echo "未访问任何机器人网卡。"
echo "未启动 XHand / FR3 / D435i。"
echo "未发送任何真实运动命令。"
echo "============================================================"
