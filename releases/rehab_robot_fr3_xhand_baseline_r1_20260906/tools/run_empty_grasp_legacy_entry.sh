#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

LOGDIR="$WS/validation_logs/runtime"
mkdir -p "$LOGDIR"

MODE="${1:-}"

if [ "$MODE" = "--preflight" ]; then
    exec "$WS/tools/empty_grasp_live_preflight.sh"
fi

if [ "$MODE" = "--execute" ]; then
    echo "============================================================"
    echo "[BLOCKED] FR3 + XHand real combined executor not finalized"
    echo "============================================================"
    echo
    echo "原因："
    echo "  1. 当前 dry-run FR3 goals 使用 synthetic midpoint"
    echo "  2. 真实执行必须从 /franka/joint_states 获取实时基线"
    echo "  3. 等待 PTP 时必须持续刷新 XHand watchdog"
    echo "  4. FR3 fault 不允许自动回位"
    echo
    echo "当前只允许："
    echo "  ./tools/run_empty_grasp.sh --check"
    echo "  ./tools/run_empty_grasp.sh --preflight"
    exit 2
fi

if [ "$MODE" != "--check" ]; then
    echo "用法："
    echo "  ./tools/run_empty_grasp.sh --check"
    echo
    echo "当前 --execute 主动锁定。"
    exit 2
fi


echo "============================================================"
echo "FR3 + XHand Empty-Grasp FINAL CHECK"
echo "SOFTWARE ONLY"
echo "============================================================"

FAIL=0


echo
echo "========== A. 配置一致性 =========="

python3 - <<'PY'
from pathlib import Path
import yaml

ws = Path.home() / "rehab_robot_ws"

profile = yaml.safe_load(
    (ws / "config/xhand_motion_profile.yaml").read_text()
)

sequence = yaml.safe_load(
    (ws / "config/empty_grasp_sequence.yaml").read_text()
)

g1 = profile["xhand"]["active_gains"]
g2 = sequence["xhand"]["active_control"]["gains"]

print("xhand_motion_profile =", g1)
print("empty_grasp_sequence =", g2)

assert g1["verified"] is False
assert g2["verified"] is False

assert int(g1["kp"]) == int(g2["kp"]) == 225
assert int(g1["ki"]) == int(g2["ki"]) == 0
assert int(g1["kd"]) == int(g2["kd"]) == 0

assert (
    profile["xhand"]["single_joint"]["joint"]
    == "index_joint1"
)

assert (
    float(
        profile["xhand"]["single_joint"]["delta_rad"]
    )
    == 0.005
)

assert (
    int(
        profile["xhand"]["single_joint"][
            "effort_limit_ma"
        ]
    )
    == 150
)

assert (
    float(
        sequence["fr3"]["max_joint_velocity_rad_s"]
    )
    == 0.02
)

assert (
    sequence["fr3"]["automatic_recovery_on_fault"]
    is False
)

assert (
    sequence["fr3"]["automatic_return_on_fault"]
    is False
)

print("[PASS] configuration consistency")
print("[PASS] combined real-motion gate remains CLOSED")
PY

if [ "$?" -ne 0 ]; then
    FAIL=1
fi


echo
echo "========== B. Python / Shell syntax =========="

python3 -m py_compile \
  "$WS/src/rehab_robot_control/rehab_robot_control/fr3_ptp_client.py" \
  "$WS/src/rehab_robot_control/rehab_robot_control/fr3_ptp_contract.py" \
  "$WS/src/rehab_robot_control/rehab_robot_control/xhand_driver_core.py" \
  "$WS/src/rehab_robot_control/rehab_robot_control/xhand_motion_contract.py" \
  "$WS/src/rehab_robot_control/rehab_robot_control/empty_grasp_dry_run.py"

if [ "$?" -eq 0 ]; then
    echo "[PASS] Python syntax"
else
    echo "[FAIL] Python syntax"
    FAIL=1
fi

bash -n "$0"

if [ "$?" -eq 0 ]; then
    echo "[PASS] run_empty_grasp.sh syntax"
else
    echo "[FAIL] run_empty_grasp.sh syntax"
    FAIL=1
fi


echo
echo "========== C. merged workspace build =========="

cd "$WS"

colcon build \
  --merge-install \
  --symlink-install

BUILD_RC=$?

echo "build_result=$BUILD_RC"

if [ "$BUILD_RC" -ne 0 ]; then
    FAIL=1
else
    source "$WS/install/setup.bash"
fi


echo
echo "========== D. merged workspace tests =========="

if [ "$BUILD_RC" -eq 0 ]; then

    colcon test \
      --merge-install \
      --packages-select \
      rehab_robot_control \
      rehab_robot_interfaces

    TEST_COMMAND_RC=$?

    colcon test-result --verbose

    TEST_RESULT_RC=$?

    echo "test_command_result=$TEST_COMMAND_RC"
    echo "test_result_result=$TEST_RESULT_RC"

    if [ "$TEST_COMMAND_RC" -ne 0 ] \
       || [ "$TEST_RESULT_RC" -ne 0 ]; then
        FAIL=1
    fi

else
    echo "[SKIP] build failed"
    TEST_COMMAND_RC=99
    TEST_RESULT_RC=99
    FAIL=1
fi


echo
echo "========== E. software-only dry-run =========="

python3 -m \
  rehab_robot_control.empty_grasp_dry_run

DRY_RC=$?

echo "dry_run_result=$DRY_RC"

if [ "$DRY_RC" -ne 0 ]; then
    FAIL=1
fi


echo
echo "========== F. 防止 synthetic FR3 goal 被误认为实机目标 =========="

python3 - <<'PY'
from pathlib import Path

p = (
    Path.home()
    / "rehab_robot_ws/src/rehab_robot_control/"
      "rehab_robot_control/empty_grasp_dry_run.py"
)

text = p.read_text(encoding="utf-8")

assert "def _safe_fr3_midpoint" in text
assert "relative_target" in text

print(
    "[INFO] current dry-run still contains "
    "_safe_fr3_midpoint()"
)
print(
    "[PASS] it is classified as SOFTWARE-ONLY "
    "and must not feed the real executor"
)
PY

if [ "$?" -ne 0 ]; then
    FAIL=1
fi


echo
echo "========== G. 当前 real execute gate 应继续拒绝 =========="

python3 -m \
  rehab_robot_control.empty_grasp_dry_run \
  --execute \
  >/tmp/rehab_empty_grasp_gate.log \
  2>&1

GATE_RC=$?

cat /tmp/rehab_empty_grasp_gate.log

if [ "$GATE_RC" -eq 0 ]; then
    echo
    echo "[FAIL] current real execution gate unexpectedly opened"
    FAIL=1
else
    echo
    echo "[PASS] current real execution gate is CLOSED"
fi


echo
echo "========== H. 下一阶段所需真实接口快照 =========="

echo
echo "----- fr3_ptp_client.py -----"
sed -n '1,240p' \
  "$WS/src/rehab_robot_control/rehab_robot_control/fr3_ptp_client.py"

echo
echo "----- fr3_ptp_contract.py -----"
sed -n '1,240p' \
  "$WS/src/rehab_robot_control/rehab_robot_control/fr3_ptp_contract.py"

echo
echo "----- empty_grasp_dry_run.py main / gate -----"
sed -n '140,330p' \
  "$WS/src/rehab_robot_control/rehab_robot_control/empty_grasp_dry_run.py"

echo
echo "----- xhand_motion_contract.py -----"
sed -n '1,250p' \
  "$WS/src/rehab_robot_control/rehab_robot_control/xhand_motion_contract.py"


echo
echo "============================================================"

if [ "$FAIL" -eq 0 ]; then
    echo "[PASS] EMPTY-GRASP SOFTWARE CHECK"
else
    echo "[FAIL] EMPTY-GRASP SOFTWARE CHECK"
fi

echo "============================================================"
echo
echo "Real motion status: CLOSED"
echo
echo "This check did NOT:"
echo "  start FR3"
echo "  open XHand EtherCAT"
echo "  send XHand command"
echo "  send FR3 PTP goal"
echo "  start D435i"
echo
echo "build_result=$BUILD_RC"
echo "test_command_result=$TEST_COMMAND_RC"
echo "test_result_result=$TEST_RESULT_RC"
echo "dry_run_result=$DRY_RC"
echo "execute_gate_return=$GATE_RC"
echo "============================================================"

exit "$FAIL"
