#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

CONTROL="$HOME/rehab_robot_ws/src/rehab_robot_control"

echo "============================================================"
echo "Motion dry-run stack verification"
echo "============================================================"

python3 -m py_compile \
  "$CONTROL/rehab_robot_control/fr3_ptp_contract.py" \
  "$CONTROL/rehab_robot_control/fr3_ptp_client.py" \
  "$CONTROL/rehab_robot_control/xhand_motion_contract.py" \
  "$CONTROL/rehab_robot_control/empty_grasp_dry_run.py"

echo "[PASS] syntax"

echo
echo "========== pytest =========="

python3 -m pytest -q \
  "$CONTROL/test/test_fr3_ptp_contract.py" \
  "$CONTROL/test/test_xhand_motion_contract.py" \
  "$CONTROL/test/test_empty_grasp_dry_run.py"

echo
echo "========== Empty-Grasp dry run =========="

ros2 run \
  rehab_robot_control \
  rehab_empty_grasp_dry_run

echo
echo "========== Real execute gate =========="

LOG="$(mktemp)"

set +e

ros2 run \
  rehab_robot_control \
  rehab_empty_grasp_dry_run \
  --execute \
  >"$LOG" 2>&1

RC=$?

set -e

cat "$LOG"

if grep -Fq \
  "XHand active kp/ki/kd are not verified" \
  "$LOG"
then
    echo "[PASS] real execution correctly blocked"
else
    echo "[FAIL] real execution gate reason missing"
    rm -f "$LOG"
    exit 1
fi

if [ "$RC" -eq 0 ]; then
    echo "[FAIL] --execute unexpectedly succeeded"
    rm -f "$LOG"
    exit 1
fi

rm -f "$LOG"

echo
echo "============================================================"
echo "[PASS] Motion dry-run stack verified"
echo
echo "No FR3 PTP goal sent."
echo "No XHand command sent."
echo "No EtherCAT opened."
echo "No D435i started."
echo "============================================================"
