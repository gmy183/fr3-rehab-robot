#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

LOGDIR="$WS/validation_logs/runtime"
mkdir -p "$LOGDIR"

echo "============================================================"
echo "XHand Phase-1 FINAL 一键验证"
echo "FR3 不参与"
echo "============================================================"

CONFLICT="$(
  ps -eo pid,args \
    | grep -E \
      'xhand_control_ros2|test_ethercat|phase1_xhand.py|phase1_xhand_whole.py|phase1_xhand_sequential.py|xhand_mode_probe.py' \
    | grep -v grep \
    || true
)"

if [ -n "$CONFLICT" ]; then
  echo "$CONFLICT"
  echo "[FAIL] 已有 XHand 控制进程"
  exit 2
fi

ip -br link show "$XHAND_IF" || exit 2

CARRIER="$(
  cat "/sys/class/net/$XHAND_IF/carrier" \
    2>/dev/null
)"

echo "carrier=$CARRIER"

if [ "$CARRIER" != "1" ]; then
  echo "[FAIL] EtherCAT carrier != 1"
  exit 2
fi

python3 -m py_compile \
  "$WS/tools/xhand_phase1_validate.py" \
  || exit 2

sudo -v || exit 2

LOG="$LOGDIR/xhand_phase1_final_$(date +%Y%m%d_%H%M%S).log"

echo "log=$LOG"
echo
echo "XHand 必须完全空载。"
echo "异常时 Ctrl+C / 急停。"
echo

sudo -n env \
  HOME="$HOME" \
  XHAND_IF="$XHAND_IF" \
  XHAND_SERIAL="$XHAND_SERIAL" \
  PYTHONUNBUFFERED=1 \
  /bin/bash --noprofile --norc -c '
    source "$HOME/rehab_robot_ws/env.sh"
    exec python3 \
      "$HOME/rehab_robot_ws/tools/xhand_phase1_validate.py"
  ' 2>&1 | tee "$LOG"

RC="${PIPESTATUS[0]}"

echo
echo "============================================================"
echo "result=$RC"
echo "log=$LOG"
echo "============================================================"

exit "$RC"
