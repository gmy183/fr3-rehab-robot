#!/usr/bin/env bash

set +e

WS="$HOME/rehab_robot_ws"

source "$HOME/robot_env/rehab.sh"

FR3_RUNNER="$WS/teach/run_v21_bidirectional.sh"
XHAND_RUNTIME="$WS/tools/xhand_grasp_hold_at_p8.py"

echo "========================================================================"
echo "FR3 MANUAL RESTORE"
echo
echo "P8 -> P7 -> P6 -> P5 -> P4 -> P3 -> P2 -> P1"
echo
echo "XHand must already be POWERLESS."
echo "NO retry / NO ErrorRecovery / NO automatic fallback"
echo "========================================================================"

if [ ! -f "$FR3_RUNNER" ]; then
    echo "[FAIL] missing FR3 runner"
    exit 2
fi

# 防止 XHand 仍在主动保持时启动 FR3 恢复。
if pgrep -f "$XHAND_RUNTIME" >/dev/null 2>&1; then
    echo
    echo "[BLOCKED] XHand grasp hold is still active."
    echo "Return to the combined terminal."
    echo "Press ENTER there first to make XHand powerless."
    exit 2
fi


echo
echo "========== FR3 OFFICIAL SESSION =========="

"$WS/tools/ensure_fr3_moveit_session.sh"
SESSION_RC=$?

if [ "$SESSION_RC" -ne 0 ]; then
    echo "[FAIL] FR3 official session unavailable"
    exit "$SESSION_RC"
fi

echo
echo "Required:"
echo "  FR3 currently at P8"
echo "  XHand POWERLESS"
echo "  workspace clear"
echo "  E-stop reachable"
echo

read -r -p \
"Type exactly RESTORE-FR3-P8-P1 : " \
CONFIRM

if [ "$CONFIRM" != "RESTORE-FR3-P8-P1" ]; then
    echo "[ABORT]"
    exit 1
fi

printf '%s\n' "EXECUTE-P8-P1" \
| bash "$FR3_RUNNER" 8to1

RC="${PIPESTATUS[1]}"

echo
echo "RESTORE_RESULT=$RC"

if [ "$RC" -ne 0 ]; then
    echo
    echo "[FAIL] FR3 restore interrupted/failed"
    echo "[SAFE] stop at current pose"
    echo "[SAFE] no retry"
    echo "[SAFE] no ErrorRecovery"
    echo "[SAFE] no additional return command"
    exit "$RC"
fi

echo
echo "========================================================================"
echo "[PASS] FR3 RESTORED TO P1"
echo "========================================================================"

exit 0
