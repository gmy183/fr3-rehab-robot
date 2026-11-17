#!/usr/bin/env bash

set +e

WS="$HOME/rehab_robot_ws"

source "$HOME/robot_env/rehab.sh"

FR3_RUNNER="$WS/teach/run_v21_bidirectional.sh"

XHAND_FROZEN="$WS/validated/xhand_20261005/xhand_open_grasp_final.py"
XHAND_RUNTIME="$WS/tools/xhand_grasp_hold_at_p8.py"

EXPECTED_SHA="f5d43e4bc34f1afc1f1c3067ec0d44ef90941373eeb384668126dd0a911c0e06"

echo "========================================================================"
echo "FR3 + XHAND FINAL COMBINED EXECUTOR"
echo
echo "P1 -> P8 -> XHand GRASP -> ACTIVE HOLD"
echo "========================================================================"
echo
echo "FR3:"
echo "  official MoveIt V2.1"
echo "  P1 -> P2 -> P3 -> P4 -> P5 -> P6 -> P7 -> P8"
echo
echo "XHand:"
echo "  trajectory = 9 joints"
echo "  thumb_rota_joint2 = ACTIVE HOLD ONLY"
echo "  thumb_rota_joint1 = POWERLESS"
echo "  index_bend_joint   = POWERLESS"
echo
echo "NO automatic retry"
echo "NO automatic FR3 return"
echo "NO ErrorRecovery"
echo "========================================================================"

echo
echo "========== 1. FILE GATE =========="

for f in \
    "$FR3_RUNNER" \
    "$XHAND_FROZEN" \
    "$XHAND_RUNTIME"
do
    if [ ! -f "$f" ]; then
        echo "[FAIL] missing: $f"
        exit 2
    fi
done

echo "[PASS] files present"

echo
echo "========== 2. XHAND FROZEN SHA =========="

ACTUAL_SHA="$(
    sha256sum "$XHAND_FROZEN" \
    | awk '{print $1}'
)"

if [ "$ACTUAL_SHA" != "$EXPECTED_SHA" ]; then
    echo "[FAIL] frozen XHand SHA mismatch"
    echo "expected=$EXPECTED_SHA"
    echo "actual  =$ACTUAL_SHA"
    exit 2
fi

echo "[PASS] frozen XHand SHA"

echo
echo "========== 3. PYTHON GATE =========="

python3 -m py_compile \
    "$XHAND_RUNTIME"

if [ $? -ne 0 ]; then
    echo "[FAIL] XHand runtime syntax"
    exit 2
fi

echo "[PASS] XHand runtime syntax"

echo
echo "========== 4. SUDO =========="

sudo -v

if [ $? -ne 0 ]; then
    echo "[FAIL] sudo unavailable"
    exit 2
fi

echo "[PASS] sudo"

echo
echo "========== FR3 OFFICIAL SESSION =========="

"$WS/tools/ensure_fr3_moveit_session.sh"
SESSION_RC=$?

if [ "$SESSION_RC" -ne 0 ]; then
    echo "[FAIL] FR3 official session unavailable"
    exit "$SESSION_RC"
fi

echo
echo "========================================================================"
echo "REAL COMBINED MOTION"
echo
echo "Required:"
echo "  FR3 currently at P1"
echo "  Desk = EXEC"
echo "  FCI = ON"
echo "  Joints = UNLOCKED"
echo "  workspace clear"
echo "  E-stop immediately reachable"
echo
echo "Sequence:"
echo "  FR3 P1 -> P8"
echo "  then XHand GRASP"
echo "  then XHand ACTIVE HOLD at P8"
echo
echo "On any failure:"
echo "  STOP"
echo "  NO retry"
echo "  NO automatic return"
echo "  NO ErrorRecovery"
echo "========================================================================"
echo

read -r -p \
"Type exactly EXECUTE-P1-P8-GRASP : " \
CONFIRM

if [ "$CONFIRM" != "EXECUTE-P1-P8-GRASP" ]; then
    echo "[ABORT]"
    exit 1
fi

echo
echo "========================================================================"
echo "STAGE A: FR3 P1 -> P8"
echo "========================================================================"

# 只在这里自动喂给已验证的逐段 validator。
# 操作者只需要上面的一个总确认。
printf '%s\n' "EXECUTE-P1-P8" \
| bash "$FR3_RUNNER" 1to8

FR3_RC="${PIPESTATUS[1]}"

echo
echo "FR3_RESULT=$FR3_RC"

if [ "$FR3_RC" -ne 0 ]; then
    echo
    echo "[FAIL] FR3 P1 -> P8 failed"
    echo "[SAFE] XHand grasp WILL NOT start"
    echo "[SAFE] no automatic FR3 return"
    echo "[SAFE] no ErrorRecovery"
    exit "$FR3_RC"
fi

echo
echo "========================================================================"
echo "[PASS] FR3 reached validated P8"
echo "STAGE B: XHand GRASP"
echo "========================================================================"

sudo -E env \
    XHAND_CONFIRM="EXECUTE-XHAND-OPEN-GRASP" \
    bash -c '
        source /opt/ros/jazzy/setup.bash
        source /home/gu/franka_ros2_ws/install/setup.bash
        source /home/gu/rehab_robot_ws/install/setup.bash

        exec python3 -u \
          /home/gu/rehab_robot_ws/tools/xhand_grasp_hold_at_p8.py
    '

XHAND_RC=$?

echo
echo "XHAND_RESULT=$XHAND_RC"

if [ "$XHAND_RC" -ne 0 ]; then
    echo
    echo "[FAIL] XHand grasp/hold failed"
    echo "[SAFE] XHand driver performs powerless shutdown"
    echo "[SAFE] FR3 remains at P8/current stopped state"
    echo "[SAFE] no automatic return"
    exit "$XHAND_RC"
fi

echo
echo "========================================================================"
echo "[PASS] FINAL COMBINED ACTION COMPLETE"
echo
echo "FR3   : P8"
echo "XHand : POWERLESS after operator released hold"
echo
echo "To restore FR3 to P1:"
echo "  ~/rehab_robot_ws/tools/restore_fr3_p8_to_p1.sh"
echo "========================================================================"

exit 0
