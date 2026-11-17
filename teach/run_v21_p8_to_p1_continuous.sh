#!/usr/bin/env bash
set +e

source "$HOME/robot_env/rehab.sh"

VALIDATOR="$HOME/rehab_robot_ws/teach/fr3_v21_segment_validator.py"

if [ ! -f "$VALIDATOR" ]; then
    echo "[FAIL] validator missing: $VALIDATOR"
    exit 1
fi

echo "========================================================================"
echo "FR3 V2.1 CONTINUOUS TASK EXECUTION"
echo "P8 -> P7 -> P6 -> P5 -> P4 -> P3 -> P2 -> P1"
echo "========================================================================"
echo
echo "velocity scaling     = 0.03"
echo "acceleration scaling = 0.03"
echo
echo "ONE operator confirmation only."
echo "After confirmation all validated segments run automatically."
echo
echo "NO retry"
echo "NO auto-return"
echo "NO ErrorRecovery"
echo "Any failure stops the remaining route."
echo "========================================================================"
echo

read -r -p "Type exactly EXECUTE-P8-P1-CONTINUOUS : " ANSWER

if [ "$ANSWER" != "EXECUTE-P8-P1-CONTINUOUS" ]; then
    echo "[ABORT] confirmation mismatch"
    exit 1
fi

SEGMENTS=(
    "P8 P7"
    "P7 P6"
    "P6 P5"
    "P5 P4"
    "P4 P3"
    "P3 P2"
    "P2 P1"
)

INDEX=0
TOTAL="${#SEGMENTS[@]}"

for SEG in "${SEGMENTS[@]}"; do
    INDEX=$((INDEX + 1))
    read -r FROM TO <<< "$SEG"

    echo
    echo "########################################################################"
    echo "AUTO SEGMENT $INDEX / $TOTAL"
    echo "$FROM -> $TO"
    echo "########################################################################"

    TOKEN="EXECUTE-${FROM}-${TO}"

    # Feed the already-approved per-segment confirmation
    # automatically after the validator finishes its own
    # live-state, planning and trajectory audits.
    printf '%s\n' "$TOKEN" | \
    python3 -u "$VALIDATOR" \
        --from-point "$FROM" \
        --to-point "$TO" \
        --execute

    RC=$?

    if [ "$RC" -ne 0 ]; then
        echo
        echo "========================================================================"
        echo "[STOP] $FROM -> $TO FAILED"
        echo "[SAFE] remaining route NOT executed"
        echo "[SAFE] no retry"
        echo "[SAFE] no auto-return"
        echo "[SAFE] no ErrorRecovery"
        echo "========================================================================"
        exit "$RC"
    fi

    echo "[PASS] $FROM -> $TO"
done

echo
echo "========================================================================"
echo "[PASS] FR3 V2.1 CONTINUOUS TASK COMPLETE"
echo "P8 -> P7 -> P6 -> P5 -> P4 -> P3 -> P2 -> P1"
echo
echo "[PASS] all 7 validated segments executed automatically"
echo "[SAFE] no retry"
echo "[SAFE] no auto-return"
echo "========================================================================"
exit 0
