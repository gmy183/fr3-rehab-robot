#!/usr/bin/env bash

# IMPORTANT:
# Do NOT use "set -e".
# We need to catch validator failures ourselves
# and keep the terminal open for diagnosis.

set +e

source "$HOME/robot_env/rehab.sh"

VALIDATOR="$HOME/rehab_robot_ws/teach/fr3_v21_segment_validator.py"

echo "========================================================================"
echo "FR3 V2.1 SEQUENTIAL SEGMENT VALIDATION"
echo "START  : P7"
echo "TARGET : P1"
echo "========================================================================"
echo
echo "Route:"
echo "  P7 -> P6"
echo "  P6 -> P5"
echo "  P5 -> P4"
echo "  P4 -> P3"
echo "  P3 -> P2"
echo "  P2 -> P1"
echo
echo "Each segment is independently:"
echo "  live state -> plan -> audit -> confirm -> execute -> verify"
echo
echo "NO retry"
echo "NO auto-return"
echo "NO ErrorRecovery"
echo "Any failure stops the remaining route."
echo "========================================================================"

if [ ! -f "$VALIDATOR" ]; then
    echo
    echo "[FAIL] validator missing:"
    echo "$VALIDATOR"
    read -r -p "Press ENTER to exit..."
    exit 1
fi

SEGMENTS=(
  "P7 P6"
  "P6 P5"
  "P5 P4"
  "P4 P3"
  "P3 P2"
  "P2 P1"
)

TOTAL="${#SEGMENTS[@]}"
INDEX=0

for item in "${SEGMENTS[@]}"; do
    INDEX=$((INDEX + 1))

    read -r FROM TO <<< "$item"

    echo
    echo
    echo "########################################################################"
    echo "SEGMENT $INDEX / $TOTAL"
    echo "$FROM -> $TO"
    echo "########################################################################"
    echo

    python3 -u "$VALIDATOR" \
        --from-point "$FROM" \
        --to-point "$TO" \
        --execute

    RC=$?

    echo
    echo "------------------------------------------------------------------------"
    echo "validator exit code = $RC"
    echo "------------------------------------------------------------------------"

    if [ "$RC" -ne 0 ]; then
        echo
        echo "======================================================================"
        echo "[STOP] $FROM -> $TO DID NOT PASS"
        echo "[SAFE] Remaining segments were NOT executed."
        echo "[SAFE] NO automatic retry."
        echo "[SAFE] NO automatic return."
        echo "[SAFE] NO ErrorRecovery."
        echo "======================================================================"
        echo
        read -r -p "Press ENTER after recording the error output..."
        exit "$RC"
    fi

    echo
    echo "[PASS] $FROM -> $TO"

    if [ "$TO" != "P1" ]; then
        echo
        echo "Robot should now be at $TO."
        echo "Visually check FR3/XHand clearance."
        echo "Keep E-stop ready."
        echo
        echo "Next segment:"

        NEXT_INDEX=$INDEX

        if [ "$NEXT_INDEX" -lt "$TOTAL" ]; then
            read -r NEXT_FROM NEXT_TO <<< "${SEGMENTS[$NEXT_INDEX]}"
            echo "  $NEXT_FROM -> $NEXT_TO"
        fi

        echo
        echo "The next segment will still require its own EXECUTE-... confirmation."
        sleep 1
    fi
done

echo
echo
echo "========================================================================"
echo "[PASS] V2.1 SEQUENTIAL PATH COMPLETE"
echo "P7 -> P6 -> P5 -> P4 -> P3 -> P2 -> P1"
echo
echo "[PASS] All six remaining FR3 segments completed."
echo "[SAFE] P15 full-path lock remains unchanged."
echo "========================================================================"

read -r -p "Press ENTER to finish..."
exit 0
