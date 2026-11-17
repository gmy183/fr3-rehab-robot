#!/usr/bin/env bash
set +e

source "$HOME/robot_env/rehab.sh"

VALIDATOR="$HOME/rehab_robot_ws/teach/fr3_v21_segment_validator.py"

if [ ! -f "$VALIDATOR" ]; then
    echo "[FAIL] validator missing: $VALIDATOR"
    exit 1
fi

MODE="${1:-}"

case "$MODE" in

    8to1)
        LABEL="P8 -> P1"

        SEGMENTS=(
            "P8 P7"
            "P7 P6"
            "P6 P5"
            "P5 P4"
            "P4 P3"
            "P3 P2"
            "P2 P1"
        )

        CONFIRM="EXECUTE-P8-P1"
        ;;

    1to8)
        LABEL="P1 -> P8"

        SEGMENTS=(
            "P1 P2"
            "P2 P3"
            "P3 P4"
            "P4 P5"
            "P5 P6"
            "P6 P7"
            "P7 P8"
        )

        CONFIRM="EXECUTE-P1-P8"
        ;;

    *)
        echo "Usage:"
        echo "  $0 8to1"
        echo "  $0 1to8"
        exit 1
        ;;
esac


echo "========================================================================"
echo "FR3 V2.1 BIDIRECTIONAL TRAJECTORY"
echo "ROUTE = $LABEL"
echo "========================================================================"
echo
echo "velocity scaling     = 0.03"
echo "acceleration scaling = 0.03"
echo
echo "Each segment:"
echo "  live source-state gate"
echo "  -> official MoveIt plan"
echo "  -> trajectory audit"
echo "  -> execute exact audited trajectory"
echo "  -> final joint-state gate"
echo
echo "ONE confirmation at task start"
echo "NO retry"
echo "NO auto-return"
echo "NO ErrorRecovery"
echo "Any failed segment stops the route immediately."
echo "========================================================================"
echo

read -r -p "Type exactly $CONFIRM : " ANSWER

if [ "$ANSWER" != "$CONFIRM" ]; then
    echo "[ABORT] confirmation mismatch"
    exit 1
fi


TOTAL="${#SEGMENTS[@]}"
INDEX=0

for SEG in "${SEGMENTS[@]}"; do

    INDEX=$((INDEX + 1))

    read -r FROM TO <<< "$SEG"

    TOKEN="EXECUTE-${FROM}-${TO}"

    echo
    echo "########################################################################"
    echo "SEGMENT $INDEX / $TOTAL"
    echo "$FROM -> $TO"
    echo "########################################################################"
    echo

    #
    # Validator still performs:
    # source gate + planning + trajectory audit +
    # controller gate + final error gate.
    #
    # Only its per-segment confirmation is supplied
    # automatically because the operator approved
    # the complete directional task above.
    #

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
        echo "[SAFE] remaining segments NOT executed"
        echo "[SAFE] no retry"
        echo "[SAFE] no auto-return"
        echo "[SAFE] no ErrorRecovery"
        echo "========================================================================"

        exit "$RC"
    fi

    echo
    echo "[PASS] $FROM -> $TO"

done


echo
echo "========================================================================"
echo "[PASS] FR3 V2.1 DIRECTIONAL TASK COMPLETE"
echo "ROUTE = $LABEL"
echo
echo "[PASS] all 7 segments"
echo "[PASS] official MoveIt"
echo "[SAFE] no retry"
echo "[SAFE] no auto-return"
echo "========================================================================"

exit 0
