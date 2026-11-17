#!/usr/bin/env bash

set -o pipefail

WS="$HOME/rehab_robot_ws"
OUTDIR="$WS/validation_logs/runtime"

mkdir -p "$OUTDIR"

cd "$WS" || exit 2
source "$WS/env.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"

JSON="$OUTDIR/fr3_xhand_payload_audit_${STAMP}.json"
LOG="$OUTDIR/fr3_xhand_payload_audit_${STAMP}.log"

echo "============================================================"
echo "ONE-CLICK FR3 + XHAND PAYLOAD AUDIT"
echo "READ ONLY"
echo "============================================================"

python3 \
  "$WS/tools/audit_fr3_xhand_payload.py" \
  "$JSON" \
  2>&1 \
| tee "$LOG"

RC=${PIPESTATUS[0]}

echo
echo "============================================================"
echo "AUDIT FILES"
echo "============================================================"
echo "result=$RC"
echo "json=$JSON"
echo "log=$LOG"
echo
echo "NO MOTION"
echo "NO SETLOAD"
echo "============================================================"

exit "$RC"
