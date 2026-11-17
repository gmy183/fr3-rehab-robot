#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"

echo "============================================================"
echo "FR3 + XHand REHAB MULTI-CYCLE V1.2"
echo "SOFTWARE CHECK ONLY"
echo "============================================================"

python3 -m \
  rehab_robot_control.rehab_multicycle_executor \
  "$@"

RC=$?

echo
echo "============================================================"
echo "MULTICYCLE_CHECK_RESULT=$RC"
echo "============================================================"

exit "$RC"
