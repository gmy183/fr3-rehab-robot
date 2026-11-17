#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"

echo "============================================================"
echo "FR3 + XHand REHAB TASK STATE MACHINE V1"
echo "SOFTWARE ONLY"
echo "============================================================"

python3 -m \
  rehab_robot_control.rehab_task_check

RC=$?

echo
echo "============================================================"
echo "REHAB_TASK_CHECK_RESULT=$RC"
echo "============================================================"

exit "$RC"
