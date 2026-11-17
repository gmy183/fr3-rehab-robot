#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

case "${1:-}" in

    --check)
        exec \
          "$WS/tools/run_empty_grasp_legacy_entry.sh" \
          --check
        ;;

    --preflight)
        exec \
          "$WS/tools/run_empty_grasp_legacy_entry.sh" \
          --preflight
        ;;

    --execute)
        exec \
          "$WS/tools/empty_grasp_execute.sh"
        ;;

    *)
        echo "Usage:"
        echo "  ./tools/run_empty_grasp.sh --check"
        echo "  ./tools/run_empty_grasp.sh --preflight"
        echo "  ./tools/run_empty_grasp.sh --execute"
        ;;
esac
