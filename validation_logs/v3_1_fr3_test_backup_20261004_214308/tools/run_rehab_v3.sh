#!/usr/bin/env bash

set -o pipefail

WS="$HOME/rehab_robot_ws"

if [ ! -f "$WS/env.sh" ]; then
    echo "[BLOCK] missing $WS/env.sh"
    exit 2
fi

source "$WS/env.sh"

export PYTHONPATH="$WS/src/rehab_robot_control:${PYTHONPATH:-}"

MODE=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --mode)
            MODE="${2:-}"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 --mode check|dry-run|real"
            exit 0
            ;;
        *)
            echo "[BLOCK] unknown argument: $1"
            exit 2
            ;;
    esac
done

case "$MODE" in
    check|dry-run|real)
        ;;
    *)
        echo "Usage: $0 --mode check|dry-run|real"
        exit 2
        ;;
esac

exec python3 -m rehab_robot_control.v3.cli \
    --workspace "$WS" \
    --mode "$MODE"
