#!/usr/bin/env bash

RUNTIME="$HOME/rehab_robot_ws/.runtime"

echo "============================================================"
echo "停止康复机械臂三设备 ROS 进程"
echo "============================================================"

for name in xhand d435i fr3
do
    PIDFILE="$RUNTIME/$name.pid"

    if [ ! -f "$PIDFILE" ]; then
        echo "[INFO] $name：没有 pid 文件"
        continue
    fi

    PID="$(cat "$PIDFILE" 2>/dev/null)"

    if [ -z "$PID" ]; then
        rm -f "$PIDFILE"
        continue
    fi

    if kill -0 "$PID" 2>/dev/null; then
        echo "[STOP] $name pid=$PID"

        kill -TERM -- "-$PID" 2>/dev/null || \
        kill -TERM "$PID" 2>/dev/null || true

        for _ in $(seq 1 30)
        do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$PID" 2>/dev/null; then
            echo "[WARN] $name 强制结束"
            kill -KILL -- "-$PID" 2>/dev/null || \
            kill -KILL "$PID" 2>/dev/null || true
        fi
    else
        echo "[INFO] $name 已停止"
    fi

    rm -f "$PIDFILE"
done

echo
echo "[PASS] stop_all_devices 完成"
