#!/usr/bin/env bash

WS="$HOME/rehab_robot_ws"

source "$WS/env.sh"
source "$WS/config/hardware_profile.env"

PY="$WS/tools/phase1_xhand.py"
LOGDIR="$WS/validation_logs/runtime"

mkdir -p "$LOGDIR"

case "${1:-}" in

    --sequential)
        echo "============================================================"
        echo "XHand 逐关节闭合方向验证"
        echo "每次只激活一个关节"
        echo "kp=225 / kd=0 / 150mA / |delta|=0.005"
        echo "方向由设备 angle_stretching/angle_closing 自动确定"
        echo "FR3 不参与"
        echo "============================================================"

        CONFLICT="$(
            ps -eo pid,args               | grep -E                 'xhand_control_ros2|test_ethercat|xhand_mode_probe|phase1_xhand_whole.py|phase1_xhand_sequential.py'               | grep -v grep               || true
        )"

        if [ -n "$CONFLICT" ]; then
            echo "$CONFLICT"
            echo "[FAIL] 已有 XHand 进程"
            exit 2
        fi

        ip -br link show "$XHAND_IF" || exit 2

        CARRIER="$(
            cat "/sys/class/net/$XHAND_IF/carrier"               2>/dev/null
        )"

        echo "carrier=$CARRIER"

        if [ "$CARRIER" != "1" ]; then
            echo "[FAIL] XHand EtherCAT carrier != 1"
            exit 2
        fi

        sudo -v || exit 2

        LOG="$LOGDIR/xhand_phase1_sequential_$(date +%Y%m%d_%H%M%S).log"

        echo "log=$LOG"
        echo
        echo "要求：XHand 完全空载，手指周围无人和物体。"
        echo

        sudo -n env           HOME="$HOME"           XHAND_IF="$XHAND_IF"           XHAND_SERIAL="$XHAND_SERIAL"           PYTHONUNBUFFERED=1           /bin/bash --noprofile --norc -c '
              source "$HOME/rehab_robot_ws/env.sh"
              cd "$HOME/rehab_robot_ws/tools"
              exec python3 phase1_xhand_sequential.py
          ' 2>&1 | tee "$LOG"

        RC="${PIPESTATUS[0]}"

        echo
        echo "============================================================"
        echo "result=$RC"
        echo "log=$LOG"
        echo "============================================================"

        exit "$RC"
        ;;

    --whole)
        echo "============================================================"
        echo "XHand Phase-1 整手空载微动验证"
        echo "9 active joints"
        echo "delta = +0.005 rad"
        echo "candidate kp = 225"
        echo "validation limit = 150 mA/joint"
        echo "profile maximum = 220 mA/joint"
        echo "FR3 = 不参与"
        echo "============================================================"

        CONFLICT="$(
            pgrep -af               'xhand_control_ros2|test_ethercat|xhand_mode_probe|phase1_xhand_whole'               2>/dev/null               || true
        )"

        if [ -n "$CONFLICT" ]; then
            echo "$CONFLICT"
            echo
            echo "[FAIL] 已有 XHand 进程"
            exit 2
        fi

        ip -br link show "$XHAND_IF" || exit 2

        CARRIER="$(
            cat "/sys/class/net/$XHAND_IF/carrier"               2>/dev/null
        )"

        echo "carrier=$CARRIER"

        if [ "$CARRIER" != "1" ]; then
            echo "[FAIL] XHand EtherCAT carrier != 1"
            exit 2
        fi

        sudo -v || exit 2

        LOG="$LOGDIR/xhand_phase1_whole_$(date +%Y%m%d_%H%M%S).log"

        echo "log=$LOG"
        echo
        echo "空载要求："
        echo "  - 手指周围没有人体或物体"
        echo "  - FR3 不参与"
        echo "  - 异常时立即 Ctrl+C / 急停"
        echo

        sudo -n env           HOME="$HOME"           XHAND_IF="$XHAND_IF"           XHAND_SERIAL="$XHAND_SERIAL"           PYTHONUNBUFFERED=1           /bin/bash --noprofile --norc -c '
              source "$HOME/rehab_robot_ws/env.sh"
              exec python3                 "$HOME/rehab_robot_ws/tools/phase1_xhand_whole.py"
          ' 2>&1 | tee "$LOG"

        RC="${PIPESTATUS[0]}"

        echo
        echo "============================================================"
        echo "result=$RC"
        echo "log=$LOG"
        echo "============================================================"

        exit "$RC"
        ;;

    --single225)
        echo "============================================================"
        echo "XHand Phase-1：kp=225 单关节重新验证"
        echo "delta=+0.005 rad"
        echo "tor_max=150 mA"
        echo "kd=0"
        echo "============================================================"

        export XHAND_TEST_KP=225
        exec "$0" --single
        ;;

    --check)
        echo "============================================================"
        echo "XHand Phase-1 静态检查"
        echo "本模式不打开 EtherCAT"
        echo "============================================================"

        python3 "$PY" --check
        exit $?
        ;;

    --single)
        echo "============================================================"
        echo "XHand Phase-1 单关节空载微动"
        echo "joint  : index_joint1"
        echo "delta  : +0.005 rad"
        echo "limit  : 150 mA"
        echo "FR3    : 不参与"
        echo "============================================================"

        echo
        echo "========== 冲突进程 =========="

        CONFLICT="$(
            pgrep -af \
              'xhand_control_ros2|test_ethercat' \
              2>/dev/null \
              || true
        )"

        if [ -n "$CONFLICT" ]; then
            echo "$CONFLICT"
            echo
            echo "[FAIL] 已有 XHand 进程，未打开硬件"
            exit 2
        fi

        echo "[PASS] 无冲突 XHand 进程"

        echo
        echo "========== EtherCAT 网卡 =========="

        ip -br link show "$XHAND_IF" || exit 2

        CARRIER="$(
            cat \
              "/sys/class/net/$XHAND_IF/carrier" \
              2>/dev/null
        )"

        echo "carrier=$CARRIER"

        if [ "$CARRIER" != "1" ]; then
            echo "[FAIL] XHand 物理链路未连接"
            exit 2
        fi

        echo
        echo "========== sudo =========="

        sudo -v || exit 2

        LOG="$LOGDIR/xhand_phase1_single_kp${XHAND_TEST_KP:-150}_$(date +%Y%m%d_%H%M%S).log"

        echo "log=$LOG"

        echo
        echo "空载要求："
        echo "  - XHand 手指周围无人体、无假脚、无夹持物"
        echo "  - 只允许 index_joint1 微小动作"
        echo "  - 出现异常可立即 Ctrl+C"
        echo

        sudo -n env \
          HOME="$HOME" \
          XHAND_IF="$XHAND_IF" \
          XHAND_SERIAL="$XHAND_SERIAL" \
          XHAND_TEST_KP="${XHAND_TEST_KP:-150}" \
          PYTHONUNBUFFERED=1 \
          /bin/bash --noprofile --norc -c '
              source "$HOME/rehab_robot_ws/env.sh"
              python3 \
                "$HOME/rehab_robot_ws/tools/phase1_xhand.py" \
                --single
          ' 2>&1 | tee "$LOG"

        RC="${PIPESTATUS[0]}"

        echo
        echo "============================================================"
        echo "result=$RC"
        echo "log=$LOG"
        echo "============================================================"

        exit "$RC"
        ;;

    *)
        echo "用法："
        echo "  ./tools/phase1_xhand.sh --check"
        echo "  ./tools/phase1_xhand.sh --single"
        echo "  ./tools/phase1_xhand.sh --single225"
        exit 2
        ;;
esac
