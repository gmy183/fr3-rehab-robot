#!/usr/bin/env bash
set -eo pipefail

source "$HOME/robot_env/rehab.sh"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WS="$HOME/rehab_robot_ws"
LOGDIR="$WS/validation_logs/runtime"
mkdir -p "$LOGDIR"

LOG="$LOGDIR/fr3_handeye_stability_$(date +%Y%m%d_%H%M%S).log"

echo "=================================================="
echo "FR3 + D435i HAND-EYE STABILITY TEST"
echo "Duration: 180 seconds"
echo "NO TRAJECTORY / NO AUTO RECOVERY"
echo "=================================================="

echo
echo "请确认机器人工作空间安全、现场有人监护、急停可用。"
read -r -p "确认后输入 YES 开始： " CONFIRM

if [ "$CONFIRM" != "YES" ]; then
    echo "[STOP] Cancelled"
    exit 1
fi

check_controllers()
{
    local STATES NAME

    STATES=$(timeout 8 ros2 control list_controllers) || return 1

    echo "$STATES"

    for NAME in \
        fr3_arm_controller \
        joint_state_broadcaster \
        franka_robot_state_broadcaster
    do
        echo "$STATES" | awk -v n="$NAME" \
            '$1==n && $NF=="active" {ok=1}
             END {exit !ok}' || return 1
    done
}

echo
echo "========== 1. OFFICIAL SESSION =========="

CM_PID=$(pgrep -f \
'^/opt/ros/jazzy/lib/controller_manager/ros2_control_node' \
|| true)

if [ -n "$CM_PID" ]; then

    echo "[INFO] Existing controller_manager: $CM_PID"

    if ! check_controllers; then
        echo "[STOP] Existing FR3 session is unhealthy."
        echo "[SAFE] No second MoveIt session started."
        exit 2
    fi

    echo "[PASS] Existing official session healthy"

else

    echo "[INFO] No controller_manager"

    if pgrep -af \
      'ros2 launch franka_fr3_moveit_config moveit.launch.py' \
      >/dev/null; then
        echo "[STOP] An existing MoveIt launch process was detected"
        exit 2
    fi

    "$WS/tools/ensure_fr3_moveit_session.sh" || exit 2

    if ! check_controllers; then
        echo "[FAIL] Controllers not active"
        exit 2
    fi

fi

echo
echo "========== 2. GRIPPER CHECK =========="

if pgrep -af \
'^.*/franka_gripper_node([[:space:]]|$)'; then
    echo "[FAIL] Unexpected Franka gripper node"
    exit 3
fi

echo "[PASS] No Franka gripper node"

echo
echo "========== 3. VISION MODE =========="

if pgrep -af \
'^python3 .*/handeye_v2/handeye_capture_v2.py$'; then
    echo "[MODE] FR3 + V2 vision"
else
    echo "[MODE] FR3 only / baseline"
fi

echo
echo "========== 4. 180 SECOND MONITOR =========="

set +e

python3 -u - <<'PY' 2>&1 | tee "$LOG"
import time
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import JointState

DURATION = 180.0
NAMES = [f"fr3_joint{i}" for i in range(1, 8)]

rclpy.init()
node = Node("fr3_handeye_stability_monitor")

state = {
    "count": 0,
    "last": None,
    "first_q": None,
    "max_age": 0.0,
    "max_dq": 0.0,
    "max_drift": 0.0,
    "failure": None,
}

def callback(msg):
    now_wall = time.time()
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    age = abs(now_wall - stamp)

    table = dict(zip(msg.name, msg.position))

    if not all(n in table for n in NAMES):
        state["failure"] = "Missing FR3 joints"
        return

    q = [float(table[n]) for n in NAMES]

    if not all(math.isfinite(x) for x in q):
        state["failure"] = "Non-finite joint position"
        return

    if state["first_q"] is None:
        state["first_q"] = q

    drift = max(abs(a-b) for a,b in zip(q, state["first_q"]))

    state["max_drift"] = max(state["max_drift"], drift)
    state["max_age"] = max(state["max_age"], age)

    if len(msg.velocity) == len(msg.name):
        velocities = dict(zip(msg.name, msg.velocity))
        dq = max(abs(float(velocities[n])) for n in NAMES)
        state["max_dq"] = max(state["max_dq"], dq)

        if not math.isfinite(dq) or dq > 0.08:
            state["failure"] = "Unexpected joint velocity"

    if age > 1.0:
        state["failure"] = "Stale joint timestamp"

    if drift > 0.02:
        state["failure"] = "Unexpected joint position change"

    state["last"] = time.monotonic()
    state["count"] += 1

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

node.create_subscription(
    JointState,
    "/franka/joint_states",
    callback,
    qos,
)

start = time.monotonic()
next_report = start + 10.0
result = 0

print("[START] 180-second static monitoring", flush=True)

try:
    while time.monotonic() - start < DURATION:

        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.015)

        elapsed = time.monotonic() - start

        if state["failure"]:
            print("[FAIL]", state["failure"])
            result = 2
            break

        if elapsed > 5.0 and state["count"] == 0:
            print("[FAIL] No FR3 joint state received")
            result = 2
            break

        if state["last"] is not None:
            if time.monotonic() - state["last"] > 1.0:
                print("[FAIL] Joint state stream stopped")
                result = 2
                break

        if time.monotonic() >= next_report:
            print(
                f"[RUNNING] {elapsed:.0f}/180 s | "
                f"samples={state['count']} | "
                f"max_age={state['max_age']:.3f}s | "
                f"max_drift={state['max_drift']:.4f}rad",
                flush=True,
            )
            next_report += 10.0

except KeyboardInterrupt:
    print("[STOP] Operator interrupted monitoring")
    result = 130

finally:
    node.destroy_node()
    rclpy.shutdown()

if result == 0:
    if state["count"] < 1800:
        print("[FAIL] Insufficient joint-state samples")
        result = 2
    else:
        print("[PASS] 180-second joint-state monitoring completed")

print("samples =", state["count"])
print("max_age_s =", round(state["max_age"], 4))
print("max_velocity_rad_s =", round(state["max_dq"], 5))
print("max_position_drift_rad =", round(state["max_drift"], 5))

raise SystemExit(result)
PY

RESULT=${PIPESTATUS[0]}
set -e

echo
echo "========== 5. FINAL CONTROLLER CHECK =========="

if ! check_controllers; then
    echo "[FAIL] Controller state"
    RESULT=2
fi

echo
echo "========== 6. RESULT =========="

if [ "$RESULT" -eq 0 ]; then
    echo "[PASS] FR3 STABILITY 180s"
else
    echo "[FAIL] FR3 STABILITY"
    echo "[SAFE] No automatic recovery attempted"
fi

echo "LOG=$LOG"

exit "$RESULT"
