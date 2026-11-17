#!/usr/bin/env bash

set -o pipefail

WS="$HOME/rehab_robot_ws"
OUTDIR="$WS/validation_logs/runtime"

SERVER="$WS/tools/rehab_move_group_plan_only_server.py"

PLANNER="$WS/src/rehab_robot_control/rehab_robot_control/rehab_continuous_short_plan_v2_1.py"

EXECUTOR="$WS/src/rehab_robot_control/rehab_robot_control/rehab_continuous_short_execute_v2_1.py"

PARAMS="$HOME/franka_ros2_ws/install/franka_fr3_moveit_config/share/franka_fr3_moveit_config/config/fr3_ros_controllers.yaml"

TYPE="joint_trajectory_controller/JointTrajectoryController"

STAMP="$(date +%Y%m%d_%H%M%S)"

PLAN="$OUTDIR/continuous_short_v2_2_${STAMP}.json"

PRESTATE="$OUTDIR/continuous_short_v2_2_prestate_${STAMP}.json"

MOVEIT_LOG="$OUTDIR/continuous_short_v2_2_moveit_${STAMP}.log"

SERVER_PID=""

CONTROLLER_LOADED=0
CONTROLLER_ACTIVE=0


cleanup_moveit() {

    if [ -n "$SERVER_PID" ]; then
        kill -INT "$SERVER_PID" \
            2>/dev/null || true
    fi

    sleep 1

    PIDS="$(
        pgrep -f \
        '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
        2>/dev/null \
        || true
    )"

    for PID in $PIDS; do
        kill -TERM "$PID" \
            2>/dev/null || true
    done

    if [ -n "$SERVER_PID" ]; then
        kill -TERM "$SERVER_PID" \
            2>/dev/null || true
    fi

    SERVER_PID=""

    sleep 1
}


cleanup_controller() {

    if [ "$CONTROLLER_ACTIVE" -eq 1 ]; then

        echo
        echo "[SAFE] deactivating fr3_arm_controller"

        ros2 control switch_controllers \
          --deactivate fr3_arm_controller \
          --strict \
          -c /controller_manager \
          >/dev/null 2>&1 || true

        CONTROLLER_ACTIVE=0
    fi

    if [ "$CONTROLLER_LOADED" -eq 1 ]; then

        echo "[SAFE] unloading fr3_arm_controller"

        ros2 control unload_controller \
          -c /controller_manager \
          fr3_arm_controller \
          >/dev/null 2>&1 || true

        CONTROLLER_LOADED=0
    fi
}


final_cleanup() {

    cleanup_moveit
    cleanup_controller
}


trap final_cleanup EXIT INT TERM


cd "$WS" || exit 2
source "$WS/env.sh"

mkdir -p "$OUTDIR"


echo "============================================================"
echo "FR3 CONTINUOUS SHORT REAL VALIDATION V2.2"
echo "ACTIVE FIRST -> SETTLE -> PLAN -> 5 mm TRAJECTORY"
echo "============================================================"


echo
echo "========== 1. Initial safety state =========="

MODE="$(
    timeout 3 \
    ros2 topic echo \
      /franka_robot_state_broadcaster/robot_state \
      --once \
      2>/dev/null \
    | awk '/^robot_mode:/ {print $2; exit}'
)"

echo "robot_mode=$MODE"

if [ "$MODE" != "1" ]; then
    echo "[BLOCK] FR3 is not initially IDLE"
    exit 2
fi

CTRL="$(
    ros2 control list_controllers \
      2>/dev/null || true
)"

printf '%s\n' "$CTRL"

BAD="$(
    printf '%s\n' "$CTRL" \
    | awk '$NF=="active" {print $1}' \
    | grep -vxE \
      'joint_state_broadcaster|franka_robot_state_broadcaster' \
    || true
)"

if [ -n "$BAD" ]; then
    echo "[BLOCK] unexpected motion controller active"
    exit 2
fi

if printf '%s\n' "$CTRL" \
   | grep -q '^fr3_arm_controller '
then
    echo "[BLOCK] fr3_arm_controller already loaded"
    exit 2
fi


echo
echo "========== 2. Capture PRE-ACTIVATION pose =========="

python3 - "$PRESTATE" <<'PY'
import json
import sys

import rclpy

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    PlannerNode,
)

out = sys.argv[1]

rclpy.init()

node = PlannerNode()

try:

    q = node.read_joint_state()

    xyz, quat = node.read_flange_pose()

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

data = {
    "q": q,
    "xyz": list(xyz),
    "quat": list(quat),
}

with open(
    out,
    "w",
    encoding="utf-8",
) as f:
    json.dump(
        data,
        f,
        indent=2,
    )

print(
    "preactivation xyz =",
    [round(v, 6) for v in xyz],
)

print(
    "preactivation q =",
    [round(v, 6) for v in q],
)
PY

if [ "$?" -ne 0 ]; then
    echo "[BLOCK] unable to capture initial state"
    exit 2
fi


echo
echo "============================================================"
echo "CONTROLLER-ACTIVATION SAFETY GATE"
echo "============================================================"
echo
echo "Important:"
echo "The previous test showed that ACTIVATING the effort controller"
echo "can itself shift the arm by several millimetres before any"
echo "trajectory goal is sent."
echo
echo "Before continuing confirm:"
echo "  Desk = EXEC"
echo "  FCI = ON"
echo "  joints unlocked"
echo "  Pilot hand-guiding = OFF"
echo "  workspace completely clear"
echo "  NO person / patient / ankle contact"
echo "  XHand NOT being controlled"
echo
echo "Type exactly:"
echo
echo "EXECUTE-FR3-ACTIVEFIRST-5MM"
echo

read -r CONFIRM

if [ "$CONFIRM" != \
     "EXECUTE-FR3-ACTIVEFIRST-5MM" ]
then
    echo "[BLOCK] activation not confirmed"
    exit 2
fi


echo
echo "========== 3. Register controller =========="

ros2 param set \
  /controller_manager \
  fr3_arm_controller.type \
  "$TYPE"

if [ "$?" -ne 0 ]; then
    echo "[FAIL] controller type registration"
    exit 2
fi

PARAMS_CURRENT="$(
    ros2 param get \
      /controller_manager \
      fr3_arm_controller.params_file \
      2>&1 || true
)"

if ! printf '%s\n' "$PARAMS_CURRENT" \
   | grep -Fq "$PARAMS"
then

    ros2 param set \
      /controller_manager \
      fr3_arm_controller.params_file \
      "['$PARAMS']"

    if [ "$?" -ne 0 ]; then
        echo "[FAIL] params_file registration"
        exit 2
    fi
fi


echo
echo "========== 4. Load controller =========="

LOAD_OUT="$(
    timeout 8 \
    ros2 service call \
      /controller_manager/load_controller \
      controller_manager_msgs/srv/LoadController \
      "{name: 'fr3_arm_controller'}" \
      2>&1
)"

printf '%s\n' "$LOAD_OUT"

if ! printf '%s\n' "$LOAD_OUT" \
   | grep -Eq \
     'ok=True|ok:[[:space:]]*true'
then
    echo "[FAIL] load_controller"
    exit 2
fi

CONTROLLER_LOADED=1


echo
echo "========== 5. Configure controller =========="

CONFIG_OUT="$(
    timeout 8 \
    ros2 service call \
      /controller_manager/configure_controller \
      controller_manager_msgs/srv/ConfigureController \
      "{name: 'fr3_arm_controller'}" \
      2>&1
)"

printf '%s\n' "$CONFIG_OUT"

if ! printf '%s\n' "$CONFIG_OUT" \
   | grep -Eq \
     'ok=True|ok:[[:space:]]*true'
then
    echo "[FAIL] configure_controller"
    exit 2
fi


echo
echo "========== 6. Activate controller =========="

ros2 control switch_controllers \
  --activate fr3_arm_controller \
  --strict \
  -c /controller_manager

if [ "$?" -ne 0 ]; then
    echo "[FAIL] controller activation"
    exit 2
fi

CONTROLLER_ACTIVE=1

STATE="$(
    ros2 control list_controllers \
    2>/dev/null \
    | awk '
      $1=="fr3_arm_controller" {
        print $NF
        exit
      }
    '
)"

echo "fr3_arm_controller_state=$STATE"

if [ "$STATE" != "active" ]; then
    echo "[FAIL] controller not active"
    exit 2
fi

echo "[PASS] controller active"


echo
echo "========== 7. ACTIVE-HOLD settling audit =========="

python3 - "$PRESTATE" <<'PY'
import json
import math
import sys
import time

import rclpy

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    PlannerNode,
    JOINT_NAMES,
)

pre_path = sys.argv[1]

with open(
    pre_path,
    "r",
    encoding="utf-8",
) as f:
    pre = json.load(f)


def dist3(a, b):

    return math.sqrt(
        sum(
            (float(x) - float(y)) ** 2
            for x, y in zip(a, b)
        )
    )


rclpy.init()

node = PlannerNode()

try:

    # Allow the known control-mode transition to settle first.
    print(
        "[WAIT] controller equilibrium settling 2.0 s"
    )

    time.sleep(2.0)

    samples = []

    start = time.monotonic()

    while (
        time.monotonic() - start
        < 1.5
    ):

        q = node.read_joint_state(
            timeout_s=1.0
        )

        samples.append(q)

        time.sleep(0.04)

    settled_q = node.read_joint_state()

    settled_xyz, _ = (
        node.read_flange_pose()
    )

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if len(samples) < 10:
    raise SystemExit(
        "[BLOCK] insufficient settling samples"
    )


ranges = []

for j in range(7):

    values = [
        q[j]
        for q in samples
    ]

    ranges.append(
        max(values) - min(values)
    )


max_hold_range = max(ranges)

max_hold_joint = (
    JOINT_NAMES[
        ranges.index(max_hold_range)
    ]
)


activation_joint_delta = max(
    abs(a - b)
    for a, b in zip(
        settled_q,
        pre["q"],
    )
)

activation_xyz_delta = dist3(
    settled_xyz,
    pre["xyz"],
)


print()
print(
    "settled xyz =",
    [
        round(v, 6)
        for v in settled_xyz
    ],
)

print(
    f"activation Cartesian shift = "
    f"{activation_xyz_delta * 1000:.3f} mm"
)

print(
    f"activation max joint shift = "
    f"{activation_joint_delta:.6f} rad"
)

print(
    f"1.5 s maximum hold range = "
    f"{max_hold_range:.6f} rad "
    f"({max_hold_joint})"
)


# These are safety gates, not production accuracy limits.
if activation_xyz_delta > 0.020:
    print(
        "[BLOCK] activation displacement > 20 mm"
    )
    raise SystemExit(2)

if activation_joint_delta > 0.030:
    print(
        "[BLOCK] activation joint shift > 0.030 rad"
    )
    raise SystemExit(2)

if max_hold_range > 0.0015:
    print(
        "[BLOCK] active controller has not stabilized"
    )
    raise SystemExit(2)


print(
    "[PASS] ACTIVE-HOLD equilibrium stable"
)
PY

SETTLE_RC=$?

if [ "$SETTLE_RC" -ne 0 ]; then
    echo "[FAULT] active-hold stability gate failed"
    echo "[SAFE] no trajectory goal sent"
    exit 2
fi


echo
echo "========== 8. Start MoveIt WHILE controller remains ACTIVE =========="

if pgrep -f \
  '^/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' \
  >/dev/null
then
    echo "[BLOCK] existing move_group"
    exit 2
fi

python3 "$SERVER" \
  --robot-ip 192.168.1.25 \
  >"$MOVEIT_LOG" 2>&1 &

SERVER_PID=$!

READY=0

for I in $(seq 1 40); do

    SERVICES="$(
        timeout 2 \
        ros2 service list -t \
        2>/dev/null \
        || true
    )"

    if printf '%s\n' "$SERVICES" \
       | grep -q \
       '^/compute_cartesian_path \[moveit_msgs/srv/GetCartesianPath\]' \
       && \
       printf '%s\n' "$SERVICES" \
       | grep -q \
       '^/compute_fk \[moveit_msgs/srv/GetPositionFK\]'
    then
        READY=1
        break
    fi

    echo "[WAIT] MoveIt $I/40"
    sleep 0.5
done

if [ "$READY" -ne 1 ]; then

    echo "[FAIL] MoveIt services unavailable"

    tail -n 100 "$MOVEIT_LOG" \
      2>/dev/null || true

    exit 2
fi


echo
echo "========== 9. PLAN from ACTIVE-HOLD state =========="

python3 "$PLANNER" \
  --output "$PLAN"

PLAN_RC=$?

cleanup_moveit

if [ "$PLAN_RC" -ne 0 ]; then
    echo "[BLOCK] active-state trajectory planning failed"
    exit 2
fi

echo
echo "[PASS] active-state 5 mm candidate audited"


echo
echo "========== 10. Revalidate ACTIVE start immediately before goal =========="

python3 - "$PLAN" <<'PY'
import json
import sys

import rclpy

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    PlannerNode,
    max_abs_delta,
)

with open(
    sys.argv[1],
    "r",
    encoding="utf-8",
) as f:
    plan = json.load(f)

rclpy.init()

node = PlannerNode()

try:

    live = node.read_joint_state()

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


error = max_abs_delta(
    live,
    plan["start_q"],
)

print(
    f"active-plan start q error = "
    f"{error:.6f} rad"
)

if error > 0.003:

    print(
        "[BLOCK] active equilibrium moved "
        "after planning"
    )

    raise SystemExit(2)


print(
    "[PASS] execution start still valid"
)
PY

if [ "$?" -ne 0 ]; then
    exit 2
fi


echo
echo "========== 11. Execute ONE 5 mm trajectory =========="

python3 "$EXECUTOR" \
  --plan "$PLAN"

EXEC_RC=$?

echo "trajectory_execution_result=$EXEC_RC"

if [ "$EXEC_RC" -ne 0 ]; then

    echo "[FAULT] FollowJointTrajectory validation failed"
    echo "[SAFE] no retry"
    echo "[SAFE] no automatic return"

    exit 2
fi


echo
echo "========== 12. Final pose WHILE controller is ACTIVE =========="

python3 - "$PLAN" <<'PY'
import json
import math
import sys

import rclpy

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    PlannerNode,
)

with open(
    sys.argv[1],
    "r",
    encoding="utf-8",
) as f:
    plan = json.load(f)

target = plan["target_xyz"]

rclpy.init()

node = PlannerNode()

try:

    xyz, _ = node.read_flange_pose()

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


error = math.sqrt(
    sum(
        (float(a) - float(b)) ** 2
        for a, b in zip(
            xyz,
            target,
        )
    )
)

print(
    "active final xyz =",
    [
        round(v, 6)
        for v in xyz
    ],
)

print(
    f"ACTIVE final target error = "
    f"{error * 1000:.3f} mm"
)

if error > 0.003:
    print(
        "[WARN] active final Cartesian "
        "error > 3 mm"
    )
PY


echo
echo "========== 13. Deactivate controller =========="

ros2 control switch_controllers \
  --deactivate fr3_arm_controller \
  --strict \
  -c /controller_manager

if [ "$?" -ne 0 ]; then
    echo "[FAULT] controller deactivation failed"
    exit 2
fi

CONTROLLER_ACTIVE=0


echo
echo "========== 14. Unload controller =========="

ros2 control unload_controller \
  -c /controller_manager \
  fr3_arm_controller

if [ "$?" -ne 0 ]; then
    echo "[FAULT] controller unload failed"
    exit 2
fi

CONTROLLER_LOADED=0


echo
echo "========== 15. Post-controller pose =========="

sleep 1

python3 - "$PLAN" <<'PY'
import json
import math
import sys

import rclpy

from rehab_robot_control.rehab_continuous_trajectory_plan_v2 import (
    PlannerNode,
)

with open(
    sys.argv[1],
    "r",
    encoding="utf-8",
) as f:
    plan = json.load(f)

target = plan["target_xyz"]

rclpy.init()

node = PlannerNode()

try:

    xyz, _ = node.read_flange_pose()

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


error = math.sqrt(
    sum(
        (float(a) - float(b)) ** 2
        for a, b in zip(
            xyz,
            target,
        )
    )
)

print(
    "post-controller xyz =",
    [
        round(v, 6)
        for v in xyz
    ],
)

print(
    f"post-controller target error = "
    f"{error * 1000:.3f} mm"
)
PY


echo
echo "========== 16. Final system state =========="

ros2 control list_controllers \
  2>/dev/null || true

echo

timeout 3 \
ros2 topic echo \
  /franka_robot_state_broadcaster/robot_state \
  --once \
  2>/dev/null \
| grep -E \
  '^robot_mode:|joint_reflex:|cartesian_reflex:|communication_constraints_violation:|instability_detected:' \
|| true


echo
echo "============================================================"
echo "[PASS] FR3 ACTIVE-FIRST 5 mm VALIDATION COMPLETE"
echo "============================================================"
echo "architecture:"
echo "ACTIVATE -> SETTLE -> PLAN -> EXECUTE"
echo
echo "ONE FollowJointTrajectory goal"
echo "NO PTP"
echo "NO RETRY"
echo "NO AUTO RETURN"
echo "NO XHAND"
echo "============================================================"

trap - EXIT INT TERM

exit 0
