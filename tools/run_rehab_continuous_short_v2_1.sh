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

PLAN="$OUTDIR/continuous_short_v2_1_${STAMP}.json"

MOVEIT_LOG="$OUTDIR/continuous_short_v2_1_moveit_${STAMP}.log"

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
echo "FR3 CONTINUOUS SHORT REAL VALIDATION V2.1"
echo "5 mm EMPTY-WORKSPACE TEST"
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
    echo "[BLOCK] FR3 not IDLE"
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
    echo "[BLOCK] motion controller already active"
    exit 2
fi


echo
echo "========== 2. Plan 5 mm candidate =========="

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

python3 "$PLANNER" \
  --output "$PLAN"

PLAN_RC=$?

cleanup_moveit

if [ "$PLAN_RC" -ne 0 ]; then
    echo "[BLOCK] short trajectory planning/audit failed"
    exit 2
fi

echo
echo "[PASS] 5 mm candidate passed dense audit"


echo
echo "========== 3. Candidate summary =========="

python3 - "$PLAN" <<'PY'
import json
import sys

with open(
    sys.argv[1],
    "r",
    encoding="utf-8",
) as f:
    p = json.load(f)

print(
    "execution_eligible =",
    p["execution_eligible"],
)

print(
    "distance_mm =",
    round(
        p["test_distance_m"] * 1000,
        3,
    ),
)

print(
    "duration_s =",
    round(
        p["duration_s"],
        3,
    ),
)

print(
    "target_xyz =",
    [
        round(v, 6)
        for v in p["target_xyz"]
    ],
)

print(
    "max_cross_track_mm =",
    round(
        p["audit"][
            "max_cross_track_m"
        ] * 1000,
        3,
    ),
)

print(
    "vmax_rad_s =",
    p["audit"][
        "max_velocity_rad_s"
    ],
)

print(
    "amax_rad_s2 =",
    p["audit"][
        "max_acceleration_rad_s2"
    ],
)
PY


echo
echo "============================================================"
echo "REAL-MOTION SAFETY GATE"
echo "============================================================"
echo
echo "Before continuing confirm physically:"
echo "  Desk = EXEC"
echo "  FCI = ON"
echo "  joints unlocked"
echo "  Pilot hand-guiding = OFF"
echo "  robot workspace completely clear"
echo "  NO person / ankle / patient in contact"
echo "  XHand is NOT being controlled"
echo
echo "Robot will move ONLY ABOUT 5 mm."
echo "There is NO automatic retry."
echo "There is NO automatic return."
echo
echo "Type exactly:"
echo
echo "EXECUTE-FR3-5MM-CONTINUOUS"
echo

read -r CONFIRM

if [ "$CONFIRM" != \
     "EXECUTE-FR3-5MM-CONTINUOUS" ]
then
    echo "[BLOCK] execution not confirmed"
    exit 2
fi


echo
echo "========== 4. Revalidate start state =========="

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

    error = max_abs_delta(
        live,
        plan["start_q"],
    )

    print(
        f"start q error = "
        f"{error:.6f} rad"
    )

    if error > 0.003:
        print(
            "[BLOCK] robot moved since planning"
        )

        raise SystemExit(2)

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
PY

if [ "$?" -ne 0 ]; then
    exit 2
fi


echo
echo "========== 5. Register official controller =========="

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
echo "========== 6. Load controller =========="

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
echo "========== 7. Configure controller =========="

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
echo "========== 8. Activate controller =========="

if ! ros2 control switch_controllers \
      -h \
      >/dev/null 2>&1
then
    echo "[BLOCK] switch_controllers CLI unavailable"
    exit 2
fi

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
    echo "[FAIL] controller is not active"
    exit 2
fi

echo "[PASS] trajectory controller active"


echo
echo "========== 9. Execute ONE trajectory goal =========="

python3 "$EXECUTOR" \
  --plan "$PLAN"

EXEC_RC=$?

echo "trajectory_execution_result=$EXEC_RC"

if [ "$EXEC_RC" -ne 0 ]; then
    echo "[FAULT] short validation failed"
    echo "[SAFE] no retry"
    echo "[SAFE] no automatic return"
    exit 2
fi


echo
echo "========== 10. Deactivate + unload =========="

ros2 control switch_controllers \
  --deactivate fr3_arm_controller \
  --strict \
  -c /controller_manager

if [ "$?" -ne 0 ]; then
    echo "[FAULT] controller deactivation failed"
    exit 2
fi

CONTROLLER_ACTIVE=0

ros2 control unload_controller \
  -c /controller_manager \
  fr3_arm_controller

if [ "$?" -ne 0 ]; then
    echo "[FAULT] controller unload failed"
    exit 2
fi

CONTROLLER_LOADED=0


echo
echo "========== 11. Final pose =========="

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

    error = math.sqrt(
        sum(
            (a - b) ** 2
            for a, b in zip(
                xyz,
                target,
            )
        )
    )

    print(
        "current xyz =",
        [
            round(v, 6)
            for v in xyz
        ],
    )

    print(
        f"target position error = "
        f"{error * 1000:.3f} mm"
    )

    if error > 0.003:
        print(
            "[WARN] final Cartesian error > 3 mm"
        )

finally:

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
PY


echo
echo "========== 12. Final system state =========="

ros2 control list_controllers \
  2>/dev/null || true

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
echo "[PASS] FR3 5 mm CONTINUOUS VALIDATION COMPLETE"
echo "============================================================"
echo "ONE FollowJointTrajectory goal"
echo "NO PTP"
echo "NO RETRY"
echo "NO AUTO RETURN"
echo "NO XHAND"
echo "============================================================"

trap - EXIT INT TERM

exit 0
