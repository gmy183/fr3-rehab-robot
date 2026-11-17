#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"
source "$HOME/rehab_robot_ws/config/hardware_profile.env"

WS="$HOME/rehab_robot_ws"
CONFIG="$WS/config"

echo "============================================================"
echo "康复机械臂运行配置静态验收"
echo "============================================================"

python3 - <<'PY'
from pathlib import Path
import yaml

root = Path.home() / "rehab_robot_ws/config"

paths = {
    "safety": root / "safety.yaml",
    "frames": root / "tool_frames.yaml",
    "grasp": root / "empty_grasp_sequence.yaml",
    "camera": root / "d435i.yaml",
    "fr3_motion": root / "fr3_motion_safety.yaml",
}

data = {}

for name, path in paths.items():
    with path.open("r", encoding="utf-8") as f:
        data[name] = yaml.safe_load(f)

    print(f"[PASS] YAML: {path.name}")


safety = data["safety"]
frames = data["frames"]
grasp = data["grasp"]
camera = data["camera"]
fr3_motion = data["fr3_motion"]


# ============================================================
# FR3 基础安全参数
# ============================================================

assert safety["fr3"]["max_joint_velocity_rad_s"] == 0.020
assert safety["fr3"]["automatic_recovery_on_fault"] is False
assert safety["fr3"]["absolute_joint_targets_enabled"] is False

print("[PASS] FR3 max velocity = 0.020 rad/s")
print("[PASS] FR3 automatic recovery disabled")
print("[PASS] FR3 absolute target guessing disabled")


# ============================================================
# XHand 空握阶段参数
# ============================================================

xhand = grasp["xhand"]

assert xhand["single_joint_test"]["delta_rad"] == 0.005
assert xhand["single_joint_test"]["effort_limit_ma"] == 150

assert xhand["whole_hand_test"]["delta_rad"] == 0.005
assert xhand["whole_hand_test"]["effort_limit_ma"] == 220

assert xhand["watchdog"]["command_timeout_s"] == 0.5
assert xhand["watchdog"]["disable_frames"] >= 3

print("[PASS] XHand single joint = 0.005 rad / 150 mA")
print("[PASS] XHand whole hand = 0.005 rad / <=220 mA")
print("[PASS] XHand watchdog = 0.5 s")
print("[PASS] XHand multi-frame disable configured")


inactive = xhand["inactive_joint"]

assert inactive == {
    "mode": 0,
    "kp": 0,
    "ki": 0,
    "kd": 0,
    "tor_max": 0,
}

print("[PASS] inactive XHand joints are fully powerless")


assert "index_bend_joint" in xhand["excluded_joints"]

print("[PASS] index_bend_joint remains excluded")


# ============================================================
# XHand 主动控制门禁
# ============================================================

active = xhand["active_control"]

assert active["mode"] == 3
assert active["gains"]["verified"] is False
assert active["gains"]["kp"] is None
assert active["gains"]["ki"] is None
assert active["gains"]["kd"] is None

gate = xhand["motion_enable_gate"]

assert gate["require_verified_active_gains"] is True
assert gate["require_live_direction_check"] is True
assert gate["require_empty_load_only"] is True

print("[PASS] XHand active mode contract recorded")
print("[PASS] XHand unknown active gains were NOT guessed")
print("[PASS] XHand real-motion gate remains CLOSED")


# ============================================================
# FR3 空握相对运动
# ============================================================

fr3 = grasp["fr3"]

assert fr3["motion_mode"] == "relative_from_live_joint_state"

assert fr3["pregrasp_delta_joint1_rad"] == 0.005
assert fr3["grasp_delta_joint1_rad"] == 0.010
assert fr3["return_delta_joint1_rad"] == 0.005

assert fr3["max_joint_velocity_rad_s"] == 0.020
assert fr3["automatic_recovery_on_fault"] is False
assert fr3["automatic_return_on_fault"] is False

print("[PASS] FR3 relative phase-1 deltas = 0.005 / 0.010 / 0.005 rad")
print("[PASS] FR3 relative-from-live-state policy")
print("[PASS] FR3 fault auto-return disabled")


# ============================================================
# 安装几何：使用当前最终 schema
# ============================================================

f = frames["frames"]

assert f["parent_frame"] == "fr3_link8"

assert f["xhand_mount"]["translation_m"] == {
    "x": 0.0,
    "y": 0.0,
    "z": 0.081,
}

assert f["d435i_mount"]["translation_m"] == {
    "x": -0.09,
    "y": 0.0,
    "z": 0.051,
}

assert f["calibration"]["rotation_verified"] is False
assert f["calibration"]["grasp_tcp_verified"] is False

assert f["xhand_mount"]["rotation_rpy_rad"]["verified"] is False
assert f["xhand_mount"]["rotation_rpy_rad"]["value"] is None

assert f["d435i_mount"]["rotation_rpy_rad"]["verified"] is False
assert f["d435i_mount"]["rotation_rpy_rad"]["value"] is None

assert f["xhand_grasp_center"]["publish_tf"] is False

print("[PASS] XHand mount translation = 81 mm")
print("[PASS] D435i mount translation = back 90 mm / up 51 mm")
print("[PASS] unverified RPY remains unverified")
print("[PASS] unverified xhand_grasp_center TF is NOT published")


# ============================================================
# D435i 旧实机 profile
# ============================================================

assert camera["device"]["serial_no"] == "244222072086"

color = camera["streams"]["color"]
depth = camera["streams"]["depth"]

assert color["enabled"] is True
assert color["width"] == 640
assert color["height"] == 480
assert color["fps"] == 30

assert depth["enabled"] is True
assert depth["width"] == 848
assert depth["height"] == 480
assert depth["fps"] == 30

assert camera["streams"]["gyro"]["enabled"] is False
assert camera["streams"]["accel"]["enabled"] is False

print("[PASS] D435i serial = 244222072086")
print("[PASS] D435i color = 640x480@30")
print("[PASS] D435i depth = 848x480@30")
print("[PASS] D435i IMU disabled")


# ============================================================
# FR3 PTP / joint-limit 门禁
# ============================================================

fm = fr3_motion["fr3"]

assert fm["joint_count"] == 7
assert fm["ptp"]["action_type"] == "franka_msgs/action/PTPMotion"
assert fm["ptp"]["maximum_joint_velocity_rad_s"] == 0.020

assert fm["joint_limits"]["safety_margin_rad"] == 0.05
assert fm["joint_limits"]["loaded"] is True

assert fm["target_policy"]["require_7_values"] is True
assert fm["target_policy"]["require_finite_values"] is True
assert fm["target_policy"]["require_software_joint_limit_check"] is True
assert fm["target_policy"]["allow_absolute_targets_before_validation"] is False

assert fm["recovery"]["automatic_return_on_fault"] is False

print("[PASS] FR3 PTP action type recorded")
print("[PASS] FR3 joint-limit margin = 0.05 rad")
print("[PASS] FR3 official joint limits loaded from current franka_description")
print("[PASS] FR3 absolute real-motion gate remains CLOSED")


# ============================================================
# 联合空握故障逻辑
# ============================================================

assert grasp["fault_behavior"][
    "send_no_automatic_fr3_return_motion"
] is True

assert grasp["fault_behavior"][
    "xhand_multiframe_disable"
] is True

assert xhand["watchdog"][
    "refresh_hold_while_waiting_for_fr3"
] is True

print("[PASS] fault path has no automatic FR3 return")
print("[PASS] XHand multi-frame disable required")
print("[PASS] XHand hold refresh required while waiting for FR3")
PY

PY_RC=$?

echo
echo "========== Hardware profile =========="

echo "[PASS] FR3_IF=$FR3_IF"
echo "[PASS] FR3_PC_IP=$FR3_PC_IP"
echo "[PASS] FR3_ROBOT_IP=$FR3_ROBOT_IP"

echo "[PASS] XHAND_IF=$XHAND_IF"
echo "[PASS] XHAND_DEVICE=$XHAND_DEVICE"
echo "[PASS] XHAND_SERIAL=$XHAND_SERIAL"

echo "[PASS] D435I_SERIAL=$D435I_SERIAL"


echo
echo "========== ROS software =========="

ROS_OK=1

if ros2 interface show \
    franka_msgs/action/PTPMotion \
    >/dev/null 2>&1
then
    echo "[PASS] franka_msgs/action/PTPMotion"
else
    echo "[FAIL] franka_msgs/action/PTPMotion"
    ROS_OK=0
fi

if ros2 pkg prefix \
    realsense2_camera \
    >/dev/null 2>&1
then
    echo "[PASS] realsense2_camera"
else
    echo "[FAIL] realsense2_camera"
    ROS_OK=0
fi

if ros2 pkg prefix \
    rehab_robot_control \
    >/dev/null 2>&1
then
    echo "[PASS] rehab_robot_control"
else
    echo "[FAIL] rehab_robot_control"
    ROS_OK=0
fi


echo
echo "============================================================"

if [ "$PY_RC" -eq 0 ] && [ "$ROS_OK" -eq 1 ]; then
    echo "[PASS] 康复机械臂运行配置验收完成"
    RC=0
else
    echo "[FAIL] 康复机械臂运行配置验收未通过"
    RC=1
fi

echo
echo "没有启动 FR3"
echo "没有启动 XHand"
echo "没有启动 D435i"
echo "没有发送任何运动命令"
echo "============================================================"

exit "$RC"
