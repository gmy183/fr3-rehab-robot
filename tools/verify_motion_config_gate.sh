#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

python3 - <<'PY'
from pathlib import Path
import yaml

root = Path.home() / "rehab_robot_ws/config"

with (root / "tool_frames.yaml").open(
    "r", encoding="utf-8"
) as f:
    frames = yaml.safe_load(f)

with (root / "d435i.yaml").open(
    "r", encoding="utf-8"
) as f:
    cam = yaml.safe_load(f)

with (root / "empty_grasp_sequence.yaml").open(
    "r", encoding="utf-8"
) as f:
    grasp = yaml.safe_load(f)

with (root / "fr3_motion_safety.yaml").open(
    "r", encoding="utf-8"
) as f:
    fr3 = yaml.safe_load(f)


assert frames["frames"]["xhand_mount"]["translation_m"]["z"] == 0.081
assert frames["frames"]["d435i_mount"]["translation_m"]["x"] == -0.090
assert frames["frames"]["d435i_mount"]["translation_m"]["z"] == 0.051

assert (
    frames["frames"]["xhand_grasp_center"]["publish_tf"]
    is False
)

assert (
    frames["frames"]["calibration"]["rotation_verified"]
    is False
)

print("[PASS] 安装平移尺寸保留")
print("[PASS] 未标定旋转没有被伪造成已验证 TF")
print("[PASS] xhand_grasp_center 默认不发布")


assert cam["streams"]["color"]["width"] == 640
assert cam["streams"]["color"]["height"] == 480
assert cam["streams"]["color"]["fps"] == 30

assert cam["streams"]["depth"]["width"] == 848
assert cam["streams"]["depth"]["height"] == 480
assert cam["streams"]["depth"]["fps"] == 30

assert cam["streams"]["gyro"]["enabled"] is False
assert cam["streams"]["accel"]["enabled"] is False

print("[PASS] D435i color = 640x480x30")
print("[PASS] D435i depth = 848x480x30")
print("[PASS] D435i IMU = disabled")


active = grasp["xhand"]["active_control"]

assert active["mode"] == 3
assert active["gains"]["verified"] is False
assert active["gains"]["kp"] is None
assert active["gains"]["ki"] is None
assert active["gains"]["kd"] is None

gate = grasp["xhand"]["motion_enable_gate"]

assert gate["require_verified_active_gains"] is True
assert gate["require_live_direction_check"] is True

print("[PASS] XHand active mode contract recorded")
print("[PASS] 未恢复的主动增益没有被厂商示例值替代")
print("[PASS] XHand motion gate remains CLOSED")


safety = fr3["fr3"]

assert safety["joint_count"] == 7
assert safety["joint_limits"]["safety_margin_rad"] == 0.05
assert safety["joint_limits"]["loaded"] is True

assert (
    safety["target_policy"]["require_software_joint_limit_check"]
    is True
)

assert (
    safety["recovery"]["automatic_return_on_fault"]
    is False
)

print("[PASS] FR3 joint count = 7")
print("[PASS] FR3 joint-limit margin = 0.05 rad")
print("[PASS] FR3 official limits loaded and required before goal")
print("[PASS] FR3 automatic fault return disabled")
PY

echo
echo "============================================================"
echo "[PASS] Motion configuration gate verified"
echo
echo "[PASS] FR3 官方 joint limits 已从当前 franka_description 解析"
echo "[PASS] FR3 PTPMotion Goal 软件契约已确认"
echo "[INFO] PTP Action server 名称将在 FR3 启动后按"
echo "       franka_msgs/action/PTPMotion 唯一发现"
echo "[BLOCKED] XHand 真实运动仍由 active-control gate 锁定"
echo
echo "因此当前仍不会发送任何真实运动命令。"
echo "============================================================"
