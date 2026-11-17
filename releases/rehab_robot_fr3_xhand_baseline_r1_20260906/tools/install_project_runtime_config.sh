#!/usr/bin/env bash
set -Eeuo pipefail

WS="$HOME/rehab_robot_ws"
CONFIG="$WS/config"
TOOLS="$WS/tools"
VSCODE="$WS/.vscode"

source "$WS/env.sh"

mkdir -p "$CONFIG" "$TOOLS" "$VSCODE"


echo "============================================================"
echo "1. 恢复项目安全参数"
echo "============================================================"

cat > "$CONFIG/safety.yaml" <<'YAML'
project:
  name: rehab_robot
  stage: empty_grasp_recovery

global:
  automatic_motion_on_startup: false
  automatic_fault_return: false
  require_explicit_execute: true
  require_hardware_preflight: true

fr3:
  max_joint_velocity_rad_s: 0.020
  automatic_recovery_on_fault: false

  relative_motion_only_before_validation: true

  # 不猜测旧硬盘中已丢失的绝对七关节目标。
  absolute_joint_targets_enabled: false

xhand:
  command_timeout_s: 0.5
  state_timeout_s: 0.2

  default_micro_close_rad: 0.005
  single_joint_effort_limit_ma: 150
  whole_hand_effort_limit_ma: 220

  inactive_joint_mode: 0
  inactive_joint_kp: 0
  inactive_joint_ki: 0
  inactive_joint_kd: 0
  inactive_joint_tor_max: 0

  disable_frames: 3

  excluded_joints:
    - index_bend_joint

  fault_auto_return: false

d435i:
  enable_imu_during_empty_grasp: false
YAML


echo
echo "============================================================"
echo "2. 恢复安装几何"
echo "============================================================"

cat > "$CONFIG/tool_frames.yaml" <<'YAML'
frames:
  base_frame: fr3_link0
  flange_frame: fr3_link8

  xhand:
    frame: xhand_grasp_center

    # XHand 抓握中心相对 FR3 link8：
    # 垂直方向 81 mm
    translation_m:
      x: 0.000
      y: 0.000
      z: 0.081

    orientation:
      verified: false
      note: >
        平移尺寸来自旧项目记录。
        最终轴向/姿态在真机连接后只做物理核对，
        当前软件阶段不猜测旋转矩阵。

  d435i:
    frame: d435i_link

    # 相机相对 FR3 link8：
    # 垂直 51 mm，机械臂背部方向 90 mm
    translation_m:
      x: -0.090
      y: 0.000
      z: 0.051

    orientation:
      verified: false
      note: >
        平移安装尺寸已恢复。
        相机具体光轴符号和旋转方向最终真机核对。
YAML


echo
echo "============================================================"
echo "3. 恢复 Empty-Grasp 参数"
echo "============================================================"

cat > "$CONFIG/empty_grasp_sequence.yaml" <<'YAML'
project:
  name: rehab_robot
  sequence: fr3_xhand_empty_grasp
  mode: relative_safe_recovery

frames:
  base_frame: fr3_link0
  tcp_frame: xhand_grasp_center

preflight:
  require_fr3_state: true
  require_xhand_state: true
  require_xhand_command_interface: true
  require_fr3_ptp_action: true

  d435i_required_for_motion: false
  d435i_keep_online_if_available: true

xhand:
  joint_count: 12

  excluded_joints:
    - index_bend_joint

  single_joint_test:
    enabled: true
    joint_name: index_joint1
    delta_rad: 0.005
    effort_limit_ma: 150

  whole_hand_test:
    enabled: true
    delta_rad: 0.005
    effort_limit_ma: 220

  grasp:
    default_close_delta_rad: 0.005
    maximum_effort_limit_ma: 220

  watchdog:
    command_timeout_s: 0.5
    refresh_hold_while_waiting_for_fr3: true
    disable_frames: 3

  inactive_joint:
    mode: 0
    kp: 0
    ki: 0
    kd: 0
    tor_max: 0

fr3:
  motion_mode: relative_from_live_joint_state

  # 旧 phase-1 微小功能验证参数
  pregrasp_delta_joint1_rad: 0.005
  grasp_delta_joint1_rad: 0.010
  return_delta_joint1_rad: 0.005

  max_joint_velocity_rad_s: 0.020

  # 空握恢复阶段的小幅康复功能验证。
  rehab_stage_1:
    delta_joint1_rad: 0.005

  automatic_recovery_on_fault: false
  automatic_return_on_fault: false

timing:
  settle_time_s: 1.0
  hold_time_s: 1.0

sequence:
  - preflight_readonly
  - xhand_single_joint_0p005
  - xhand_whole_hand_0p005
  - fr3_relative_pregrasp
  - xhand_grasp
  - fr3_relative_grasp
  - fr3_rehab_stage_1
  - controlled_finish

fault_behavior:
  send_no_automatic_fr3_return_motion: true
  xhand_multiframe_disable: true
  require_operator_recovery: true
YAML


echo
echo "============================================================"
echo "4. 恢复 D435i 配置"
echo "============================================================"

cat > "$CONFIG/d435i.yaml" <<'YAML'
device:
  model: Intel RealSense D435i
  serial_no: "244222072086"

streams:
  color:
    enabled: true

  depth:
    enabled: true

  gyro:
    enabled: false

  accel:
    enabled: false

project_role:
  empty_grasp:
    required_for_control: false
    keep_online: true

  later_perception_stage:
    enabled: true
YAML


echo
echo "============================================================"
echo "5. 创建项目配置一致性验收"
echo "============================================================"

cat > "$TOOLS/verify_project_runtime_config.sh" <<'SH'
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

files = {
    "safety": root / "safety.yaml",
    "frames": root / "tool_frames.yaml",
    "grasp": root / "empty_grasp_sequence.yaml",
    "camera": root / "d435i.yaml",
}

data = {}

for name, path in files.items():
    with path.open("r", encoding="utf-8") as f:
        data[name] = yaml.safe_load(f)

    print(f"[PASS] YAML: {path.name}")


safety = data["safety"]
grasp = data["grasp"]
frames = data["frames"]
camera = data["camera"]


assert safety["fr3"]["max_joint_velocity_rad_s"] == 0.020
assert safety["fr3"]["automatic_recovery_on_fault"] is False
assert safety["fr3"]["absolute_joint_targets_enabled"] is False

print("[PASS] FR3 max velocity = 0.020 rad/s")
print("[PASS] FR3 automatic recovery disabled")
print("[PASS] FR3 absolute target guessing disabled")


assert grasp["xhand"]["single_joint_test"]["delta_rad"] == 0.005
assert grasp["xhand"]["single_joint_test"]["effort_limit_ma"] == 150

assert grasp["xhand"]["whole_hand_test"]["delta_rad"] == 0.005
assert grasp["xhand"]["whole_hand_test"]["effort_limit_ma"] == 220

assert grasp["xhand"]["watchdog"]["command_timeout_s"] == 0.5

print("[PASS] XHand single joint = 0.005 rad / 150 mA")
print("[PASS] XHand whole hand = 0.005 rad / <=220 mA")
print("[PASS] XHand watchdog = 0.5 s")


inactive = grasp["xhand"]["inactive_joint"]

assert inactive == {
    "mode": 0,
    "kp": 0,
    "ki": 0,
    "kd": 0,
    "tor_max": 0,
}

print("[PASS] inactive XHand joints are fully powerless")


assert grasp["fr3"]["pregrasp_delta_joint1_rad"] == 0.005
assert grasp["fr3"]["grasp_delta_joint1_rad"] == 0.010
assert grasp["fr3"]["return_delta_joint1_rad"] == 0.005

print("[PASS] FR3 relative phase-1 deltas")


xhand_xyz = frames["frames"]["xhand"]["translation_m"]
camera_xyz = frames["frames"]["d435i"]["translation_m"]

assert xhand_xyz == {
    "x": 0.0,
    "y": 0.0,
    "z": 0.081,
}

assert camera_xyz == {
    "x": -0.09,
    "y": 0.0,
    "z": 0.051,
}

print("[PASS] XHand translation = 81 mm")
print("[PASS] D435i translation = back 90 mm / up 51 mm")


assert camera["device"]["serial_no"] == "244222072086"

assert camera["streams"]["color"]["enabled"] is True
assert camera["streams"]["depth"]["enabled"] is True
assert camera["streams"]["gyro"]["enabled"] is False
assert camera["streams"]["accel"]["enabled"] is False

print("[PASS] D435i serial/config")
print("[PASS] D435i IMU disabled for current stage")


assert grasp["fault_behavior"][
    "send_no_automatic_fr3_return_motion"
] is True

assert grasp["xhand"]["watchdog"][
    "refresh_hold_while_waiting_for_fr3"
] is True

print("[PASS] fault path has no automatic FR3 return")
print("[PASS] XHand hold refresh required while waiting for FR3")
PY


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

ros2 interface show \
    franka_msgs/action/PTPMotion \
    >/dev/null

echo "[PASS] franka_msgs/action/PTPMotion"

ros2 pkg prefix \
    realsense2_camera \
    >/dev/null

echo "[PASS] realsense2_camera"

ros2 pkg prefix \
    rehab_robot_control \
    >/dev/null

echo "[PASS] rehab_robot_control"


echo
echo "============================================================"
echo "[PASS] 康复机械臂运行配置验收完成"
echo
echo "没有启动 FR3"
echo "没有启动 XHand"
echo "没有启动 D435i"
echo "没有发送任何运动命令"
echo "============================================================"
SH

chmod +x "$TOOLS/verify_project_runtime_config.sh"


echo
echo "============================================================"
echo "6. 加入 VSCode 配置验收任务"
echo "============================================================"

python3 - <<'PY'
import json
from pathlib import Path

path = Path.home() / "rehab_robot_ws/.vscode/tasks.json"

data = json.loads(
    path.read_text(encoding="utf-8")
)

label = "02｜项目运行参数验收（不启动设备）"

task = {
    "label": label,
    "type": "shell",
    "command": "bash tools/verify_project_runtime_config.sh",
    "options": {
        "cwd": "${workspaceFolder}"
    },
    "presentation": {
        "reveal": "always",
        "panel": "dedicated",
        "clear": False
    },
    "problemMatcher": []
}

tasks = [
    item for item in data.get("tasks", [])
    if item.get("label") != label
]

tasks.append(task)

data["tasks"] = tasks

path.write_text(
    json.dumps(
        data,
        ensure_ascii=False,
        indent=2
    ) + "\n",
    encoding="utf-8"
)

print("[PASS] VSCode task:", label)
PY


echo
echo "============================================================"
echo "7. 执行配置验收"
echo "============================================================"

bash "$TOOLS/verify_project_runtime_config.sh"


echo
echo "============================================================"
echo "[PASS] 项目运行参数已经恢复"
echo
echo "下一层：FR3 PTP + XHand command + Empty-Grasp executor"
echo "当前仍没有启动任何硬件"
echo "============================================================"
