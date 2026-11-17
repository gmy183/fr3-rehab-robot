#!/usr/bin/env bash
set -Eeuo pipefail

source "$HOME/rehab_robot_ws/env.sh"

WS="$HOME/rehab_robot_ws"
CONTROL="$WS/src/rehab_robot_control"
CONFIG="$WS/config"
TOOLS="$WS/tools"

echo "============================================================"
echo "1. 更新已过期 FR3 gate 检查"
echo "============================================================"

python3 - <<'PY'
from pathlib import Path

paths = [
    Path.home() / "rehab_robot_ws/tools/verify_project_runtime_config.sh",
    Path.home() / "rehab_robot_ws/tools/verify_motion_config_gate.sh",
]

for path in paths:
    text = path.read_text(encoding="utf-8")

    text = text.replace(
        'assert fm["joint_limits"]["loaded"] is False',
        'assert fm["joint_limits"]["loaded"] is True',
    )

    text = text.replace(
        'assert safety["joint_limits"]["loaded"] is False',
        'assert safety["joint_limits"]["loaded"] is True',
    )

    text = text.replace(
        "FR3 official joint limits still required before real goal",
        "FR3 official joint limits loaded from current franka_description",
    )

    text = text.replace(
        "FR3 official limits required before goal",
        "FR3 official limits loaded and required before goal",
    )

    path.write_text(text, encoding="utf-8")
    print("[PASS]", path.name)
PY


echo
echo "============================================================"
echo "2. 创建最终 XHand 运动配置"
echo "============================================================"

cat > "$CONFIG/xhand_motion_profile.yaml" <<'YAML'
xhand:
  mode: 3

  active_gains:
    verified: false
    source: pending_real_device_parameter_read
    kp: null
    ki: null
    kd: null

  # 最终安全配置优先：
  # thumb rotation 固定；
  # index_bend_joint 最终审查中继续排除。
  fixed_joints:
    - thumb_rota_joint1
    - thumb_rota_joint2

  excluded_joints:
    - index_bend_joint

  single_joint:
    joint: index_joint1
    delta_rad: 0.005
    effort_limit_ma: 150

  safe_whole_hand_group:
    active_joints:
      - thumb_bend_joint
      - index_joint1
      - index_joint2
      - mid_joint1
      - mid_joint2
      - ring_joint1
      - ring_joint2
      - pinky_joint1
      - pinky_joint2

    delta_rad: 0.005
    effort_limit_ma: 220

  inactive_joint:
    mode: 0
    kp: 0
    ki: 0
    kd: 0
    tor_max: 0

  watchdog:
    command_timeout_s: 0.5
    disable_frames: 3
    hold_refresh_hz: 20.0

  hardware_motion_gate:
    require_verified_active_gains: true
    require_live_position_baseline: true
    require_live_direction_validation: true
    require_empty_load: true
YAML


echo
echo "============================================================"
echo "3. XHand 命令契约"
echo "============================================================"

cat > "$CONTROL/rehab_robot_control/xhand_motion_contract.py" <<'PY'
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
from typing import Optional

import yaml

from .xhand_sdk_adapter import (
    JointCommandSpec,
    build_command,
)
from .xhand_state_mapper import JOINT_NAMES


class XHandMotionGateError(RuntimeError):
    pass


@dataclass(frozen=True)
class ActiveGains:
    mode: int
    kp: Optional[int]
    ki: Optional[int]
    kd: Optional[int]
    verified: bool


def load_profile(path: Path | None = None):
    if path is None:
        path = (
            Path.home()
            / "rehab_robot_ws/config/xhand_motion_profile.yaml"
        )

    return yaml.safe_load(
        path.read_text(encoding="utf-8")
    )["xhand"]


def load_active_gains(profile=None) -> ActiveGains:
    if profile is None:
        profile = load_profile()

    gains = profile["active_gains"]

    return ActiveGains(
        mode=int(profile["mode"]),
        kp=gains["kp"],
        ki=gains["ki"],
        kd=gains["kd"],
        verified=bool(gains["verified"]),
    )


def require_verified_gains(gains: ActiveGains) -> None:
    if not gains.verified:
        raise XHandMotionGateError(
            "XHand active kp/ki/kd are not verified"
        )

    if None in (gains.kp, gains.ki, gains.kd):
        raise XHandMotionGateError(
            "verified XHand gains contain missing values"
        )


def validate_positions(values):
    result = tuple(float(v) for v in values)

    if len(result) != 12:
        raise XHandMotionGateError(
            "XHand position vector must contain 12 joints"
        )

    if not all(math.isfinite(v) for v in result):
        raise XHandMotionGateError(
            "XHand position vector contains non-finite value"
        )

    return result


def validate_active_joints(names, profile=None):
    if profile is None:
        profile = load_profile()

    names = tuple(names)

    unknown = [
        name for name in names
        if name not in JOINT_NAMES
    ]

    if unknown:
        raise XHandMotionGateError(
            "unknown XHand joints: "
            + ", ".join(unknown)
        )

    forbidden = set(
        profile["fixed_joints"]
        + profile["excluded_joints"]
    )

    bad = [
        name for name in names
        if name in forbidden
    ]

    if bad:
        raise XHandMotionGateError(
            "forbidden active XHand joints: "
            + ", ".join(bad)
        )

    return names


def plan_relative_motion(
    current_positions,
    *,
    active_joints,
    delta_rad,
    effort_limit_ma,
    profile=None,
):
    if profile is None:
        profile = load_profile()

    current = validate_positions(
        current_positions
    )

    active_joints = validate_active_joints(
        active_joints,
        profile=profile,
    )

    delta = float(delta_rad)

    if not math.isfinite(delta):
        raise XHandMotionGateError(
            "delta_rad must be finite"
        )

    if abs(delta) > 0.005 + 1e-12:
        raise XHandMotionGateError(
            "first-stage XHand delta exceeds 0.005 rad"
        )

    effort = int(effort_limit_ma)

    if effort <= 0 or effort > 220:
        raise XHandMotionGateError(
            "XHand effort limit must be in (0, 220] mA"
        )

    target = list(current)

    active_set = set(active_joints)

    for index, name in enumerate(JOINT_NAMES):
        if name in active_set:
            target[index] += delta

    return {
        "active_joints": active_joints,
        "current_positions": current,
        "target_positions": tuple(target),
        "delta_rad": delta,
        "effort_limit_ma": effort,
    }


def build_verified_sdk_command(
    plan,
    *,
    gains: ActiveGains,
):
    require_verified_gains(gains)

    active = set(
        plan["active_joints"]
    )

    specs = []

    for index, name in enumerate(JOINT_NAMES):
        if name in active:
            specs.append(
                JointCommandSpec(
                    position=plan[
                        "target_positions"
                    ][index],
                    effort_limit=int(
                        plan["effort_limit_ma"]
                    ),
                    mode=int(gains.mode),
                    kp=int(gains.kp),
                    ki=int(gains.ki),
                    kd=int(gains.kd),
                )
            )
        else:
            specs.append(
                JointCommandSpec(
                    position=plan[
                        "current_positions"
                    ][index],
                    effort_limit=0,
                    mode=0,
                    kp=0,
                    ki=0,
                    kd=0,
                )
            )

    return build_command(specs)
PY


echo
echo "============================================================"
echo "4. FR3 PTP Client 安全骨架"
echo "============================================================"

cat > "$CONTROL/rehab_robot_control/fr3_ptp_client.py" <<'PY'
from __future__ import annotations

from rclpy.action import ActionClient

from franka_msgs.action import PTPMotion

from .fr3_ptp_contract import (
    FR3PTPContractError,
    PTP_ACTION_TYPE,
    build_goal,
    discover_unique_ptp_action,
)


class FR3PTPExecutionGateError(RuntimeError):
    pass


def discover_from_node(node):
    graph = node.get_action_names_and_types()

    return discover_unique_ptp_action(
        graph
    )


class FR3PTPClient:
    """
    Real ActionClient wrapper.

    Merely constructing this object does not send a goal.
    """

    def __init__(
        self,
        node,
        action_name: str,
    ):
        if not action_name:
            raise FR3PTPContractError(
                "PTP action name must not be empty"
            )

        self._node = node
        self._action_name = action_name

        self._client = ActionClient(
            node,
            PTPMotion,
            action_name,
        )

    @property
    def action_name(self):
        return self._action_name

    def wait_ready(self, timeout_sec=2.0):
        return self._client.wait_for_server(
            timeout_sec=float(timeout_sec)
        )

    def build_goal(
        self,
        target,
        *,
        maximum_velocity_rad_s=0.020,
        goal_tolerance=0.001,
    ):
        return build_goal(
            target,
            maximum_velocity_rad_s=(
                maximum_velocity_rad_s
            ),
            goal_tolerance=goal_tolerance,
        )

    def send_goal_async(
        self,
        goal,
        *,
        execute=False,
    ):
        if not execute:
            raise FR3PTPExecutionGateError(
                "PTP goal blocked: execute flag is false"
            )

        return self._client.send_goal_async(
            goal
        )
PY


echo
echo "============================================================"
echo "5. Empty-Grasp dry-run 执行器"
echo "============================================================"

cat > "$CONTROL/rehab_robot_control/empty_grasp_dry_run.py" <<'PY'
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from .fr3_ptp_contract import (
    build_goal,
    load_joint_limits,
    relative_target,
)
from .xhand_motion_contract import (
    XHandMotionGateError,
    load_active_gains,
    load_profile,
    plan_relative_motion,
    require_verified_gains,
)


class EmptyGraspExecutionBlocked(RuntimeError):
    pass


def _safe_fr3_midpoint():
    limits = load_joint_limits()

    values = []

    for i in range(1, 8):
        item = limits[
            f"fr3_joint{i}"
        ]

        values.append(
            (
                float(item["safe_lower_rad"])
                + float(item["safe_upper_rad"])
            )
            / 2.0
        )

    return tuple(values)


def load_sequence():
    path = (
        Path.home()
        / "rehab_robot_ws/config/"
          "empty_grasp_sequence.yaml"
    )

    return yaml.safe_load(
        path.read_text(encoding="utf-8")
    )


def build_dry_run_plan():
    seq = load_sequence()
    profile = load_profile()

    fr3_cfg = seq["fr3"]

    fr3_start = _safe_fr3_midpoint()

    fr3_pregrasp = relative_target(
        fr3_start,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "pregrasp_delta_joint1_rad"
            ]
        ),
    )

    fr3_grasp = relative_target(
        fr3_pregrasp,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "grasp_delta_joint1_rad"
            ]
        ),
    )

    fr3_rehab = relative_target(
        fr3_grasp,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "rehab_stage_1"
            ][
                "delta_joint1_rad"
            ]
        ),
    )

    fr3_goals = [
        build_goal(fr3_pregrasp),
        build_goal(fr3_grasp),
        build_goal(fr3_rehab),
    ]

    xhand_start = [0.0] * 12

    single_cfg = profile[
        "single_joint"
    ]

    single = plan_relative_motion(
        xhand_start,
        active_joints=[
            single_cfg["joint"]
        ],
        delta_rad=single_cfg[
            "delta_rad"
        ],
        effort_limit_ma=single_cfg[
            "effort_limit_ma"
        ],
        profile=profile,
    )

    whole_cfg = profile[
        "safe_whole_hand_group"
    ]

    whole = plan_relative_motion(
        xhand_start,
        active_joints=whole_cfg[
            "active_joints"
        ],
        delta_rad=whole_cfg[
            "delta_rad"
        ],
        effort_limit_ma=whole_cfg[
            "effort_limit_ma"
        ],
        profile=profile,
    )

    return {
        "sequence": seq["sequence"],
        "fr3_start": fr3_start,
        "fr3_goals": fr3_goals,
        "xhand_single": single,
        "xhand_whole": whole,
    }


def require_real_execution_gate():
    profile = load_profile()
    gains = load_active_gains(
        profile
    )

    try:
        require_verified_gains(
            gains
        )
    except XHandMotionGateError as exc:
        raise EmptyGraspExecutionBlocked(
            str(exc)
        ) from exc


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    ns = parser.parse_args(args)

    plan = build_dry_run_plan()

    print("=" * 60)
    print("FR3 + XHand Empty-Grasp software plan")
    print("=" * 60)

    print()
    print("sequence:")

    for index, stage in enumerate(
        plan["sequence"],
        1,
    ):
        print(
            f"  {index:02d}. {stage}"
        )

    print()
    print("FR3:")
    print(
        "  software limits: loaded"
    )
    print(
        "  max velocity: 0.020 rad/s"
    )
    print(
        "  relative PTP goals: 3"
    )

    for i, goal in enumerate(
        plan["fr3_goals"],
        1,
    ):
        print(
            f"  goal{i}: "
            f"{list(goal.goal_joint_configuration)}"
        )

    print()
    print("XHand single:")
    print(
        "  joints =",
        list(
            plan[
                "xhand_single"
            ][
                "active_joints"
            ]
        ),
    )
    print(
        "  delta =",
        plan[
            "xhand_single"
        ][
            "delta_rad"
        ],
    )
    print(
        "  limit =",
        plan[
            "xhand_single"
        ][
            "effort_limit_ma"
        ],
        "mA",
    )

    print()
    print("XHand safe whole-hand group:")
    print(
        "  joints =",
        list(
            plan[
                "xhand_whole"
            ][
                "active_joints"
            ]
        ),
    )
    print(
        "  delta =",
        plan[
            "xhand_whole"
        ][
            "delta_rad"
        ],
    )
    print(
        "  limit =",
        plan[
            "xhand_whole"
        ][
            "effort_limit_ma"
        ],
        "mA",
    )

    if ns.execute:
        print()
        print(
            "[CHECK] real execution gate"
        )

        require_real_execution_gate()

        # No real execution implementation is reached
        # in the current stage.
        raise EmptyGraspExecutionBlocked(
            "real execution backend is not enabled"
        )

    print()
    print(
        "[PASS] DRY-RUN ONLY"
    )
    print(
        "No ROS Action goal sent."
    )
    print(
        "No XHand command sent."
    )
    print(
        "No hardware opened."
    )


if __name__ == "__main__":
    main()
PY


echo
echo "============================================================"
echo "6. 软件测试"
echo "============================================================"

cat > "$CONTROL/test/test_xhand_motion_contract.py" <<'PY'
import pytest

from rehab_robot_control.xhand_motion_contract import (
    ActiveGains,
    XHandMotionGateError,
    build_verified_sdk_command,
    load_profile,
    plan_relative_motion,
)


def verified_test_gains():
    # TEST-ONLY arbitrary values.
    # They are never a project hardware profile.
    return ActiveGains(
        mode=3,
        kp=1,
        ki=0,
        kd=0,
        verified=True,
    )


def test_single_joint_plan():
    profile = load_profile()

    plan = plan_relative_motion(
        [0.0] * 12,
        active_joints=["index_joint1"],
        delta_rad=0.005,
        effort_limit_ma=150,
        profile=profile,
    )

    assert plan["active_joints"] == (
        "index_joint1",
    )

    assert (
        plan["target_positions"][4]
        == pytest.approx(0.005)
    )


def test_delta_above_project_limit_rejected():
    with pytest.raises(
        XHandMotionGateError,
        match="exceeds 0.005",
    ):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["index_joint1"],
            delta_rad=0.006,
            effort_limit_ma=150,
        )


def test_effort_above_project_limit_rejected():
    with pytest.raises(
        XHandMotionGateError,
        match="220",
    ):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["index_joint1"],
            delta_rad=0.005,
            effort_limit_ma=221,
        )


def test_index_bend_is_forbidden():
    with pytest.raises(
        XHandMotionGateError,
        match="forbidden",
    ):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["index_bend_joint"],
            delta_rad=0.005,
            effort_limit_ma=150,
        )


def test_thumb_rotation_is_fixed():
    with pytest.raises(
        XHandMotionGateError,
        match="forbidden",
    ):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["thumb_rota_joint1"],
            delta_rad=0.005,
            effort_limit_ma=150,
        )


def test_real_profile_gains_block_sdk_command():
    profile = load_profile()

    plan = plan_relative_motion(
        [0.0] * 12,
        active_joints=["index_joint1"],
        delta_rad=0.005,
        effort_limit_ma=150,
        profile=profile,
    )

    gains = ActiveGains(
        mode=3,
        kp=None,
        ki=None,
        kd=None,
        verified=False,
    )

    with pytest.raises(
        XHandMotionGateError,
        match="not verified",
    ):
        build_verified_sdk_command(
            plan,
            gains=gains,
        )


def test_inactive_joints_are_powerless():
    plan = plan_relative_motion(
        [0.0] * 12,
        active_joints=["index_joint1"],
        delta_rad=0.005,
        effort_limit_ma=150,
    )

    command = build_verified_sdk_command(
        plan,
        gains=verified_test_gains(),
    )

    assert len(
        command.finger_command
    ) == 12

    for index, finger in enumerate(
        command.finger_command
    ):
        if index == 4:
            assert int(finger.mode) == 3
            assert int(finger.tor_max) == 150
        else:
            assert int(finger.mode) == 0
            assert int(finger.kp) == 0
            assert int(finger.ki) == 0
            assert int(finger.kd) == 0
            assert int(finger.tor_max) == 0
PY


cat > "$CONTROL/test/test_empty_grasp_dry_run.py" <<'PY'
import pytest

from rehab_robot_control.empty_grasp_dry_run import (
    EmptyGraspExecutionBlocked,
    build_dry_run_plan,
    require_real_execution_gate,
)


def test_dry_run_plan():
    plan = build_dry_run_plan()

    assert len(
        plan["fr3_goals"]
    ) == 3

    assert (
        plan["xhand_single"][
            "delta_rad"
        ]
        == pytest.approx(0.005)
    )

    assert (
        plan["xhand_single"][
            "effort_limit_ma"
        ]
        == 150
    )

    assert (
        plan["xhand_whole"][
            "effort_limit_ma"
        ]
        == 220
    )


def test_real_execute_is_fail_closed():
    with pytest.raises(
        EmptyGraspExecutionBlocked,
        match="not verified",
    ):
        require_real_execution_gate()
PY


echo
echo "============================================================"
echo "7. 注册 dry-run 可执行入口"
echo "============================================================"

python3 - "$CONTROL/setup.py" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(
    encoding="utf-8"
)

entry = (
    '            "rehab_empty_grasp_dry_run = '
    'rehab_robot_control.empty_grasp_dry_run:main",\n'
)

if "rehab_empty_grasp_dry_run =" not in text:
    anchor = (
        '            "xhand_control_ros2_node = '
        'rehab_robot_control.xhand_ros2_node:main",\n'
    )

    if anchor not in text:
        raise SystemExit(
            "setup.py entry-point anchor not found"
        )

    text = text.replace(
        anchor,
        anchor + entry,
        1,
    )

    path.write_text(
        text,
        encoding="utf-8",
    )

print("[PASS] rehab_empty_grasp_dry_run entry")
PY


echo
echo "============================================================"
echo "8. 构建 control 包"
echo "============================================================"

cd "$WS"

colcon build \
  --merge-install \
  --symlink-install \
  --packages-select rehab_robot_control

source "$WS/env.sh"


echo
echo "============================================================"
echo "9. 创建最终软件验收脚本"
echo "============================================================"

cat > "$TOOLS/verify_motion_dryrun_stack.sh" <<'SH'
#!/usr/bin/env bash
set -Eeo pipefail

source "$HOME/rehab_robot_ws/env.sh"

CONTROL="$HOME/rehab_robot_ws/src/rehab_robot_control"

echo "============================================================"
echo "Motion dry-run stack verification"
echo "============================================================"

python3 -m py_compile \
  "$CONTROL/rehab_robot_control/fr3_ptp_contract.py" \
  "$CONTROL/rehab_robot_control/fr3_ptp_client.py" \
  "$CONTROL/rehab_robot_control/xhand_motion_contract.py" \
  "$CONTROL/rehab_robot_control/empty_grasp_dry_run.py"

echo "[PASS] syntax"

echo
echo "========== pytest =========="

python3 -m pytest -q \
  "$CONTROL/test/test_fr3_ptp_contract.py" \
  "$CONTROL/test/test_xhand_motion_contract.py" \
  "$CONTROL/test/test_empty_grasp_dry_run.py"

echo
echo "========== Empty-Grasp dry run =========="

ros2 run \
  rehab_robot_control \
  rehab_empty_grasp_dry_run

echo
echo "========== Real execute gate =========="

LOG="$(mktemp)"

set +e

ros2 run \
  rehab_robot_control \
  rehab_empty_grasp_dry_run \
  --execute \
  >"$LOG" 2>&1

RC=$?

set -e

cat "$LOG"

if grep -Fq \
  "XHand active kp/ki/kd are not verified" \
  "$LOG"
then
    echo "[PASS] real execution correctly blocked"
else
    echo "[FAIL] real execution gate reason missing"
    rm -f "$LOG"
    exit 1
fi

if [ "$RC" -eq 0 ]; then
    echo "[FAIL] --execute unexpectedly succeeded"
    rm -f "$LOG"
    exit 1
fi

rm -f "$LOG"

echo
echo "============================================================"
echo "[PASS] Motion dry-run stack verified"
echo
echo "No FR3 PTP goal sent."
echo "No XHand command sent."
echo "No EtherCAT opened."
echo "No D435i started."
echo "============================================================"
SH

chmod +x \
  "$TOOLS/verify_motion_dryrun_stack.sh"


echo
echo "============================================================"
echo "10. VSCode 加入运动软件任务"
echo "============================================================"

python3 - <<'PY'
import json
from pathlib import Path

path = (
    Path.home()
    / "rehab_robot_ws/.vscode/tasks.json"
)

data = json.loads(
    path.read_text(
        encoding="utf-8"
    )
)

new_tasks = [
    {
        "label": "30｜运动层软件总验收",
        "type": "shell",
        "command": "bash tools/verify_motion_dryrun_stack.sh"
    },
    {
        "label": "31｜联合空握｜Dry Run",
        "type": "shell",
        "command": "ros2 run rehab_robot_control rehab_empty_grasp_dry_run"
    },
    {
        "label": "32｜联合空握｜检查真实执行门禁",
        "type": "shell",
        "command": "ros2 run rehab_robot_control rehab_empty_grasp_dry_run --execute || true"
    },
    {
        "label": "33｜FR3｜重新解析官方关节限位",
        "type": "shell",
        "command": "python3 tools/resolve_fr3_motion_contract.py"
    }
]

labels = {
    item["label"]
    for item in new_tasks
}

tasks = [
    item
    for item in data.get(
        "tasks",
        []
    )
    if item.get("label")
    not in labels
]

for task in new_tasks:
    task["options"] = {
        "cwd": "${workspaceFolder}"
    }

    task["presentation"] = {
        "reveal": "always",
        "panel": "dedicated",
        "clear": False
    }

    task["problemMatcher"] = []

tasks.extend(new_tasks)

data["tasks"] = tasks

path.write_text(
    json.dumps(
        data,
        ensure_ascii=False,
        indent=2,
    ) + "\n",
    encoding="utf-8",
)

print("[PASS] VSCode motion tasks")
PY


echo
echo "============================================================"
echo "11. 执行验收"
echo "============================================================"

bash "$TOOLS/verify_project_runtime_config.sh"
bash "$TOOLS/verify_motion_config_gate.sh"
bash "$TOOLS/verify_motion_dryrun_stack.sh"

echo
echo "============================================================"
echo "[PASS] FR3 + XHand 运动软件框架配置完成"
echo
echo "真实执行仍然 fail-closed："
echo "XHand active kp/ki/kd 尚未通过真实设备参数/接管验证。"
echo
echo "没有启动任何设备。"
echo "============================================================"
