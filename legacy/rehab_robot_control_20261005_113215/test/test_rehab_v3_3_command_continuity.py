from pathlib import Path

from rehab_robot_control.v3.fr3_test_runner import FR3TestRunner


def _runner_text():
    p = Path.home() / "rehab_robot_ws/src/rehab_robot_control/rehab_robot_control/v3/fr3_test_runner.py"
    return p.read_text(encoding="utf-8")


def _tm_text():
    p = Path.home() / "rehab_robot_ws/src/rehab_robot_control/rehab_robot_control/v3/trajectory_manager.py"
    return p.read_text(encoding="utf-8")


def test_v3_3_desired_command_state_is_explicit():
    text = _tm_text()
    assert "class DesiredJointState" in text
    assert "def capture_desired_joint_state" in text
    assert "msg.desired_joint_state" in text
    assert "msg.ddq_d" in text


def test_v3_3_plan_starts_from_commanded_qd():
    text = _runner_text()
    assert '"command_start_q_d"' in text
    assert 'knots[0] = list(command_state.q_d)' in text
    assert '"positions": list(command_state.q_d)' in text
    assert "COMMAND_HOLD_PREFIX_S" in text


def test_v3_3_pre_goal_revalidates_command_state():
    text = _runner_text()
    assert "desired = planner.capture_desired_joint_state()" in text
    assert "commanded q_d changed after planning; no goal sent" in text
    assert "commanded dq_d is not stationary; no goal sent" in text
    assert "commanded ddq_d is not stationary; no goal sent" in text


def test_v3_3_controller_interpolates_from_desired_state():
    p = Path.home() / "rehab_robot_ws/config/fr3_position_v3_2_controller.yaml"
    text = p.read_text(encoding="utf-8")
    assert "command_interfaces:" in text
    assert "- position" in text
    assert "interpolate_from_desired_state: true" in text
    assert "set_last_command_interface_value_as_state_on_activation: true" in text


def test_v3_3_limits_remain_conservative():
    assert FR3TestRunner.TEST_DISTANCE_M == 0.005
    assert FR3TestRunner.TEST_MAX_JOINT_VELOCITY <= 0.005
    assert FR3TestRunner.TEST_MAX_JOINT_ACCELERATION <= 0.025
    assert FR3TestRunner.TEST_MAX_JOINT_JERK_RAD_S3 <= 5.0
    assert FR3TestRunner.COMMAND_HOLD_PREFIX_S >= 0.25
