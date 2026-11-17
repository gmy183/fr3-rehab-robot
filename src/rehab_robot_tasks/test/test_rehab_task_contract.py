import pytest

from rehab_robot_tasks.rehab_task_contract import (
    RehabTaskConfig,
    RehabTaskContractError,
    default_task_config,
    validate_task_config,
)


def test_default_config_valid():
    config = default_task_config()

    assert (
        validate_task_config(config)
        == config
    )


def test_empty_task_name_rejected():
    config = RehabTaskConfig(
        task_name="",
        motion_profile="motion",
        repetitions=1,
        speed_scale=0.2,
        grasp_profile="grasp",
    )

    with pytest.raises(
        RehabTaskContractError
    ):
        validate_task_config(
            config
        )


def test_invalid_repetitions_rejected():
    config = RehabTaskConfig(
        task_name="task",
        motion_profile="motion",
        repetitions=0,
        speed_scale=0.2,
        grasp_profile="grasp",
    )

    with pytest.raises(
        RehabTaskContractError
    ):
        validate_task_config(
            config
        )


def test_invalid_speed_scale_rejected():
    config = RehabTaskConfig(
        task_name="task",
        motion_profile="motion",
        repetitions=1,
        speed_scale=1.1,
        grasp_profile="grasp",
    )

    with pytest.raises(
        RehabTaskContractError
    ):
        validate_task_config(
            config
        )
