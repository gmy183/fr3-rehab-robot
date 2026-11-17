from __future__ import annotations

from dataclasses import dataclass
import math


class RehabTaskContractError(ValueError):
    pass


@dataclass(frozen=True)
class RehabTaskConfig:
    """
    High-level rehabilitation task definition.

    No FR3 joint target or XHand SDK command belongs here.
    Those are supplied later by the hardware execution layer.
    """

    task_name: str
    motion_profile: str
    repetitions: int
    speed_scale: float
    grasp_profile: str
    record_data: bool = True


def validate_task_config(
    config: RehabTaskConfig,
) -> RehabTaskConfig:

    if not isinstance(
        config.task_name,
        str,
    ) or not config.task_name.strip():
        raise RehabTaskContractError(
            "task_name must be non-empty"
        )

    if not isinstance(
        config.motion_profile,
        str,
    ) or not config.motion_profile.strip():
        raise RehabTaskContractError(
            "motion_profile must be non-empty"
        )

    if not isinstance(
        config.grasp_profile,
        str,
    ) or not config.grasp_profile.strip():
        raise RehabTaskContractError(
            "grasp_profile must be non-empty"
        )

    repetitions = int(
        config.repetitions
    )

    if not 1 <= repetitions <= 20:
        raise RehabTaskContractError(
            "repetitions must be in [1, 20]"
        )

    speed_scale = float(
        config.speed_scale
    )

    if not math.isfinite(
        speed_scale
    ):
        raise RehabTaskContractError(
            "speed_scale must be finite"
        )

    if not 0.0 < speed_scale <= 1.0:
        raise RehabTaskContractError(
            "speed_scale must be in (0, 1]"
        )

    return config


def default_task_config():
    """
    First engineering rehabilitation task.

    motion_profile and grasp_profile are symbolic names.
    No physical trajectory is executed by this module.
    """

    return RehabTaskConfig(
        task_name="rehab_task_01",
        motion_profile="rehab_motion_01",
        repetitions=1,
        speed_scale=0.20,
        grasp_profile="xhand_rehab_hold_01",
        record_data=True,
    )
