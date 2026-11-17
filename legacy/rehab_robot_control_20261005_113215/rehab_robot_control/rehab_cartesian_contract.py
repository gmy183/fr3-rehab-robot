from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


class RehabCartesianContractError(
    ValueError
):
    pass


@dataclass(frozen=True)
class RelativeCartesianMove:
    """
    Relative motion expressed in the rehabilitation
    task frame.

    Translation: metres
    Rotation: radians
    """

    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0

    droll: float = 0.0
    dpitch: float = 0.0
    dyaw: float = 0.0


def default_config_path():
    return (
        Path.home()
        / "rehab_robot_ws/config/"
          "rehab_cartesian_motion.yaml"
    )


def load_cartesian_config(
    path=None,
):
    if path is None:
        path = default_config_path()

    path = Path(path)

    data = yaml.safe_load(
        path.read_text(
            encoding="utf-8"
        )
    )

    return data[
        "rehab_cartesian"
    ]


def translation_norm(
    move,
):
    return math.sqrt(
        move.dx ** 2
        + move.dy ** 2
        + move.dz ** 2
    )


def rotation_norm(
    move,
):
    return math.sqrt(
        move.droll ** 2
        + move.dpitch ** 2
        + move.dyaw ** 2
    )


def validate_relative_move(
    move,
    *,
    config=None,
):
    if config is None:
        config = load_cartesian_config()

    values = (
        move.dx,
        move.dy,
        move.dz,
        move.droll,
        move.dpitch,
        move.dyaw,
    )

    if not all(
        math.isfinite(float(v))
        for v in values
    ):
        raise RehabCartesianContractError(
            "Cartesian move contains "
            "non-finite value"
        )

    safety = config[
        "safety"
    ]

    max_translation = float(
        safety[
            "max_translation_per_segment_m"
        ]
    )

    max_rotation = float(
        safety[
            "max_rotation_per_segment_rad"
        ]
    )

    tnorm = translation_norm(
        move
    )

    rnorm = rotation_norm(
        move
    )

    if tnorm > max_translation + 1e-12:
        raise RehabCartesianContractError(
            "Cartesian translation exceeds "
            f"segment limit: "
            f"{tnorm:.6f} m > "
            f"{max_translation:.6f} m"
        )

    if rnorm > max_rotation + 1e-12:
        raise RehabCartesianContractError(
            "Cartesian rotation exceeds "
            f"segment limit: "
            f"{rnorm:.6f} rad > "
            f"{max_rotation:.6f} rad"
        )

    return move


def require_real_cartesian_gate(
    *,
    config=None,
):
    if config is None:
        config = load_cartesian_config()

    robot = config[
        "robot"
    ]

    safety = config[
        "safety"
    ]

    execution = config[
        "execution"
    ]

    if not bool(
        robot[
            "tool_frame_calibrated"
        ]
    ):
        raise RehabCartesianContractError(
            "xhand_rehab_tcp is not calibrated"
        )

    if not bool(
        safety[
            "patient_motion_profile_verified"
        ]
    ):
        raise RehabCartesianContractError(
            "patient motion profile is "
            "not verified"
        )

    if not bool(
        execution[
            "enabled"
        ]
    ):
        raise RehabCartesianContractError(
            "Cartesian real execution "
            "gate remains closed"
        )

    return True
