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
