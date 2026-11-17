from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)


class RehabMoveItTopologyError(
    RuntimeError
):
    pass


# Verified on the actual FR3 TF tree used by this workstation.
FR3_BASE_LINK = "fr3_link0"
FR3_FLANGE_LINK = "fr3_link8"


@dataclass(frozen=True)
class MoveItTopology:
    package_share: Path
    kinematics_path: Path
    controller_path: Path

    planning_group: str

    base_link: str
    tip_link: str

    trajectory_controller: str
    trajectory_action_ns: str

    # Deliberately None on this workstation.
    # The installed MoveIt package does not expose an SRDF file
    # and our rehabilitation layer does not need to parse one.
    srdf_path: Path | None = None


def _load_yaml(
    path: Path,
):
    if not path.exists():
        raise RehabMoveItTopologyError(
            f"required MoveIt file not found: {path}"
        )

    data = yaml.safe_load(
        path.read_text(
            encoding="utf-8"
        )
    )

    if not isinstance(
        data,
        dict,
    ):
        raise RehabMoveItTopologyError(
            f"invalid YAML mapping: {path}"
        )

    return data


def _discover_planning_group(
    kinematics_path: Path,
):
    """
    Discover the MoveIt planning group from the installed
    kinematics.yaml.

    This avoids making any assumption about where an SRDF
    happens to live.
    """

    data = _load_yaml(
        kinematics_path
    )

    candidates = []

    for name, item in data.items():

        if not isinstance(
            item,
            dict,
        ):
            continue

        # Normal MoveIt kinematics.yaml entries contain a
        # kinematics solver definition.
        if (
            "kinematics_solver"
            in item
        ):
            candidates.append(
                str(name)
            )

    # Some generated MoveIt configs may omit an explicit solver
    # field but still expose one mapping per planning group.
    if not candidates:

        for name, item in data.items():

            if isinstance(
                item,
                dict,
            ):
                candidates.append(
                    str(name)
                )

    candidates = list(
        dict.fromkeys(
            candidates
        )
    )

    if not candidates:
        raise RehabMoveItTopologyError(
            "no planning group found "
            "in kinematics.yaml"
        )

    # Prefer the conventional FR3 arm group.
    for preferred in (
        "fr3_arm",
        "fr3_manipulator",
        "arm",
    ):
        if preferred in candidates:
            return preferred

    for name in candidates:

        lower = name.lower()

        if (
            "fr3" in lower
            and "arm" in lower
        ):
            return name

    for name in candidates:

        if "arm" in name.lower():
            return name

    if len(candidates) == 1:
        return candidates[0]

    raise RehabMoveItTopologyError(
        "ambiguous MoveIt planning groups: "
        + ", ".join(candidates)
    )


def _discover_controller(
    controller_path: Path,
):
    data = _load_yaml(
        controller_path
    )

    names = data.get(
        "controller_names",
        []
    )

    if not isinstance(
        names,
        list,
    ):
        raise RehabMoveItTopologyError(
            "controller_names is not a list"
        )

    matches = []

    for name in names:

        item = data.get(
            name,
            {}
        )

        if not isinstance(
            item,
            dict,
        ):
            continue

        ctype = str(
            item.get(
                "type",
                ""
            )
        )

        if (
            "FollowJointTrajectory"
            not in ctype
        ):
            continue

        matches.append(
            (
                str(name),
                str(
                    item.get(
                        "action_ns",
                        "follow_joint_trajectory",
                    )
                ),
            )
        )

    if not matches:
        raise RehabMoveItTopologyError(
            "FollowJointTrajectory "
            "controller not found"
        )

    # Prefer official FR3 arm controller.
    for name, action_ns in matches:

        if name == "fr3_arm_controller":
            return (
                name,
                action_ns,
            )

    if len(matches) == 1:
        return matches[0]

    raise RehabMoveItTopologyError(
        "multiple trajectory controllers found: "
        + ", ".join(
            name
            for name, _ in matches
        )
    )


def discover_fr3_moveit_topology(
    package_name=(
        "franka_fr3_moveit_config"
    ),
):
    share = Path(
        get_package_share_directory(
            package_name
        )
    )

    if not share.exists():
        raise RehabMoveItTopologyError(
            "FR3 MoveIt package share "
            "directory does not exist"
        )

    config_dir = (
        share
        / "config"
    )

    kinematics = (
        config_dir
        / "kinematics.yaml"
    )

    controllers = (
        config_dir
        / "fr3_controllers.yaml"
    )

    planning_group = (
        _discover_planning_group(
            kinematics
        )
    )

    (
        controller_name,
        action_ns,
    ) = _discover_controller(
        controllers
    )

    return MoveItTopology(
        package_share=share,
        kinematics_path=kinematics,
        controller_path=controllers,

        planning_group=planning_group,

        base_link=FR3_BASE_LINK,
        tip_link=FR3_FLANGE_LINK,

        trajectory_controller=(
            controller_name
        ),

        trajectory_action_ns=(
            action_ns
        ),

        srdf_path=None,
    )
