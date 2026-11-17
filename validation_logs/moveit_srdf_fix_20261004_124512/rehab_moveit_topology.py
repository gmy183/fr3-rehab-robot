from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)


class RehabMoveItTopologyError(
    RuntimeError
):
    pass


@dataclass(frozen=True)
class MoveItTopology:
    package_share: Path
    srdf_path: Path
    controller_path: Path

    planning_group: str
    base_link: str
    tip_link: str

    trajectory_controller: str
    trajectory_action_ns: str


def _find_srdf_file(
    config_dir,
):
    candidates = list(
        config_dir.glob(
            "*.srdf"
        )
    )

    candidates += list(
        config_dir.glob(
            "*.srdf.xacro"
        )
    )

    if not candidates:
        raise RehabMoveItTopologyError(
            "FR3 SRDF file not found"
        )

    # Prefer FR3-specific filename.
    candidates.sort(
        key=lambda p: (
            "fr3" not in p.name.lower(),
            p.name,
        )
    )

    return candidates[0]


def _discover_group_from_srdf(
    srdf_path,
):
    root = ET.parse(
        srdf_path
    ).getroot()

    groups = root.findall(
        "group"
    )

    if not groups:
        raise RehabMoveItTopologyError(
            "no MoveIt planning groups "
            "found in SRDF"
        )

    # Prefer a chain ending at fr3_link8.
    for group in groups:

        name = group.attrib.get(
            "name",
            "",
        )

        for chain in group.findall(
            "chain"
        ):
            base = chain.attrib.get(
                "base_link"
            )

            tip = chain.attrib.get(
                "tip_link"
            )

            if (
                base == "fr3_link0"
                and tip == "fr3_link8"
            ):
                return (
                    name,
                    base,
                    tip,
                )

    # Then prefer group name containing fr3 + arm.
    for group in groups:

        name = group.attrib.get(
            "name",
            ""
        )

        if (
            "fr3" in name.lower()
            and "arm" in name.lower()
        ):
            chain = group.find(
                "chain"
            )

            if chain is not None:
                return (
                    name,
                    chain.attrib.get(
                        "base_link"
                    ),
                    chain.attrib.get(
                        "tip_link"
                    ),
                )

    raise RehabMoveItTopologyError(
        "unable to identify FR3 arm "
        "planning group"
    )


def _discover_controller(
    controller_path,
):
    data = yaml.safe_load(
        controller_path.read_text(
            encoding="utf-8"
        )
    )

    names = data.get(
        "controller_names",
        []
    )

    for name in names:

        item = data.get(
            name,
            {}
        )

        ctype = str(
            item.get(
                "type",
                ""
            )
        )

        if (
            "FollowJointTrajectory"
            in ctype
        ):
            return (
                name,
                str(
                    item.get(
                        "action_ns",
                        "follow_joint_trajectory",
                    )
                ),
            )

    raise RehabMoveItTopologyError(
        "FollowJointTrajectory "
        "controller not found"
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

    config_dir = (
        share
        / "config"
    )

    srdf = _find_srdf_file(
        config_dir
    )

    controller = (
        config_dir
        / "fr3_controllers.yaml"
    )

    if not controller.exists():
        raise RehabMoveItTopologyError(
            "fr3_controllers.yaml not found"
        )

    (
        group,
        base,
        tip,
    ) = _discover_group_from_srdf(
        srdf
    )

    (
        controller_name,
        action_ns,
    ) = _discover_controller(
        controller
    )

    return MoveItTopology(
        package_share=share,
        srdf_path=srdf,
        controller_path=controller,
        planning_group=group,
        base_link=base,
        tip_link=tip,
        trajectory_controller=(
            controller_name
        ),
        trajectory_action_ns=(
            action_ns
        ),
    )
