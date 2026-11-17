from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
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
    package_share: Path,
):
    """
    Franka ROS2 installs the semantic model under:

        franka_fr3_moveit_config/srdf/
            fr3_arm.srdf.xacro

    Do not assume SRDF lives in config/.
    """

    candidates = []

    for directory in (
        package_share / "srdf",
        package_share / "config",
    ):
        if not directory.exists():
            continue

        candidates.extend(
            directory.glob(
                "*.srdf"
            )
        )

        candidates.extend(
            directory.glob(
                "*.srdf.xacro"
            )
        )

    if not candidates:
        # Last-resort recursive discovery to tolerate
        # minor package layout changes.
        candidates.extend(
            package_share.rglob(
                "*.srdf"
            )
        )

        candidates.extend(
            package_share.rglob(
                "*.srdf.xacro"
            )
        )

    candidates = list(
        dict.fromkeys(
            Path(p)
            for p in candidates
        )
    )

    if not candidates:
        raise RehabMoveItTopologyError(
            "FR3 SRDF/SRDF.xacro file "
            "not found in MoveIt package"
        )

    # Prefer the official FR3 semantic file.
    candidates.sort(
        key=lambda p: (
            p.name != "fr3_arm.srdf.xacro",
            "fr3" not in p.name.lower(),
            "srdf" not in p.parts,
            str(p),
        )
    )

    return candidates[0]


def _render_srdf(
    srdf_path: Path,
):
    """
    Return expanded SRDF XML.

    Raw *.srdf is read directly.
    *.srdf.xacro is expanded using the installed xacro tool.

    We explicitly disable the Franka gripper because this
    workstation uses XHand instead of the Franka Hand.
    """

    if (
        srdf_path.name.endswith(
            ".xacro"
        )
    ):
        cmd = [
            "xacro",
            str(srdf_path),
            "hand:=false",
        ]

        try:
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=True,
                text=True,
            )

        except FileNotFoundError as exc:
            raise RehabMoveItTopologyError(
                "xacro executable not found"
            ) from exc

        except subprocess.CalledProcessError as exc:
            stderr = (
                exc.stderr
                or ""
            ).strip()

            raise RehabMoveItTopologyError(
                "failed to expand FR3 SRDF xacro"
                + (
                    f": {stderr}"
                    if stderr
                    else ""
                )
            ) from exc

        xml_text = result.stdout

    else:
        xml_text = (
            srdf_path.read_text(
                encoding="utf-8"
            )
        )

    if not xml_text.strip():
        raise RehabMoveItTopologyError(
            "expanded FR3 SRDF is empty"
        )

    return xml_text


def _discover_group_from_srdf(
    srdf_path: Path,
):
    xml_text = _render_srdf(
        srdf_path
    )

    try:
        root = ET.fromstring(
            xml_text
        )

    except ET.ParseError as exc:
        raise RehabMoveItTopologyError(
            "expanded FR3 SRDF XML "
            "could not be parsed"
        ) from exc

    groups = list(
        root.iter(
            "group"
        )
    )

    if not groups:
        raise RehabMoveItTopologyError(
            "no MoveIt planning groups "
            "found in expanded SRDF"
        )

    # Best match:
    # explicit FR3 arm kinematic chain.
    for group in groups:

        name = group.attrib.get(
            "name",
            "",
        )

        for chain in group.findall(
            "chain"
        ):
            base = chain.attrib.get(
                "base_link",
                "",
            )

            tip = chain.attrib.get(
                "tip_link",
                "",
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

    # Fallback:
    # official group normally identifies itself as fr3 arm.
    for group in groups:

        name = group.attrib.get(
            "name",
            "",
        )

        if (
            "fr3" not in name.lower()
            or "arm" not in name.lower()
        ):
            continue

        chain = group.find(
            "chain"
        )

        if chain is None:
            continue

        base = chain.attrib.get(
            "base_link",
            "",
        )

        tip = chain.attrib.get(
            "tip_link",
            "",
        )

        if base and tip:
            return (
                name,
                base,
                tip,
            )

    raise RehabMoveItTopologyError(
        "unable to identify FR3 arm "
        "planning group in SRDF"
    )


def _discover_controller(
    controller_path: Path,
):
    data = yaml.safe_load(
        controller_path.read_text(
            encoding="utf-8"
        )
    )

    if not isinstance(
        data,
        dict,
    ):
        raise RehabMoveItTopologyError(
            "invalid MoveIt controller YAML"
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
            action_ns = str(
                item.get(
                    "action_ns",
                    "follow_joint_trajectory",
                )
            )

            return (
                str(name),
                action_ns,
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

    if not share.exists():
        raise RehabMoveItTopologyError(
            "FR3 MoveIt package share "
            "directory does not exist"
        )

    srdf = _find_srdf_file(
        share
    )

    controller = (
        share
        / "config"
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
