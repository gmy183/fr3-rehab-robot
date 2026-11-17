"""Eight-point FR3 rehabilitation teach-path contract."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math

import yaml


DEFAULT_PATH = (
    Path.home()
    / "rehab_robot_ws"
    / "teach"
    / "teach_path_8pts_v1.yaml"
)

DEFAULT_START = "P8"
DEFAULT_END = "P1"
DEFAULT_MAX_STEP_M = 0.015


class TeachPathError(RuntimeError):
    pass


@dataclass(frozen=True)
class TeachPoint:
    point_id: str
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class TeachPath:
    version: int
    name: str
    frame: str
    tool: str
    source: str
    orientation_mode: str
    validated: bool
    full_path_enabled: bool
    points: tuple[TeachPoint, ...]


def load_teach_path(
    path: Path | None = None,
) -> TeachPath:
    if path is None:
        path = DEFAULT_PATH

    data = yaml.safe_load(
        Path(path).read_text(
            encoding="utf-8",
        )
    )

    root = data["teach_path"]

    points = []

    for item in root["points"]:
        point_id = str(item["id"])

        xyz = tuple(
            float(value)
            for value in item["position_m"]
        )

        if len(xyz) != 3:
            raise TeachPathError(
                f"{point_id}: position must have 3 values"
            )

        if not all(
            math.isfinite(value)
            for value in xyz
        ):
            raise TeachPathError(
                f"{point_id}: non-finite position"
            )

        points.append(
            TeachPoint(
                point_id=point_id,
                position_m=xyz,
            )
        )

    ids = tuple(
        point.point_id
        for point in points
    )

    if ids != (
        "P1",
        "P2",
        "P3",
        "P4",
        "P5",
        "P6",
        "P7",
        "P8",
    ):
        raise TeachPathError(
            f"unexpected point IDs/order: {ids}"
        )

    if root["frame"] != "fr3_link0":
        raise TeachPathError(
            "teach path frame must be fr3_link0"
        )

    if root["tool"] != "fr3_link8":
        raise TeachPathError(
            "teach path tool must be fr3_link8"
        )

    if (
        root["orientation_mode"]
        != "hold_current_orientation"
    ):
        raise TeachPathError(
            "unsupported orientation mode"
        )

    execution = root.get(
        "execution",
        {},
    )

    return TeachPath(
        version=int(root["version"]),
        name=str(root["name"]),
        frame=str(root["frame"]),
        tool=str(root["tool"]),
        source=str(root["source"]),
        orientation_mode=str(
            root["orientation_mode"]
        ),
        validated=bool(
            execution.get(
                "validated",
                False,
            )
        ),
        full_path_enabled=bool(
            execution.get(
                "full_path_enabled",
                False,
            )
        ),
        points=tuple(points),
    )


def build_route(
    path: TeachPath,
    start_id: str = DEFAULT_START,
    end_id: str = DEFAULT_END,
) -> tuple[TeachPoint, ...]:
    ids = [
        point.point_id
        for point in path.points
    ]

    if start_id not in ids:
        raise TeachPathError(
            f"unknown start point: {start_id}"
        )

    if end_id not in ids:
        raise TeachPathError(
            f"unknown end point: {end_id}"
        )

    start_index = ids.index(start_id)
    end_index = ids.index(end_id)

    if start_index >= end_index:
        route = path.points[
            end_index:start_index + 1
        ][::-1]
    else:
        route = path.points[
            start_index:end_index + 1
        ]

    return tuple(route)


def edge_summaries(
    route: tuple[TeachPoint, ...],
    max_step_m: float = DEFAULT_MAX_STEP_M,
):
    if max_step_m <= 0.0:
        raise TeachPathError(
            "max_step_m must be positive"
        )

    result = []

    for first, second in zip(
        route[:-1],
        route[1:],
    ):
        distance = math.dist(
            first.position_m,
            second.position_m,
        )

        segments = max(
            1,
            math.ceil(
                distance / max_step_m
            ),
        )

        result.append(
            (
                first.point_id,
                second.point_id,
                distance,
                segments,
            )
        )

    return tuple(result)


def polyline_length(
    route: tuple[TeachPoint, ...],
) -> float:
    return sum(
        math.dist(
            first.position_m,
            second.position_m,
        )
        for first, second in zip(
            route[:-1],
            route[1:],
        )
    )
