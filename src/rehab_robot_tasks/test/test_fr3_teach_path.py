import math

from rehab_robot_tasks.fr3_teach_path import (
    build_route,
    edge_summaries,
    load_teach_path,
    polyline_length,
)


def test_existing_8point_path_contract():
    path = load_teach_path()

    assert path.version == 1
    assert path.name == "manual_8point_approach_v1"
    assert path.frame == "fr3_link0"
    assert path.tool == "fr3_link8"

    assert len(path.points) == 8


def test_original_points_are_preserved():
    path = load_teach_path()

    table = {
        p.point_id: p.position_m
        for p in path.points
    }

    assert table["P8"] == (
        0.259547,
        0.521461,
        0.302780,
    )

    assert table["P1"] == (
        0.361755,
        -0.060096,
        0.442920,
    )


def test_default_route_is_p8_to_p1():
    path = load_teach_path()

    route = build_route(
        path,
        "P8",
        "P1",
    )

    assert tuple(
        point.point_id
        for point in route
    ) == (
        "P8",
        "P7",
        "P6",
        "P5",
        "P4",
        "P3",
        "P2",
        "P1",
    )


def test_original_path_geometry():
    path = load_teach_path()
    route = build_route(path)

    assert math.isclose(
        polyline_length(route),
        0.9127742848,
        abs_tol=1e-9,
    )

    edges = edge_summaries(
        route,
        0.015,
    )

    assert tuple(
        item[3]
        for item in edges
    ) == (
        2,
        9,
        12,
        5,
        13,
        15,
        8,
    )

    assert sum(
        item[3]
        for item in edges
    ) == 64
