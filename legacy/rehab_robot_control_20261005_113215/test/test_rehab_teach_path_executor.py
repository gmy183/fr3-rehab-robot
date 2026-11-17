import math

import pytest

from rehab_robot_control.rehab_teach_path_executor import (
    TeachPathError,
    distance,
    ordered_route_ids,
    subdivide_line,
)


def test_route_forward():
    ids = [
        "P1",
        "P2",
        "P3",
        "P4",
    ]

    assert ordered_route_ids(
        ids,
        "P1",
        "P4",
    ) == [
        "P1",
        "P2",
        "P3",
        "P4",
    ]


def test_route_reverse():
    ids = [
        "P1",
        "P2",
        "P3",
        "P4",
    ]

    assert ordered_route_ids(
        ids,
        "P4",
        "P2",
    ) == [
        "P4",
        "P3",
        "P2",
    ]


def test_same_point_rejected():
    with pytest.raises(
        TeachPathError
    ):
        ordered_route_ids(
            ["P1", "P2"],
            "P1",
            "P1",
        )


def test_subdivision_limit():
    length, targets = (
        subdivide_line(
            (0.0, 0.0, 0.0),
            (0.1238, 0.0, 0.0),
            0.015,
        )
    )

    assert math.isclose(
        length,
        0.1238,
        rel_tol=0.0,
        abs_tol=1e-12,
    )

    assert len(targets) == 9

    last = (
        0.0,
        0.0,
        0.0,
    )

    for target in targets:
        assert (
            distance(
                last,
                target,
            )
            <= 0.015 + 1e-12
        )

        last = target


def test_route_reverse_to_first_point():
    ids = [
        "P1",
        "P2",
        "P3",
        "P4",
        "P5",
        "P6",
    ]

    assert ordered_route_ids(
        ids,
        "P6",
        "P1",
    ) == [
        "P6",
        "P5",
        "P4",
        "P3",
        "P2",
        "P1",
    ]


def test_route_full_reverse_p8_to_p1():
    ids = [
        "P1",
        "P2",
        "P3",
        "P4",
        "P5",
        "P6",
        "P7",
        "P8",
    ]

    assert ordered_route_ids(
        ids,
        "P8",
        "P1",
    ) == [
        "P8",
        "P7",
        "P6",
        "P5",
        "P4",
        "P3",
        "P2",
        "P1",
    ]
