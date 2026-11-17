import math

import pytest

from rehab_robot_control.rehab_cartesian_contract import (
    RehabCartesianContractError,
    RelativeCartesianMove,
    load_cartesian_config,
    require_real_cartesian_gate,
    rotation_norm,
    translation_norm,
    validate_relative_move,
)


def test_small_translation_valid():
    move = RelativeCartesianMove(
        dx=0.005
    )

    assert (
        validate_relative_move(move)
        == move
    )


def test_translation_norm():
    move = RelativeCartesianMove(
        dx=0.003,
        dy=0.004,
    )

    assert abs(
        translation_norm(move)
        - 0.005
    ) < 1e-12


def test_rotation_norm():
    move = RelativeCartesianMove(
        dpitch=math.radians(5)
    )

    assert abs(
        rotation_norm(move)
        - math.radians(5)
    ) < 1e-12


def test_large_translation_rejected():
    move = RelativeCartesianMove(
        dx=0.021
    )

    with pytest.raises(
        RehabCartesianContractError
    ):
        validate_relative_move(move)


def test_large_rotation_rejected():
    move = RelativeCartesianMove(
        dyaw=math.radians(11)
    )

    with pytest.raises(
        RehabCartesianContractError
    ):
        validate_relative_move(move)


def test_real_gate_closed():
    config = load_cartesian_config()

    with pytest.raises(
        RehabCartesianContractError
    ):
        require_real_cartesian_gate(
            config=config
        )
