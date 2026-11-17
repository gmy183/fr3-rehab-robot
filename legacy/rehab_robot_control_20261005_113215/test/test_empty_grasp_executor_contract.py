from pathlib import Path

import yaml

from rehab_robot_control.empty_grasp_executor import (
    load_configs,
    validate_micro_gate,
)


WS = Path.home() / "rehab_robot_ws"


def test_current_micro_gate_accepts():
    profile, sequence, micro = (
        load_configs()
    )

    active = validate_micro_gate(
        profile,
        sequence,
        micro,
    )

    assert "index_joint1" in active
    assert len(active) == 9


def test_formal_verified_remains_false():
    profile = yaml.safe_load(
        (
            WS
            / "config/xhand_motion_profile.yaml"
        ).read_text()
    )["xhand"]

    assert (
        profile["active_gains"]["verified"]
        is False
    )


def test_micro_current_is_150_or_less():
    micro = yaml.safe_load(
        (
            WS
            / "config/xhand_micro_validation.yaml"
        ).read_text()
    )["micro_validation"]

    assert int(
        micro["effort_limit_ma"]
    ) <= 150


def test_micro_delta_is_0p005_or_less():
    micro = yaml.safe_load(
        (
            WS
            / "config/xhand_micro_validation.yaml"
        ).read_text()
    )["micro_validation"]

    assert float(
        micro["delta_rad"]
    ) <= 0.005


def test_whole_hand_not_claimed_precise():
    micro = yaml.safe_load(
        (
            WS
            / "config/xhand_micro_validation.yaml"
        ).read_text()
    )["micro_validation"]

    assert (
        micro["whole_hand"][
            "precise_direction_tracking_verified"
        ]
        is False
    )
