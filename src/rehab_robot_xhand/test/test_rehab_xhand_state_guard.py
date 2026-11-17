from types import SimpleNamespace

import pytest

import rehab_robot_xhand.rehab_xhand_state_guard as mod


REFERENCE = (
    0.060,
    1.060,
    0.520,
    0.010,
    0.020,
    0.010,
    0.020,
    0.000,
    0.010,
    0.000,
    0.020,
    0.000,
)


ZERO_SAMPLE = (
    (0.0,) * 12,
    (0.0,) * 12,
    (0,) * 12,
)


GOOD_SAMPLE = (
    REFERENCE,
    (70.0,) * 12,
    (3,) * 12,
)


ACTIVE = (
    "thumb_bend_joint",
    "index_joint1",
    "index_joint2",
    "mid_joint1",
    "mid_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
)


def test_complete_zero_frame_is_detected():
    assert mod.is_zero_collapse_frame(
        ZERO_SAMPLE,
        reference=REFERENCE,
    )


def test_real_nonzero_sample_is_not_zero_collapse():
    assert not mod.is_zero_collapse_frame(
        GOOD_SAMPLE,
        reference=REFERENCE,
    )


def test_one_zero_frame_then_good_is_recovered(
    monkeypatch,
):
    samples = iter(
        [
            ZERO_SAMPLE,
            GOOD_SAMPLE,
        ]
    )

    monkeypatch.setattr(
        mod,
        "read_xhand_sample",
        lambda driver: next(samples),
    )

    monkeypatch.setattr(
        mod.time,
        "sleep",
        lambda value: None,
    )

    node = SimpleNamespace()

    sample = mod.read_guarded_xhand_sample(
        node,
        object(),
        reference=REFERENCE,
    )

    assert sample == GOOD_SAMPLE
    assert node.xhand_zero_frame_total == 1
    assert node.xhand_zero_frame_consecutive == 0
    assert node.last_good_xhand_sample == GOOD_SAMPLE


def test_three_consecutive_zero_frames_hard_fault(
    monkeypatch,
):
    monkeypatch.setattr(
        mod,
        "read_xhand_sample",
        lambda driver: ZERO_SAMPLE,
    )

    monkeypatch.setattr(
        mod.time,
        "sleep",
        lambda value: None,
    )

    node = SimpleNamespace()

    with pytest.raises(
        mod.RuntimeSafetyError,
        match="consecutive zero-state",
    ):
        mod.read_guarded_xhand_sample(
            node,
            object(),
            reference=REFERENCE,
        )

    assert node.xhand_zero_frame_total == 3
    assert node.xhand_zero_frame_consecutive == 3


def test_last_good_is_not_used_to_mask_repeated_failure(
    monkeypatch,
):
    node = SimpleNamespace(
        last_good_xhand_sample=GOOD_SAMPLE,
    )

    monkeypatch.setattr(
        mod,
        "read_xhand_sample",
        lambda driver: ZERO_SAMPLE,
    )

    monkeypatch.setattr(
        mod.time,
        "sleep",
        lambda value: None,
    )

    with pytest.raises(
        mod.RuntimeSafetyError
    ):
        mod.read_guarded_xhand_sample(
            node,
            object(),
            reference=REFERENCE,
        )


def test_active_mode3_is_required():
    mod.require_active_modes(
        GOOD_SAMPLE,
        active_joints=ACTIVE,
    )


def test_real_mode_drop_is_not_hidden():
    modes = list(
        GOOD_SAMPLE[2]
    )

    modes[0] = 0

    dropped = (
        GOOD_SAMPLE[0],
        GOOD_SAMPLE[1],
        tuple(modes),
    )

    with pytest.raises(
        mod.RuntimeSafetyError,
        match="active mode dropped",
    ):
        mod.require_active_modes(
            dropped,
            active_joints=ACTIVE,
        )
