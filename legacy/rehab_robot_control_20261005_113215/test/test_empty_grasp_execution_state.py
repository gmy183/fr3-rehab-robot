import pytest

from rehab_robot_control.empty_grasp_execution_state import (
    ExecutionStateError,
    STAGES,
    STATUS_COMPLETE,
    STATUS_FAULTED,
    create_new_state,
    expected_pose_after_completed,
    load_state,
    mark_fault,
    mark_stage_completed,
    mark_stage_started,
    next_stage,
    require_resume_pose,
    save_state,
)


Q0 = (
    0.1,
    -0.6,
    0.0,
    -2.5,
    0.0,
    1.5,
    1.5,
)


def test_new_run_freezes_absolute_targets():
    state = create_new_state(
        q_start=Q0,
        run_id="test",
    )

    anchor = state[
        "fr3_anchor"
    ]

    assert (
        anchor["q_start"][0]
        == pytest.approx(0.1)
    )

    assert (
        anchor["q1_pregrasp"][0]
        == pytest.approx(0.105)
    )

    assert (
        anchor["q2_grasp"][0]
        == pytest.approx(0.115)
    )

    assert (
        anchor["q3_rehab"][0]
        == pytest.approx(0.120)
    )


def test_stage_order_cannot_skip():
    state = create_new_state(
        q_start=Q0
    )

    with pytest.raises(
        ExecutionStateError
    ):
        mark_stage_started(
            state,
            "fr3_relative_pregrasp",
        )


def test_stage_completion_advances_only_once():
    state = create_new_state(
        q_start=Q0
    )

    first = STAGES[0]

    state = mark_stage_started(
        state,
        first,
    )

    state = mark_stage_completed(
        state,
        first,
    )

    assert state[
        "completed_stages"
    ] == [first]

    assert next_stage(
        state
    ) == STAGES[1]


def test_fault_preserves_original_fr3_anchor():
    state = create_new_state(
        q_start=Q0,
        run_id="fixed-run",
    )

    original = state[
        "fr3_anchor"
    ].copy()

    state = mark_stage_started(
        state,
        STAGES[0],
    )

    state = mark_fault(
        state,
        reason="test fault",
    )

    assert (
        state["status"]
        == STATUS_FAULTED
    )

    assert (
        state["fr3_anchor"]
        == original
    )


def test_fault_inside_active_stage_blocks_auto_resume():
    state = create_new_state(
        q_start=Q0
    )

    state = mark_stage_started(
        state,
        STAGES[0],
    )

    state = mark_fault(
        state,
        reason="mid-stage fault",
    )

    with pytest.raises(
        ExecutionStateError
    ):
        require_resume_pose(
            state,
            Q0,
        )


def test_checkpoint_pose_comes_from_original_run_anchor():
    state = create_new_state(
        q_start=Q0
    )

    # 完成到 FR3 pregrasp。
    for stage in STAGES[:4]:
        state = mark_stage_started(
            state,
            stage,
        )

        state = mark_stage_completed(
            state,
            stage,
        )

    expected = (
        expected_pose_after_completed(
            state
        )
    )

    assert expected[0] == pytest.approx(
        0.105
    )


def test_atomic_save_and_load(tmp_path):
    state = create_new_state(
        q_start=Q0,
        run_id="persist",
    )

    path = (
        tmp_path
        / "execution_state.json"
    )

    save_state(
        state,
        path,
    )

    restored = load_state(
        path
    )

    assert (
        restored["run_id"]
        == "persist"
    )

    assert (
        restored["fr3_anchor"]
        == state["fr3_anchor"]
    )
