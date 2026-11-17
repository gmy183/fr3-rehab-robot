from types import SimpleNamespace

import rehab_robot_control.rehab_microcycle_ptp as mod


MICRO = {
    "refresh_hz": 20.0,
    "watchdog_s": 0.5,
}


class FakeDriver:

    def __init__(self):
        self.refresh_count = 0
        self.watchdog_count = 0

    def refresh_active_command(
        self,
        command,
    ):
        self.refresh_count += 1

    def poll_watchdog(self):
        self.watchdog_count += 1
        return False


def test_intermediate_ptp_poll_does_not_read_xhand(
    monkeypatch,
):
    """
    Fast FR3 polling iterations between 20 Hz XHand cycles
    must not trigger an XHand state read.
    """

    node = SimpleNamespace(
        last_xhand_refresh_time=100.0,
    )

    driver = FakeDriver()

    reads = []

    monkeypatch.setattr(
        mod,
        "refresh_fr3_state",
        lambda *a, **k: None,
    )

    monkeypatch.setattr(
        mod,
        "assert_fr3_fresh",
        lambda *a, **k: None,
    )

    monkeypatch.setattr(
        mod.time,
        "monotonic",
        lambda: 100.010,
    )

    monkeypatch.setattr(
        mod,
        "read_guarded_xhand_sample",
        lambda *a, **k: (
            reads.append(True)
        ),
    )

    sent = mod.maintain_xhand_validated_once(
        node,
        driver,
        command=object(),
        reference=(0.1,) * 12,
        active_joints=(),
        micro=MICRO,
    )

    assert sent is False
    assert driver.refresh_count == 0
    assert reads == []


def test_due_cycle_sends_then_guarded_reads_once(
    monkeypatch,
):
    """
    When the 20 Hz cycle is due:
        send active command once
        -> guarded XHand read once
        -> safety check once
        -> active-mode check once
    """

    node = SimpleNamespace(
        last_xhand_refresh_time=100.0,
    )

    driver = FakeDriver()

    reads = []
    checks = []
    mode_checks = []

    sample = (
        (0.1,) * 12,
        (70.0,) * 12,
        (3,) * 12,
    )

    monkeypatch.setattr(
        mod,
        "refresh_fr3_state",
        lambda *a, **k: None,
    )

    monkeypatch.setattr(
        mod,
        "assert_fr3_fresh",
        lambda *a, **k: None,
    )

    monkeypatch.setattr(
        mod.time,
        "monotonic",
        lambda: 100.060,
    )

    def fake_guarded_read(
        *args,
        **kwargs,
    ):
        reads.append(True)
        return sample

    monkeypatch.setattr(
        mod,
        "read_guarded_xhand_sample",
        fake_guarded_read,
    )

    monkeypatch.setattr(
        mod,
        "check_xhand_sample",
        lambda value, **kwargs: (
            checks.append(value)
        ),
    )

    monkeypatch.setattr(
        mod,
        "require_active_modes",
        lambda value, **kwargs: (
            mode_checks.append(value)
        ),
    )

    sent = mod.maintain_xhand_validated_once(
        node,
        driver,
        command=object(),
        reference=(0.1,) * 12,
        active_joints=(),
        micro=MICRO,
    )

    assert sent is True
    assert driver.refresh_count == 1
    assert len(reads) == 1
    assert checks == [sample]
    assert mode_checks == [sample]


def test_large_refresh_gap_is_rejected_before_send(
    monkeypatch,
):
    """
    A scheduling stall must fault before another XHand
    active command is transmitted.
    """

    node = SimpleNamespace(
        last_xhand_refresh_time=100.0,
    )

    driver = FakeDriver()

    monkeypatch.setattr(
        mod,
        "refresh_fr3_state",
        lambda *a, **k: None,
    )

    monkeypatch.setattr(
        mod.time,
        "monotonic",
        lambda: 100.300,
    )

    try:

        mod.maintain_xhand_validated_once(
            node,
            driver,
            command=object(),
            reference=(0.1,) * 12,
            active_joints=(),
            micro=MICRO,
        )

    except mod.RuntimeSafetyError as exc:

        assert "scheduling gap" in str(exc)

    else:

        raise AssertionError(
            "large scheduling gap was not rejected"
        )

    assert driver.refresh_count == 0
