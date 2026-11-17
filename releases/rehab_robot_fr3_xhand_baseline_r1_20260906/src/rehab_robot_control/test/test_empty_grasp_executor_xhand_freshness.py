import inspect

from rehab_robot_control import (
    empty_grasp_executor as executor,
)


def test_run_xhand_window_checks_fr3_before_active_refresh():
    src = inspect.getsource(
        executor.run_xhand_window
    )

    refresh = src.index(
        "refresh_fr3_state("
    )

    send = src.index(
        "driver.refresh_active_command("
    )

    assert refresh < send


def test_maintain_xhand_checks_fr3_before_active_refresh():
    src = inspect.getsource(
        executor.maintain_xhand_once
    )

    refresh = src.index(
        "refresh_fr3_state("
    )

    send = src.index(
        "driver.refresh_active_command("
    )

    assert refresh < send
