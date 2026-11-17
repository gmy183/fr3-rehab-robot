import ast
from pathlib import Path

import pytest

from rehab_robot_xhand.xhand_ros2_node import (
    validate_source_configuration,
)
from rehab_robot_xhand.xhand_readonly_session import (
    VendorXHandReadOnlyBackend,
)


def test_disabled_source_is_safe_default():
    validate_source_configuration(
        False,
        "disabled",
    )


def test_fake_source_needs_no_hardware_permission():
    validate_source_configuration(
        False,
        "fake",
    )


def test_vendor_readonly_requires_explicit_hardware_permission():
    with pytest.raises(
        RuntimeError,
        match="requires hardware_enable=true",
    ):
        validate_source_configuration(
            False,
            "vendor_readonly",
        )


def test_vendor_readonly_can_be_explicitly_authorized():
    validate_source_configuration(
        True,
        "vendor_readonly",
    )


def test_hardware_permission_cannot_be_used_with_fake():
    with pytest.raises(
        RuntimeError,
        match="only valid with state_source=vendor_readonly",
    ):
        validate_source_configuration(
            True,
            "fake",
        )


def test_ros_node_contains_no_send_command_call():
    path = (
        Path.home()
        / "rehab_robot_ws/src/rehab_robot_xhand/"
          "rehab_robot_xhand/xhand_ros2_node.py"
    )

    tree = ast.parse(
        path.read_text(encoding="utf-8")
    )

    calls = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        func = node.func

        if (
            isinstance(func, ast.Attribute)
            and func.attr == "send_command"
        ):
            calls.append(node.lineno)

    assert calls == []


def test_vendor_readonly_backend_has_no_command_api():
    assert not hasattr(
        VendorXHandReadOnlyBackend,
        "send_command",
    )
