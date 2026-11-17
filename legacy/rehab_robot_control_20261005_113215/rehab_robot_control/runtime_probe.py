from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from pathlib import Path
import importlib
import sys

PACKAGES = (
    "franka_bringup",
    "franka_hardware",
    "franka_fr3_moveit_config",
    "rehab_robot_control",
    "rehab_robot_bringup",
    "realsense2_camera",
)


def main():
    failed = False
    for package in PACKAGES:
        try:
            print(f"[PASS] {package} -> {get_package_prefix(package)}")
        except PackageNotFoundError:
            print(f"[FAIL] package not found: {package}")
            failed = True

    realtime = Path("/sys/kernel/realtime")
    rt_value = realtime.read_text().strip() if realtime.exists() else "0"
    print(f"[{'PASS' if rt_value == '1' else 'WARN'}] realtime={rt_value}")

    for module in ("xhand_controller", "xhand_controller.xhand_control"):
        try:
            loaded = importlib.import_module(module)
            print(f"[PASS] import {module} -> {getattr(loaded, '__file__', 'built-in')}")
        except Exception as exc:
            print(f"[WARN] import {module}: {exc}")

    if failed:
        sys.exit(1)
