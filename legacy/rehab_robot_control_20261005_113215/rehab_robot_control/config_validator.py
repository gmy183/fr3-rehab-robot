import argparse
from pathlib import Path
import sys
import yaml

REQUIRED_LIMITS = {
    "fr3.max_joint_velocity_rad_s": (0.0, 0.020),
    "fr3.rehab_swing_rad": (0.0, 0.010),
    "xhand.micro_close_rad": (0.0, 0.005),
    "xhand.single_joint_current_ma": (0.0, 150.0),
    "xhand.whole_hand_current_ma": (0.0, 220.0),
    "xhand.watchdog_s": (0.0, 0.5),
}


def nested_get(data, dotted):
    value = data
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            raise KeyError(dotted)
        value = value[key]
    return value


def main():
    default_path = Path.home() / "rehab_robot_ws/config/system.yaml"
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=default_path)
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    errors = []
    for key, (lower, upper) in REQUIRED_LIMITS.items():
        try:
            value = float(nested_get(data, key))
        except (KeyError, TypeError, ValueError):
            errors.append(f"{key}: missing or invalid")
            continue
        if not lower < value <= upper:
            errors.append(f"{key}: {value} outside ({lower}, {upper}]")

    excluded = nested_get(data, "xhand.excluded_joints")
    if "index_bend_joint" not in excluded:
        errors.append("xhand.excluded_joints must include index_bend_joint")

    if nested_get(data, "fr3.automatic_recovery_on_fault") is not False:
        errors.append("fr3.automatic_recovery_on_fault must be false")

    if errors:
        for error in errors:
            print(f"[FAIL] {error}")
        sys.exit(1)

    print(f"[PASS] configuration validated: {args.config}")
