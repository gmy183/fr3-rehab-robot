from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory


JOINT_NAMES = tuple(
    f"fr3_joint{i}"
    for i in range(1, 8)
)


def parse_urdf(text: str):
    root = ET.fromstring(text)

    limits = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")

        if name not in JOINT_NAMES:
            continue

        element = joint.find("limit")

        if element is None:
            continue

        lower = element.attrib.get("lower")
        upper = element.attrib.get("upper")

        if lower is None or upper is None:
            continue

        limits[name] = {
            "lower": float(lower),
            "upper": float(upper),
        }

    return limits


share = Path(
    get_package_share_directory(
        "franka_description"
    )
)

print("[INFO] franka_description =", share)

candidates = sorted(
    set(
        list(share.rglob("*fr3*.urdf.xacro"))
        + list(share.rglob("fr3.urdf.xacro"))
    )
)

if not candidates:
    print("[FAIL] 找不到 FR3 URDF xacro")
    sys.exit(1)

print()
print("候选 FR3 xacro：")

for path in candidates:
    print(" ", path)

resolved = None
resolved_source = None
errors = []

for path in candidates:
    try:
        result = subprocess.run(
            [
                "xacro",
                str(path),
            ],
            text=True,
            capture_output=True,
            timeout=30,
        )
    except Exception as exc:
        errors.append(
            f"{path}: {exc}"
        )
        continue

    if result.returncode != 0:
        errors.append(
            f"{path}: xacro rc={result.returncode}: "
            f"{result.stderr.strip()[:300]}"
        )
        continue

    try:
        limits = parse_urdf(
            result.stdout
        )
    except Exception as exc:
        errors.append(
            f"{path}: URDF parse: {exc}"
        )
        continue

    if all(
        name in limits
        for name in JOINT_NAMES
    ):
        resolved = limits
        resolved_source = path
        break


# ------------------------------------------------------------
# Fallback:
# search already-generated URDF files in the installed package.
# ------------------------------------------------------------

if resolved is None:
    for path in sorted(
        share.rglob("*.urdf")
    ):
        try:
            limits = parse_urdf(
                path.read_text(
                    encoding="utf-8"
                )
            )
        except Exception:
            continue

        if all(
            name in limits
            for name in JOINT_NAMES
        ):
            resolved = limits
            resolved_source = path
            break


if resolved is None:
    print()
    print("[FAIL] 没有从官方 franka_description 解析出完整 7 关节限制")

    if errors:
        print()
        print("xacro 尝试信息：")

        for item in errors:
            print(" ", item)

    sys.exit(1)


margin = 0.05

output = {
    "source": {
        "package": "franka_description",
        "file": str(resolved_source),
        "generated_from_current_install": True,
    },
    "joint_count": 7,
    "safety_margin_rad": margin,
    "joints": {},
}

for name in JOINT_NAMES:
    lower = resolved[name]["lower"]
    upper = resolved[name]["upper"]

    safe_lower = lower + margin
    safe_upper = upper - margin

    if safe_lower >= safe_upper:
        raise RuntimeError(
            f"invalid safe limits for {name}"
        )

    output["joints"][name] = {
        "lower_rad": lower,
        "upper_rad": upper,
        "safe_lower_rad": safe_lower,
        "safe_upper_rad": safe_upper,
    }


target = (
    Path.home()
    / "rehab_robot_ws/config/"
      "fr3_joint_limits_resolved.yaml"
)

target.write_text(
    yaml.safe_dump(
        output,
        allow_unicode=True,
        sort_keys=False,
    ),
    encoding="utf-8",
)

print()
print("[PASS] 官方 FR3 关节限制已解析")
print("[PASS] source =", resolved_source)
print("[PASS] safety margin =", margin, "rad")

for name in JOINT_NAMES:
    item = output["joints"][name]

    print(
        f"{name}: "
        f"official=[{item['lower_rad']:.6f}, "
        f"{item['upper_rad']:.6f}]  "
        f"safe=[{item['safe_lower_rad']:.6f}, "
        f"{item['safe_upper_rad']:.6f}]"
    )

print()
print("[PASS] written:", target)
