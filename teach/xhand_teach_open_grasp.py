import time
import statistics
import yaml
from pathlib import Path

from rehab_robot_xhand.xhand_readonly_session import (
    VendorXHandReadOnlyBackend,
    XHandReadOnlySession,
)

IFACE = "enx6c1ff7641594"

JOINT_NAMES = [
    "thumb_bend_joint",
    "thumb_rota_joint1",
    "thumb_rota_joint2",
    "index_bend_joint",
    "index_joint1",
    "index_joint2",
    "mid_joint1",
    "mid_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
]

ACTIVE = [0, 4, 5, 6, 7, 8, 9, 10, 11]
FIXED = [1, 2]
EXCLUDED = [3]

OUT = Path.home() / \
    "rehab_robot_ws/teach/xhand_open_grasp_v1.yaml"


def capture(session, label):
    print()
    print("=" * 72)
    print(f"准备记录：{label}")
    print("机械手保持无驱动。")
    print("请用手把机械手摆到目标姿态。")
    print("摆好并保持不动后按 Enter。")
    print("=" * 72)

    input()

    rows = []

    for _ in range(25):
        fresh = session.read_fresh_state(
            force_update=True
        )

        if fresh is None:
            raise RuntimeError(
                "XHand state read failed"
            )

        fingers = fresh.state.finger_state

        if len(fingers) != 12:
            raise RuntimeError(
                f"expected 12 joints, got {len(fingers)}"
            )

        rows.append([
            float(f.position)
            for f in fingers
        ])

        time.sleep(0.05)

    q = []

    spans = []

    for j in range(12):

        values = [
            row[j]
            for row in rows
        ]

        q.append(
            statistics.median(values)
        )

        spans.append(
            max(values) - min(values)
        )

    active_worst_span = max(
        spans[i]
        for i in ACTIVE
    )

    inactive_worst_span = max(
        spans[i]
        for i in FIXED + EXCLUDED
    )

    print()
    print(
        f"[CAPTURE] {label} "
        f"active_worst_span="
        f"{active_worst_span:.6f} rad "
        f"inactive_worst_span="
        f"{inactive_worst_span:.6f} rad"
    )

    for i in range(12):
        print(
            f"{i:02d} "
            f"{JOINT_NAMES[i]:20s} "
            f"{q[i]:+.6f} "
            f"span={spans[i]:.6f}"
        )

    if active_worst_span > 0.020:
        raise RuntimeError(
            f"{label} active joints not stationary enough: "
            f"worst span={active_worst_span:.6f}"
        )

    if inactive_worst_span > 0.020:
        print(
            f"[INFO] {label}: inactive/fixed joints moved "
            f"during teaching; ignored by execution target "
            f"(span={inactive_worst_span:.6f} rad)"
        )

    return q


backend = VendorXHandReadOnlyBackend()
session = XHandReadOnlySession(
    backend=backend
)

try:
    device_id = session.open(IFACE)

    print("=" * 72)
    print("XHAND OPEN / GRASP TEACH")
    print("READ ONLY - NO send_command")
    print("=" * 72)
    print(
        f"[PASS] device_id={device_id}"
    )

    q_open = capture(
        session,
        "OPEN"
    )

    print()
    print("现在准备记录空握闭合姿态。")
    print("不要夹物体。")
    print(
        "thumb_rota_joint1 / thumb_rota_joint2 / "
        "index_bend_joint 尽量不要手动改变。"
    )

    q_grasp = capture(
        session,
        "GRASP"
    )

    print()
    print("=" * 72)
    print("OPEN -> GRASP DELTA")
    print("=" * 72)

    delta = []

    for i in range(12):

        d = (
            q_grasp[i]
            - q_open[i]
        )

        delta.append(d)

        tag = (
            "ACTIVE"
            if i in ACTIVE
            else "FIXED"
            if i in FIXED
            else "EXCLUDED"
        )

        print(
            f"{i:02d} "
            f"{JOINT_NAMES[i]:20s} "
            f"{tag:8s} "
            f"open={q_open[i]:+.6f} "
            f"grasp={q_grasp[i]:+.6f} "
            f"delta={d:+.6f}"
        )

    # 固定/排除关节始终保持 mode=0。
    # 它们的手动姿态变化只记录，不作为 OPEN/GRASP 执行目标。
    for i in FIXED + EXCLUDED:

        if abs(delta[i]) > 0.020:
            print(
                f"[INFO] ignoring inactive joint change: "
                f"{JOINT_NAMES[i]} "
                f"delta={delta[i]:+.6f} rad"
            )

    data = {
        "format": "xhand_open_grasp_v1",
        "device_id": int(device_id),
        "interface": IFACE,

        "joint_names": JOINT_NAMES,

        "active_indices": ACTIVE,
        "fixed_indices": FIXED,
        "excluded_indices": EXCLUDED,

        "open": {
            "position": [
                float(v)
                for v in q_open
            ]
        },

        "grasp": {
            "position": [
                float(v)
                for v in q_grasp
            ]
        },

        "delta_open_to_grasp": [
            float(v)
            for v in delta
        ],

        "validated_for_execution": False,
    }

    OUT.write_text(
        yaml.safe_dump(
            data,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    print()
    print(
        f"[PASS] saved: {OUT}"
    )

finally:

    session.close()

    print(
        "[PASS] read-only EtherCAT closed"
    )
