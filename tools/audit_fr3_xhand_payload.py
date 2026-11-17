from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy

from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


BASE_LINK = "fr3_link8"

KEYWORDS = (
    "xhand",
    "hand",
    "gripper",
    "tool",
    "adapter",
    "mount",
    "end_effector",
    "endeffector",
)


# ============================================================
# Generic helpers
# ============================================================

def run_command(args, timeout=6.0):

    try:
        p = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )

        return p.returncode, p.stdout

    except subprocess.TimeoutExpired as exc:

        output = exc.stdout or ""

        if isinstance(output, bytes):
            output = output.decode(
                errors="replace"
            )

        return 124, output + "\n[TIMEOUT]"


def vec3(text, default=(0.0, 0.0, 0.0)):

    if not text:
        return tuple(default)

    try:

        values = [
            float(v)
            for v in text.split()
        ]

        if len(values) != 3:
            return tuple(default)

        return tuple(values)

    except Exception:
        return tuple(default)


def eye3():

    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def mat_add(a, b):

    return [
        [
            a[i][j] + b[i][j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def mat_mul(a, b):

    return [
        [
            sum(
                a[i][k] * b[k][j]
                for k in range(3)
            )
            for j in range(3)
        ]
        for i in range(3)
    ]


def mat_transpose(a):

    return [
        [a[j][i] for j in range(3)]
        for i in range(3)
    ]


def mat_vec(a, v):

    return [
        sum(
            a[i][j] * v[j]
            for j in range(3)
        )
        for i in range(3)
    ]


def vec_add(a, b):

    return [
        a[i] + b[i]
        for i in range(3)
    ]


def vec_sub(a, b):

    return [
        a[i] - b[i]
        for i in range(3)
    ]


def vec_scale(a, s):

    return [
        a[i] * s
        for i in range(3)
    ]


def outer(a, b):

    return [
        [
            a[i] * b[j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def rpy_matrix(rpy):

    r, p, y = rpy

    cr = math.cos(r)
    sr = math.sin(r)

    cp = math.cos(p)
    sp = math.sin(p)

    cy = math.cos(y)
    sy = math.sin(y)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]

    ry = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]

    rz = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]

    return mat_mul(
        mat_mul(rz, ry),
        rx,
    )


def quaternion_matrix(x, y, z, w):

    n = math.sqrt(
        x*x + y*y + z*z + w*w
    )

    if n <= 1e-12:
        return eye3()

    x /= n
    y /= n
    z /= n
    w /= n

    return [
        [
            1.0 - 2.0*(y*y + z*z),
            2.0*(x*y - z*w),
            2.0*(x*z + y*w),
        ],
        [
            2.0*(x*y + z*w),
            1.0 - 2.0*(x*x + z*z),
            2.0*(y*z - x*w),
        ],
        [
            2.0*(x*z - y*w),
            2.0*(y*z + x*w),
            1.0 - 2.0*(x*x + y*y),
        ],
    ]


def inertia_matrix_from_urdf(elem):

    if elem is None:
        return [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

    def f(name):
        try:
            return float(
                elem.attrib.get(
                    name,
                    "0",
                )
            )
        except Exception:
            return 0.0

    ixx = f("ixx")
    ixy = f("ixy")
    ixz = f("ixz")
    iyy = f("iyy")
    iyz = f("iyz")
    izz = f("izz")

    return [
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ]


# ============================================================
# ROS helper node
# ============================================================

class AuditNode(Node):

    def __init__(self):

        super().__init__(
            "fr3_xhand_payload_audit"
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

    def get_robot_description(self):

        client = self.create_client(
            GetParameters,
            "/robot_state_publisher/"
            "get_parameters",
        )

        if not client.wait_for_service(
            timeout_sec=3.0
        ):
            return None

        request = GetParameters.Request()

        request.names = [
            "robot_description"
        ]

        future = client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done():
            return None

        response = future.result()

        if (
            response is None
            or not response.values
        ):
            return None

        text = (
            response.values[0]
            .string_value
        )

        return text or None

    def collect_tf(self, seconds=2.0):

        deadline = (
            time.monotonic()
            + seconds
        )

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

    def lookup(self, parent, child):

        try:

            tf = (
                self.tf_buffer
                .lookup_transform(
                    parent,
                    child,
                    Time(),
                )
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            R = quaternion_matrix(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )

            p = [
                float(t.x),
                float(t.y),
                float(t.z),
            ]

            return p, R

        except Exception:
            return None


# ============================================================
# URDF payload analysis
# ============================================================

def parse_robot_description(xml_text):

    root = ET.fromstring(xml_text)

    links = {}

    for link in root.findall("link"):

        name = link.attrib.get(
            "name",
            "",
        )

        inertial = link.find(
            "inertial"
        )

        mass = 0.0
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)

        I = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

        if inertial is not None:

            mass_elem = inertial.find(
                "mass"
            )

            if mass_elem is not None:

                try:
                    mass = float(
                        mass_elem.attrib.get(
                            "value",
                            "0",
                        )
                    )
                except Exception:
                    mass = 0.0

            origin = inertial.find(
                "origin"
            )

            if origin is not None:

                xyz = vec3(
                    origin.attrib.get(
                        "xyz"
                    )
                )

                rpy = vec3(
                    origin.attrib.get(
                        "rpy"
                    )
                )

            I = inertia_matrix_from_urdf(
                inertial.find(
                    "inertia"
                )
            )

        links[name] = {
            "mass": mass,
            "com_xyz_link": list(xyz),
            "inertial_rpy": list(rpy),
            "inertia_local": I,
        }

    joints = []

    children = {}

    for joint in root.findall("joint"):

        name = joint.attrib.get(
            "name",
            "",
        )

        joint_type = joint.attrib.get(
            "type",
            "",
        )

        parent_elem = joint.find(
            "parent"
        )

        child_elem = joint.find(
            "child"
        )

        if (
            parent_elem is None
            or child_elem is None
        ):
            continue

        parent = parent_elem.attrib.get(
            "link",
            "",
        )

        child = child_elem.attrib.get(
            "link",
            "",
        )

        item = {
            "name": name,
            "type": joint_type,
            "parent": parent,
            "child": child,
        }

        joints.append(item)

        children.setdefault(
            parent,
            [],
        ).append(item)

    return links, joints, children


def descendants_from(
    base_link,
    children,
):

    found = []

    queue = [base_link]

    visited = {base_link}

    while queue:

        parent = queue.pop(0)

        for joint in children.get(
            parent,
            [],
        ):

            child = joint["child"]

            if child in visited:
                continue

            visited.add(child)

            found.append(joint)

            queue.append(child)

    return found


def compute_payload_candidate(
    node,
    links,
    descendants,
):

    link_names = []

    for joint in descendants:

        child = joint["child"]

        if child not in link_names:
            link_names.append(child)

    modeled = []

    unresolved = []

    for name in link_names:

        data = links.get(
            name,
            {},
        )

        mass = float(
            data.get(
                "mass",
                0.0,
            )
        )

        if mass <= 0.0:
            continue

        tf = node.lookup(
            BASE_LINK,
            name,
        )

        if tf is None:

            unresolved.append(
                name
            )

            modeled.append({
                "link": name,
                "mass_kg": mass,
                "tf_resolved": False,
            })

            continue

        translation, R_link = tf

        local_com = data[
            "com_xyz_link"
        ]

        com_base = vec_add(
            translation,
            mat_vec(
                R_link,
                local_com,
            ),
        )

        R_inertial = rpy_matrix(
            data[
                "inertial_rpy"
            ]
        )

        I_local = data[
            "inertia_local"
        ]

        # inertia -> link frame
        I_link = mat_mul(
            mat_mul(
                R_inertial,
                I_local,
            ),
            mat_transpose(
                R_inertial
            ),
        )

        # link frame -> fr3_link8
        I_base_com = mat_mul(
            mat_mul(
                R_link,
                I_link,
            ),
            mat_transpose(
                R_link
            ),
        )

        modeled.append({
            "link": name,
            "mass_kg": mass,
            "tf_resolved": True,
            "com_in_fr3_link8_m":
                com_base,
            "inertia_at_own_com_"
            "in_fr3_link8_kgm2":
                I_base_com,
        })

    resolved = [
        item
        for item in modeled
        if item.get(
            "tf_resolved"
        )
    ]

    total_mass = sum(
        item["mass_kg"]
        for item in resolved
    )

    if total_mass <= 0.0:

        return {
            "modeled_links": modeled,
            "unresolved_links": unresolved,
            "total_resolved_mass_kg":
                0.0,
            "combined_com_fr3_link8_m":
                None,
            "combined_inertia_at_com_"
            "fr3_link8_kgm2":
                None,
        }

    combined_com = [
        0.0,
        0.0,
        0.0,
    ]

    for item in resolved:

        combined_com = vec_add(
            combined_com,
            vec_scale(
                item[
                    "com_in_fr3_link8_m"
                ],
                item["mass_kg"],
            ),
        )

    combined_com = vec_scale(
        combined_com,
        1.0 / total_mass,
    )

    total_I = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]

    for item in resolved:

        mass = item["mass_kg"]

        p = item[
            "com_in_fr3_link8_m"
        ]

        d = vec_sub(
            p,
            combined_com,
        )

        d2 = sum(
            x*x
            for x in d
        )

        parallel_axis = [
            [
                mass * (
                    (
                        d2
                        if i == j
                        else 0.0
                    )
                    - d[i] * d[j]
                )
                for j in range(3)
            ]
            for i in range(3)
        ]

        total_I = mat_add(
            total_I,
            mat_add(
                item[
                    "inertia_at_own_com_"
                    "in_fr3_link8_kgm2"
                ],
                parallel_axis,
            ),
        )

    return {
        "modeled_links":
            modeled,

        "unresolved_links":
            unresolved,

        "total_resolved_mass_kg":
            total_mass,

        "combined_com_fr3_link8_m":
            combined_com,

        "combined_inertia_at_com_"
        "fr3_link8_kgm2":
            total_I,
    }


# ============================================================
# Main
# ============================================================

def main():

    if len(sys.argv) != 2:

        print(
            "usage: audit_fr3_xhand_payload.py "
            "<output_json>"
        )

        return 2

    output_path = Path(
        sys.argv[1]
    )

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    result = {
        "schema":
            "fr3_xhand_payload_audit_v1",

        "read_only":
            True,

        "load_changed":
            False,
    }

    print("=" * 72)
    print(
        "FR3 + XHAND PAYLOAD CONFIGURATION AUDIT V1"
    )
    print(
        "READ ONLY / NO MOTION / NO SETLOAD CALL"
    )
    print("=" * 72)


    # --------------------------------------------------------
    # 1. Current controllers
    # --------------------------------------------------------

    print()
    print(
        "========== 1. Current controller baseline =========="
    )

    rc, controllers = run_command(
        [
            "ros2",
            "control",
            "list_controllers",
        ]
    )

    print(
        controllers.rstrip()
    )

    result["controllers_raw"] = (
        controllers
    )


    # --------------------------------------------------------
    # 2. Health
    # --------------------------------------------------------

    print()
    print(
        "========== 2. Current FR3 health =========="
    )

    rc, state_text = run_command(
        [
            "ros2",
            "topic",
            "echo",
            "/franka_robot_state_broadcaster/"
            "robot_state",
            "--once",
        ],
        timeout=5.0,
    )

    health_lines = []

    for line in state_text.splitlines():

        lower = line.lower()

        if any(
            key in lower
            for key in (
                "robot_mode:",
                "joint_reflex:",
                "cartesian_reflex:",
                "communication_constraints_"
                "violation:",
                "instability_detected:",
            )
        ):

            print(line)

            health_lines.append(
                line
            )

    result[
        "robot_state_health_lines"
    ] = health_lines


    # --------------------------------------------------------
    # 3. SetLoad discovery
    # --------------------------------------------------------

    print()
    print(
        "========== 3. Discover payload/load services =========="
    )

    rc, services = run_command(
        [
            "ros2",
            "service",
            "list",
            "-t",
        ]
    )

    load_service_lines = []

    for line in services.splitlines():

        lower = line.lower()

        if (
            "load" in lower
            or "payload" in lower
        ):

            print(line)

            load_service_lines.append(
                line
            )

    if not load_service_lines:
        print(
            "[INFO] no service with load/payload "
            "in its name/type"
        )

    setload_services = [
        line
        for line in services.splitlines()
        if (
            "setload" in line.lower()
            or "set_load" in line.lower()
        )
    ]

    result[
        "load_related_services"
    ] = load_service_lines

    result[
        "setload_service_candidates"
    ] = setload_services

    print()
    print(
        "--- franka_msgs/srv/SetLoad interface ---"
    )

    rc, interface = run_command(
        [
            "ros2",
            "interface",
            "show",
            "franka_msgs/srv/SetLoad",
        ]
    )

    print(
        interface.rstrip()
        if interface.strip()
        else "[INFO] SetLoad interface unavailable"
    )

    result[
        "setload_interface_available"
    ] = (
        rc == 0
        and bool(interface.strip())
    )

    result[
        "setload_interface_raw"
    ] = interface


    # --------------------------------------------------------
    # 4. Runtime robot state payload readback
    # --------------------------------------------------------

    print()
    print(
        "========== 4. Runtime payload readback =========="
    )

    payload_patterns = (
        "m_ee",
        "m_load",
        "m_total",
        "i_ee",
        "i_load",
        "i_total",
        "f_x_cee",
        "f_x_cload",
        "f_x_ctotal",
        "payload",
        "center_of_mass",
    )

    payload_lines = []

    for line in state_text.splitlines():

        lower = line.lower()

        if any(
            token in lower
            for token in payload_patterns
        ):

            payload_lines.append(
                line
            )

    if payload_lines:

        for line in payload_lines:
            print(line)

        print(
            "[PASS] runtime payload-like "
            "fields are exposed"
        )

    else:

        print(
            "[INFO] current robot_state topic "
            "does not expose payload fields"
        )

    result[
        "runtime_payload_readback_lines"
    ] = payload_lines

    result[
        "runtime_payload_readback_available"
    ] = bool(payload_lines)


    # --------------------------------------------------------
    # 5. Relevant node parameters
    # --------------------------------------------------------

    print()
    print(
        "========== 5. Payload-related ROS parameters =========="
    )

    nodes_to_probe = [
        "/frankahardwareinterface",
        "/service_server",
        "/franka_robot_state_broadcaster",
        "/controller_manager",
    ]

    parameter_hits = {}

    for node_name in nodes_to_probe:

        rc, text = run_command(
            [
                "ros2",
                "param",
                "dump",
                node_name,
            ],
            timeout=5.0,
        )

        hits = []

        for line in text.splitlines():

            lower = line.lower()

            if any(
                word in lower
                for word in (
                    "load",
                    "mass",
                    "inertia",
                    "center",
                    "com",
                    "m_ee",
                    "f_x_c",
                    "ee_frame",
                )
            ):

                hits.append(line)

        if hits:

            print()
            print(
                f"--- {node_name} ---"
            )

            for line in hits:
                print(line)

        parameter_hits[
            node_name
        ] = hits

    if not any(
        parameter_hits.values()
    ):
        print(
            "[INFO] no payload-related runtime "
            "parameters exposed by these nodes"
        )

    result[
        "payload_parameter_hits"
    ] = parameter_hits


    # --------------------------------------------------------
    # 6. Runtime URDF + XHand model
    # --------------------------------------------------------

    print()
    print(
        "========== 6. Runtime robot_description audit =========="
    )

    rclpy.init()

    node = AuditNode()

    try:

        xml_text = (
            node.get_robot_description()
        )

        if not xml_text:

            print(
                "[FAIL] robot_description unavailable"
            )

            result[
                "robot_description_available"
            ] = False

        else:

            result[
                "robot_description_available"
            ] = True

            print(
                f"[PASS] robot_description loaded "
                f"({len(xml_text)} chars)"
            )

            links, joints, children = (
                parse_robot_description(
                    xml_text
                )
            )

            if BASE_LINK not in links:

                print(
                    f"[FAIL] {BASE_LINK} not present "
                    "in robot_description"
                )

                result[
                    "base_link_present"
                ] = False

            else:

                result[
                    "base_link_present"
                ] = True

                descendants = descendants_from(
                    BASE_LINK,
                    children,
                )

                print(
                    f"links/joints downstream of "
                    f"{BASE_LINK}: "
                    f"{len(descendants)}"
                )

                downstream_records = []

                for item in descendants:

                    child = item["child"]

                    mass = links.get(
                        child,
                        {},
                    ).get(
                        "mass",
                        0.0,
                    )

                    record = {
                        **item,
                        "mass_kg":
                            float(mass),
                    }

                    downstream_records.append(
                        record
                    )

                    marker = (
                        " <== possible XHand/tool"
                        if any(
                            key in (
                                item["name"]
                                + " "
                                + child
                            ).lower()
                            for key in KEYWORDS
                        )
                        else ""
                    )

                    print(
                        f"{item['parent']} "
                        f"--[{item['name']}, "
                        f"{item['type']}]--> "
                        f"{child} "
                        f"mass={mass:.6f} kg"
                        f"{marker}"
                    )

                result[
                    "downstream_records"
                ] = downstream_records

                keyword_matches = [
                    item
                    for item in downstream_records
                    if any(
                        key in (
                            item["name"]
                            + " "
                            + item["child"]
                        ).lower()
                        for key in KEYWORDS
                    )
                ]

                result[
                    "possible_xhand_tool_matches"
                ] = keyword_matches

                print()
                print(
                    "collecting current TF tree..."
                )

                node.collect_tf(
                    seconds=2.0
                )

                payload = (
                    compute_payload_candidate(
                        node,
                        links,
                        descendants,
                    )
                )

                result[
                    "urdf_payload_candidate"
                ] = payload

                mass = payload[
                    "total_resolved_mass_kg"
                ]

                print()
                print(
                    "---------- URDF CURRENT-CONFIG "
                    "PAYLOAD CANDIDATE ----------"
                )

                print(
                    f"resolved modeled mass = "
                    f"{mass:.6f} kg"
                )

                com = payload[
                    "combined_com_fr3_link8_m"
                ]

                inertia = payload[
                    "combined_inertia_at_com_"
                    "fr3_link8_kgm2"
                ]

                if com is not None:

                    print(
                        "combined COM in fr3_link8 "
                        "[m] = "
                        f"[{com[0]:+.6f}, "
                        f"{com[1]:+.6f}, "
                        f"{com[2]:+.6f}]"
                    )

                else:

                    print(
                        "combined COM = unavailable"
                    )

                if inertia is not None:

                    print(
                        "combined inertia about COM "
                        "in fr3_link8 [kg*m^2]:"
                    )

                    for row in inertia:

                        print(
                            "  ["
                            + ", ".join(
                                f"{v:+.8f}"
                                for v in row
                            )
                            + "]"
                        )

                unresolved = payload[
                    "unresolved_links"
                ]

                if unresolved:

                    print(
                        "[WARN] mass-bearing links "
                        "with unresolved TF:"
                    )

                    for name in unresolved:
                        print(
                            f"  - {name}"
                        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


    # --------------------------------------------------------
    # 7. Automatic assessment
    # --------------------------------------------------------

    print()
    print(
        "========== 7. AUTOMATIC ASSESSMENT =========="
    )

    conclusions = []

    if setload_services:

        conclusions.append(
            "SETLOAD_INTERFACE_PRESENT"
        )

        print(
            "[PASS] SetLoad-capable service/interface "
            "is discoverable"
        )

    else:

        conclusions.append(
            "SETLOAD_INTERFACE_NOT_DISCOVERED"
        )

        print(
            "[WARN] no active SetLoad service discovered"
        )

    if payload_lines:

        conclusions.append(
            "RUNTIME_PAYLOAD_READBACK_PRESENT"
        )

        print(
            "[PASS] current payload configuration "
            "can be read from robot_state"
        )

    else:

        conclusions.append(
            "RUNTIME_PAYLOAD_READBACK_NOT_EXPOSED"
        )

        print(
            "[INFO] current ROS robot_state does not "
            "directly expose payload readback"
        )

    urdf_payload = result.get(
        "urdf_payload_candidate",
        {},
    )

    candidate_mass = float(
        urdf_payload.get(
            "total_resolved_mass_kg",
            0.0,
        )
        or 0.0
    )

    matches = result.get(
        "possible_xhand_tool_matches",
        [],
    )

    unresolved = urdf_payload.get(
        "unresolved_links",
        [],
    )

    if matches:

        conclusions.append(
            "XHAND_OR_TOOL_FOUND_IN_RUNTIME_URDF"
        )

        print(
            "[PASS] XHand/tool-like links were "
            "found downstream of fr3_link8"
        )

    else:

        conclusions.append(
            "NO_CLEAR_XHAND_MODEL_IN_RUNTIME_URDF"
        )

        print(
            "[WARN] no clearly named XHand/tool "
            "model was found downstream of fr3_link8"
        )

    if candidate_mass > 0.0:

        conclusions.append(
            "URDF_HAS_NONZERO_DOWNSTREAM_MASS"
        )

        print(
            f"[PASS] URDF downstream mass candidate "
            f"= {candidate_mass:.6f} kg"
        )

    else:

        conclusions.append(
            "URDF_DOWNSTREAM_MASS_MISSING_OR_ZERO"
        )

        print(
            "[WARN] no usable downstream inertial "
            "mass was obtained from runtime URDF"
        )

    if unresolved:

        conclusions.append(
            "PAYLOAD_MODEL_TF_INCOMPLETE"
        )

        print(
            "[WARN] candidate model is incomplete "
            "because some mass links lack TF"
        )

    if (
        candidate_mass > 0.0
        and not unresolved
        and matches
    ):

        conclusions.append(
            "URDF_PAYLOAD_CANDIDATE_COMPLETE"
        )

        print()
        print(
            "[CANDIDATE] runtime URDF contains enough "
            "information to construct a payload candidate."
        )

        print(
            "[IMPORTANT] candidate is NOT automatically "
            "written to the robot."
        )

    else:

        conclusions.append(
            "MEASURED_PAYLOAD_DATA_STILL_REQUIRED"
        )

        print()
        print(
            "[ACTION] actual XHand + adapter measured "
            "mass/COM data is still required before SetLoad."
        )

    result[
        "automatic_conclusions"
    ] = conclusions

    result[
        "safe_to_auto_apply_load"
    ] = False

    result[
        "reason_auto_apply_disabled"
    ] = (
        "audit-only workflow; payload must be "
        "verified before writing robot-side load model"
    )

    output_path.write_text(
        json.dumps(
            result,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    print()
    print(
        "========== 8. Saved report =========="
    )

    print(
        f"JSON report = {output_path}"
    )

    print()
    print("=" * 72)
    print(
        "[DONE] PAYLOAD AUDIT COMPLETE"
    )
    print(
        "NO CONTROLLER LOAD"
    )
    print(
        "NO CONTROLLER ACTIVATION"
    )
    print(
        "NO TRAJECTORY"
    )
    print(
        "NO PTP"
    )
    print(
        "NO SETLOAD CALL"
    )
    print(
        "NO ROBOT CONFIGURATION WAS CHANGED"
    )
    print("=" * 72)

    return 0


if __name__ == "__main__":

    try:
        raise SystemExit(
            main()
        )

    except Exception as exc:

        print()
        print(
            "[FAULT] "
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        print(
            "[SAFE] NO ROBOT CONFIGURATION CHANGED"
        )

        raise SystemExit(2)
