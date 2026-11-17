#!/usr/bin/env python3

import time
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from franka_msgs.msg import FrankaRobotState


TOPIC = "/franka_robot_state_broadcaster/robot_state"
COLLECT_S = 4.0
G = 9.81


class Probe(Node):

    def __init__(self):
        super().__init__("fr3_static_load_probe")

        self.samples = []
        self.t0 = None

        self.create_subscription(
            FrankaRobotState,
            TOPIC,
            self.cb,
            qos_profile_sensor_data,
        )

    def cb(self, msg):

        if self.t0 is None:
            self.t0 = time.monotonic()

        ko = msg.k_f_ext_hat_k.wrench
        oo = msg.o_f_ext_hat_k.wrench

        self.samples.append([
            oo.force.x,
            oo.force.y,
            oo.force.z,
            oo.torque.x,
            oo.torque.y,
            oo.torque.z,

            ko.force.x,
            ko.force.y,
            ko.force.z,
            ko.torque.x,
            ko.torque.y,
            ko.torque.z,
        ])


def robust_stats(x):

    med = np.median(x, axis=0)
    mad = np.median(
        np.abs(x - med),
        axis=0
    )

    return med, mad


def main():

    rclpy.init()
    node = Probe()

    print("========================================")
    print("FR3 STATIC LOAD PROBE")
    print("READ ONLY - NO MOTION")
    print(f"collect = {COLLECT_S:.1f} s")
    print("========================================")

    start = time.monotonic()

    while (
        rclpy.ok()
        and time.monotonic() - start < COLLECT_S
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.01
        )

    if len(node.samples) < 20:
        print(
            f"[FAIL] insufficient samples: "
            f"{len(node.samples)}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 2

    x = np.asarray(
        node.samples,
        dtype=float
    )

    med, mad = robust_stats(x)

    Fo = med[0:3]
    To = med[3:6]

    Fk = med[6:9]
    Tk = med[9:12]

    force_norm = float(
        np.linalg.norm(Fo)
    )

    mass_equiv = (
        force_norm / G
    )

    # Assuming residual CoM lies on K-frame z axis:
    #
    # r = [0,0,z]
    # T = r x F
    #
    # Tx = -z Fy
    # Ty =  z Fx
    #
    # LS solution:
    den = (
        Fk[0] * Fk[0]
        + Fk[1] * Fk[1]
    )

    z_k = None

    if den > 1.0:
        z_k = (
            -Fk[1] * Tk[0]
            + Fk[0] * Tk[1]
        ) / den

    lateral_o = float(
        np.hypot(Fo[0], Fo[1])
    )

    print()
    print("========== SAMPLES ==========")
    print(f"N = {len(x)}")

    print()
    print("========== O FRAME ==========")
    print(
        "Force  [N]   = "
        f"[{Fo[0]:+.4f}, "
        f"{Fo[1]:+.4f}, "
        f"{Fo[2]:+.4f}]"
    )
    print(
        "Torque [Nm]  = "
        f"[{To[0]:+.4f}, "
        f"{To[1]:+.4f}, "
        f"{To[2]:+.4f}]"
    )
    print(
        f"|F|          = "
        f"{force_norm:.4f} N"
    )
    print(
        f"equiv mass   = "
        f"{mass_equiv:.4f} kg"
    )
    print(
        f"lateral force= "
        f"{lateral_o:.4f} N"
    )

    print()
    print("========== K FRAME ==========")
    print(
        "Force  [N]   = "
        f"[{Fk[0]:+.4f}, "
        f"{Fk[1]:+.4f}, "
        f"{Fk[2]:+.4f}]"
    )
    print(
        "Torque [Nm]  = "
        f"[{Tk[0]:+.4f}, "
        f"{Tk[1]:+.4f}, "
        f"{Tk[2]:+.4f}]"
    )

    if z_k is not None:
        print(
            f"z_equiv(K)   = "
            f"{z_k:+.4f} m"
        )
    else:
        print(
            "z_equiv(K)   = unavailable "
            "(insufficient lateral force)"
        )

    print()
    print("========== MEDIAN ABS DEV ==========")
    labels = [
        "O_Fx", "O_Fy", "O_Fz",
        "O_Tx", "O_Ty", "O_Tz",
        "K_Fx", "K_Fy", "K_Fz",
        "K_Tx", "K_Ty", "K_Tz",
    ]

    for name, value in zip(
        labels,
        mad
    ):
        print(
            f"{name:5s} MAD = "
            f"{value:.6f}"
        )

    print()
    print("========================================")
    print("[PASS] READ-ONLY STATIC PROBE")
    print("No controller activated.")
    print("No motion sent.")
    print("========================================")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
