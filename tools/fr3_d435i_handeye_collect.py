#!/usr/bin/env python3

import os
import json
import time
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener


IMAGE_TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC  = "/camera/camera/color/camera_info"

BASE_FRAME = "fr3_link0"
GRIPPER_FRAME = "fr3_link8"

PATTERN_SIZE = (11, 8)
SQUARE_SIZE_M = 0.025

OUT_DIR = os.path.expanduser(
    "~/rehab_robot_ws/calibration/fr3_d435i_handeye"
)


def quat_to_rot(x, y, z, w):

    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)

    x, y, z, w = q

    return np.array([
        [
            1 - 2*(y*y + z*z),
            2*(x*y - z*w),
            2*(x*z + y*w)
        ],
        [
            2*(x*y + z*w),
            1 - 2*(x*x + z*z),
            2*(y*z - x*w)
        ],
        [
            2*(x*z - y*w),
            2*(y*z + x*w),
            1 - 2*(x*x + y*y)
        ]
    ], dtype=np.float64)


class Collector(Node):

    def __init__(self):

        super().__init__("fr3_d435i_handeye_collect")

        os.makedirs(OUT_DIR, exist_ok=True)

        self.bridge = CvBridge()

        self.K = None
        self.D = None

        self.latest_frame = None
        self.latest_cam_T_board = None
        self.latest_rmse = None

        existing = sorted(
            Path(OUT_DIR).glob("sample_*.json")
        )

        if existing:
            nums = []
            for f in existing:
                try:
                    nums.append(int(f.stem.split("_")[-1]))
                except ValueError:
                    pass

            self.sample_count = max(nums) if nums else 0
        else:
            self.sample_count = 0

        print(
            f"[RESUME] existing samples = {self.sample_count}"
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.objp = np.zeros(
            (PATTERN_SIZE[0] * PATTERN_SIZE[1], 3),
            np.float32
        )

        self.objp[:, :2] = (
            np.mgrid[
                0:PATTERN_SIZE[0],
                0:PATTERN_SIZE[1]
            ]
            .T
            .reshape(-1, 2)
            * SQUARE_SIZE_M
        )

        self.create_subscription(
            CameraInfo,
            INFO_TOPIC,
            self.info_cb,
            10
        )

        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_cb,
            10
        )

        cv2.namedWindow(
            "FR3 D435i HandEye Collector",
            cv2.WINDOW_NORMAL
        )

        cv2.setMouseCallback(
            "FR3 D435i HandEye Collector",
            self.mouse_cb
        )

        self.timer = self.create_timer(
            0.03,
            self.render
        )

        print()
        print("========================================")
        print("FR3 + D435i Eye-in-Hand Collector")
        print("========================================")
        print("S : save sample")
        print("U : undo last sample")
        print("Q : quit")
        print()
        print("Board:")
        print("  11 x 8 inner corners")
        print("  square = 25.0 mm")
        print()
        print("Robot:")
        print("  base    = fr3_link0")
        print("  gripper = fr3_link8")
        print()
        print("Camera:")
        print("  camera_color_optical_frame")
        print("========================================")

    def mouse_cb(self, event, x, y, flags, param):

        # Double left click = save current sample
        if event == cv2.EVENT_LBUTTONDBLCLK:

            print(
                f"[MOUSE] double click at x={x}, y={y}"
            )

            self.save_sample()


    def info_cb(self, msg):

        self.K = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.D = np.array(
            msg.d,
            dtype=np.float64
        )

    def image_cb(self, msg):

        self.latest_frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

    def detect_board(self, frame):

        if self.K is None:
            return None

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        flags = (
            cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_EXHAUSTIVE
            | cv2.CALIB_CB_ACCURACY
        )

        found, corners = cv2.findChessboardCornersSB(
            gray,
            PATTERN_SIZE,
            flags=flags
        )

        if not found:
            return None

        ok, rvec, tvec = cv2.solvePnP(
            self.objp,
            corners,
            self.K,
            self.D,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not ok:
            return None

        projected, _ = cv2.projectPoints(
            self.objp,
            rvec,
            tvec,
            self.K,
            self.D
        )

        e = (
            projected.reshape(-1, 2)
            - corners.reshape(-1, 2)
        )

        rmse = float(
            np.sqrt(
                np.mean(
                    np.sum(e * e, axis=1)
                )
            )
        )

        R, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)

        return corners, rvec, tvec, T, rmse

    def get_base_T_gripper(self):

        try:

            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                GRIPPER_FRAME,
                rclpy.time.Time()
            )

        except Exception as e:

            print(
                f"[BLOCKED] TF unavailable: {e}"
            )

            return None

        tr = tf.transform.translation
        qr = tf.transform.rotation

        T = np.eye(4)

        T[:3, :3] = quat_to_rot(
            qr.x,
            qr.y,
            qr.z,
            qr.w
        )

        T[:3, 3] = [
            tr.x,
            tr.y,
            tr.z
        ]

        return T

    def save_sample(self):

        if self.latest_cam_T_board is None:

            print("[BLOCKED] checkerboard not detected")
            return

        if self.latest_rmse is None:

            print("[BLOCKED] no reprojection result")
            return

        if self.latest_rmse > 0.80:

            print(
                f"[BLOCKED] RMSE too high: "
                f"{self.latest_rmse:.3f} px > 0.80 px"
            )
            return

        base_T_gripper = self.get_base_T_gripper()

        if base_T_gripper is None:
            return

        self.sample_count += 1

        record = {
            "sample": self.sample_count,
            "timestamp": datetime.now().isoformat(),
            "base_frame": BASE_FRAME,
            "gripper_frame": GRIPPER_FRAME,
            "camera_frame": "camera_color_optical_frame",
            "pattern_inner_corners": [11, 8],
            "square_size_m": SQUARE_SIZE_M,
            "reprojection_rmse_px": self.latest_rmse,
            "base_T_gripper": base_T_gripper.tolist(),
            "camera_T_board": self.latest_cam_T_board.tolist()
        }

        path = os.path.join(
            OUT_DIR,
            f"sample_{self.sample_count:03d}.json"
        )

        with open(path, "w", encoding="utf-8") as f:
            json.dump(
                record,
                f,
                indent=2
            )

        print()
        print("========================================")
        print(
            f"[SAVED] sample {self.sample_count:03d}"
        )
        print(
            f"RMSE = {self.latest_rmse:.3f} px"
        )
        print()
        print("base_T_gripper:")
        print(base_T_gripper)
        print()
        print("camera_T_board:")
        print(self.latest_cam_T_board)
        print()
        print(path)
        print("========================================")

    def undo(self):

        if self.sample_count <= 0:
            print("[INFO] no sample to undo")
            return

        path = os.path.join(
            OUT_DIR,
            f"sample_{self.sample_count:03d}.json"
        )

        if os.path.exists(path):
            os.remove(path)

        print(
            f"[UNDO] sample {self.sample_count:03d}"
        )

        self.sample_count -= 1

    def render(self):

        if self.latest_frame is None:
            return

        display = self.latest_frame.copy()

        result = self.detect_board(display)

        if result is None:

            self.latest_cam_T_board = None
            self.latest_rmse = None

            cv2.putText(
                display,
                "BOARD NOT FOUND",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        else:

            corners, rvec, tvec, T, rmse = result

            self.latest_cam_T_board = T
            self.latest_rmse = rmse

            cv2.drawChessboardCorners(
                display,
                PATTERN_SIZE,
                corners,
                True
            )

            cv2.drawFrameAxes(
                display,
                self.K,
                self.D,
                rvec,
                tvec,
                0.075,
                2
            )

            x, y, z = tvec.reshape(3)

            cv2.putText(
                display,
                f"FOUND  RMSE={rmse:.3f}px",
                (15, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                (
                    f"XYZ(mm) "
                    f"{x*1000:+.1f} "
                    f"{y*1000:+.1f} "
                    f"{z*1000:+.1f}"
                ),
                (15, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        cv2.putText(
            display,
            f"samples={self.sample_count}",
            (15, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2
        )

        cv2.putText(
            display,
            "DOUBLE CLICK save | U undo | Q quit",
            (15, display.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "FR3 D435i HandEye Collector",
            display
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            self.save_sample()

        elif key == ord("u"):
            self.undo()

        elif key in (ord("q"), 27):
            raise KeyboardInterrupt


def main():

    rclpy.init()

    node = Collector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
