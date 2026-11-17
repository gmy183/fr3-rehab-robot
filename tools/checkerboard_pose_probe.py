#!/usr/bin/env python3

import time
import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


IMAGE_TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC  = "/camera/camera/color/camera_info"

# 12 x 9 squares -> 11 x 8 inner corners
PATTERN_SIZE = (11, 8)

# 25 mm
SQUARE_SIZE_M = 0.025


class CheckerboardPoseProbe(Node):

    def __init__(self):
        super().__init__("checkerboard_pose_probe")

        self.bridge = CvBridge()

        self.K = None
        self.D = None

        self.latest_T = None
        self.latest_rmse = None
        self.last_print = 0.0

        self.objp = np.zeros(
            (PATTERN_SIZE[0] * PATTERN_SIZE[1], 3),
            dtype=np.float32
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
            "D435i Checkerboard Pose",
            cv2.WINDOW_NORMAL
        )

        self.get_logger().info(
            f"pattern={PATTERN_SIZE}, "
            f"square={SQUARE_SIZE_M * 1000:.1f} mm"
        )

        self.get_logger().info(
            f"image={IMAGE_TOPIC}"
        )

    def info_cb(self, msg):

        self.K = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.D = np.array(
            msg.d,
            dtype=np.float64
        )

    def detect(self, gray):

        if hasattr(cv2, "findChessboardCornersSB"):

            flags = (
                cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_EXHAUSTIVE
                | cv2.CALIB_CB_ACCURACY
            )

            ok, corners = cv2.findChessboardCornersSB(
                gray,
                PATTERN_SIZE,
                flags=flags
            )

            return ok, corners

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        ok, corners = cv2.findChessboardCorners(
            gray,
            PATTERN_SIZE,
            flags
        )

        if ok:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (
                    cv2.TERM_CRITERIA_EPS
                    | cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001
                )
            )

        return ok, corners

    def image_cb(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        display = frame.copy()

        if self.K is None:
            cv2.putText(
                display,
                "WAITING CAMERA INFO",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.imshow(
                "D435i Checkerboard Pose",
                display
            )

            cv2.waitKey(1)
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        found, corners = self.detect(gray)

        if not found:

            cv2.putText(
                display,
                "CHECKERBOARD: NOT FOUND",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        else:

            cv2.drawChessboardCorners(
                display,
                PATTERN_SIZE,
                corners,
                found
            )

            ok, rvec, tvec = cv2.solvePnP(
                self.objp,
                corners,
                self.K,
                self.D,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if ok:

                projected, _ = cv2.projectPoints(
                    self.objp,
                    rvec,
                    tvec,
                    self.K,
                    self.D
                )

                err = (
                    projected.reshape(-1, 2)
                    - corners.reshape(-1, 2)
                )

                rmse = float(
                    np.sqrt(
                        np.mean(
                            np.sum(
                                err * err,
                                axis=1
                            )
                        )
                    )
                )

                R, _ = cv2.Rodrigues(rvec)

                T = np.eye(4, dtype=np.float64)
                T[:3, :3] = R
                T[:3, 3] = tvec.reshape(3)

                self.latest_T = T
                self.latest_rmse = rmse

                x, y, z = tvec.reshape(3)

                cv2.drawFrameAxes(
                    display,
                    self.K,
                    self.D,
                    rvec,
                    tvec,
                    0.075,
                    2
                )

                cv2.putText(
                    display,
                    "CHECKERBOARD: FOUND 11x8",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    display,
                    f"X={x*1000:+.1f} mm  "
                    f"Y={y*1000:+.1f} mm  "
                    f"Z={z*1000:+.1f} mm",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    display,
                    f"reproj RMSE={rmse:.3f} px",
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 0),
                    2
                )

                now = time.monotonic()

                if now - self.last_print >= 1.0:

                    print(
                        f"[POSE] "
                        f"X={x*1000:+8.2f} mm  "
                        f"Y={y*1000:+8.2f} mm  "
                        f"Z={z*1000:+8.2f} mm  "
                        f"RMSE={rmse:.3f} px"
                    )

                    self.last_print = now

        cv2.putText(
            display,
            "S: print matrix   Q/ESC: quit",
            (20, display.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "D435i Checkerboard Pose",
            display
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s") and self.latest_T is not None:

            print()
            print("========================================")
            print("camera_color_optical_frame_T_board")
            print(self.latest_T)
            print(
                f"reprojection RMSE = "
                f"{self.latest_rmse:.4f} px"
            )
            print("========================================")
            print()

        if key in (ord("q"), 27):
            raise KeyboardInterrupt


def main():

    rclpy.init()

    node = CheckerboardPoseProbe()

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
