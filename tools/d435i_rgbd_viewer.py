#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"


class RGBDViewer(Node):

    def __init__(self):
        super().__init__("d435i_rgbd_viewer")

        self.bridge = CvBridge()

        self.color = None
        self.depth = None

        self.color_count = 0
        self.depth_count = 0

        self.t0 = time.monotonic()

        self.create_subscription(
            Image,
            COLOR_TOPIC,
            self.color_cb,
            10,
        )

        self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_cb,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.update_view,
        )

        self.get_logger().info(
            f"color: {COLOR_TOPIC}"
        )
        self.get_logger().info(
            f"depth: {DEPTH_TOPIC}"
        )

    def color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

        self.color_count += 1

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )

        self.depth_count += 1

    def update_view(self):

        if self.color is not None:
            color = self.color.copy()

            cv2.putText(
                color,
                "D435i COLOR",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

            cv2.imshow(
                "D435i Color",
                color,
            )

        if self.depth is not None:

            depth = self.depth.copy()

            valid = depth > 0

            if np.any(valid):
                valid_depth = depth[valid]

                lo = np.percentile(
                    valid_depth,
                    2,
                )

                hi = np.percentile(
                    valid_depth,
                    98,
                )

                if hi <= lo:
                    hi = lo + 1

                vis = np.clip(
                    (depth.astype(np.float32) - lo)
                    / (hi - lo),
                    0,
                    1,
                )

                vis = (
                    vis * 255
                ).astype(np.uint8)

            else:
                vis = np.zeros_like(
                    depth,
                    dtype=np.uint8,
                )

            vis = cv2.applyColorMap(
                vis,
                cv2.COLORMAP_JET,
            )

            h, w = depth.shape[:2]

            u = w // 2
            v = h // 2

            z_raw = int(depth[v, u])

            valid_ratio = (
                100.0
                * np.count_nonzero(valid)
                / depth.size
            )

            cv2.drawMarker(
                vis,
                (u, v),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                20,
                2,
            )

            cv2.putText(
                vis,
                f"center raw depth = {z_raw}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                vis,
                f"valid = {valid_ratio:.1f}%",
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow(
                "D435i Depth",
                vis,
            )

        now = time.monotonic()

        dt = now - self.t0

        if dt >= 2.0:

            color_fps = (
                self.color_count / dt
            )

            depth_fps = (
                self.depth_count / dt
            )

            print(
                f"[FPS] color={color_fps:.1f} "
                f"depth={depth_fps:.1f}"
            )

            self.color_count = 0
            self.depth_count = 0
            self.t0 = now

        key = cv2.waitKey(1) & 0xFF

        if key in (
            ord("q"),
            27,
        ):
            raise KeyboardInterrupt


def main():

    rclpy.init()

    node = RGBDViewer()

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
