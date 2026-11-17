#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/aligned_depth_to_color/camera_info"


class ClickXYZ(Node):

    def __init__(self):
        super().__init__("d435i_click_xyz")

        self.bridge = CvBridge()

        self.color = None
        self.depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.mouse_u = None
        self.mouse_v = None

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

        self.create_subscription(
            CameraInfo,
            INFO_TOPIC,
            self.info_cb,
            10,
        )

        cv2.namedWindow("D435i RGB-D XYZ")

        cv2.setMouseCallback(
            "D435i RGB-D XYZ",
            self.mouse_cb,
        )

        self.timer = self.create_timer(
            0.03,
            self.render,
        )

        self.get_logger().info("RGB-D XYZ viewer ready")

    def color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )

    def info_cb(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def mouse_cb(self, event, x, y, flags, param):

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.depth is None:
            return

        if self.fx is None:
            return

        h, w = self.depth.shape[:2]

        if not (0 <= x < w and 0 <= y < h):
            return

        z = int(self.depth[y, x])

        if z <= 0:
            print(
                f"[CLICK] u={x} v={y} depth=INVALID"
            )
            return

        X = (x - self.cx) * z / self.fx
        Y = (y - self.cy) * z / self.fy
        Z = z

        self.mouse_u = x
        self.mouse_v = y

        print()
        print("========================================")
        print(f"pixel : u={x}, v={y}")
        print(f"depth : {z} raw_units")
        print()
        print("camera_color_optical_frame:")
        print(f"X = {X:+.2f} raw_units")
        print(f"Y = {Y:+.2f} raw_units")
        print(f"Z = {Z:+.2f} raw_units")
        print("========================================")

    def render(self):

        if self.color is None:
            return

        img = self.color.copy()

        if self.mouse_u is not None:

            cv2.drawMarker(
                img,
                (self.mouse_u, self.mouse_v),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                30,
                2,
            )

        cv2.putText(
            img,
            "LEFT CLICK: read aligned depth + XYZ",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.imshow(
            "D435i RGB-D XYZ",
            img,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            raise KeyboardInterrupt


def main():

    rclpy.init()

    node = ClickXYZ()

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
