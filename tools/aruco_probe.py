#!/usr/bin/env python3

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


TOPIC = "/camera/camera/color/image_raw"

DICT_CANDIDATES = [
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
    ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ("DICT_7X7_50", cv2.aruco.DICT_7X7_50),
    ("DICT_APRILTAG_36h11", cv2.aruco.DICT_APRILTAG_36h11),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
]


class ArucoProbe(Node):

    def __init__(self):
        super().__init__("aruco_probe")

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_result = None

        self.params = cv2.aruco.DetectorParameters_create()

        self.create_subscription(
            Image,
            TOPIC,
            self.cb,
            10,
        )

        cv2.namedWindow("Aruco Probe")

        self.get_logger().info(
            f"Listening: {TOPIC}"
        )

    def cb(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        display = frame.copy()
        detections = []

        for name, dict_id in DICT_CANDIDATES:

            dictionary = cv2.aruco.Dictionary_get(
                dict_id
            )

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                dictionary,
                parameters=self.params,
            )

            if ids is None:
                continue

            ids_flat = ids.flatten().tolist()

            detections.append(
                (name, ids_flat)
            )

            cv2.aruco.drawDetectedMarkers(
                display,
                corners,
                ids,
            )

        result = tuple(
            (name, tuple(ids))
            for name, ids in detections
        )

        if result and result != self.last_result:

            print()
            print("========================================")
            print("[DETECTED]")

            for name, ids in detections:
                print(
                    f"dictionary = {name}, "
                    f"id = {ids}"
                )

            print("========================================")

            self.last_result = result

        if not detections:

            cv2.putText(
                display,
                "NO MARKER",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        else:

            y = 35

            for name, ids in detections:

                txt = f"{name} ID={ids}"

                cv2.putText(
                    display,
                    txt,
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                y += 25

        cv2.imshow(
            "Aruco Probe",
            display,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            raise KeyboardInterrupt


def main():

    rclpy.init()
    node = ArucoProbe()

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
