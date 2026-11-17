#!/usr/bin/env python3
"""Standalone D435i ArUco preview. No FR3 health checks, TF, or sample writes."""
from __future__ import annotations

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Image, CameraInfo

from handeye_math import (
    extract_board, image_stamp_sec, MARKER_DICTIONARY_NAME, MARKER_ID, MARKER_SIZE_M
)

IMAGE_TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"


def frame_to_bgr(msg):
    kind = msg.encoding.lower()
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}.get(kind)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < msg.step * msg.height:
        raise ValueError("truncated image buffer")
    frame = raw[:msg.step * msg.height].reshape(msg.height, msg.step)
    frame = np.ascontiguousarray(frame[:, :msg.width * channels])
    if channels > 1:
        frame = frame.reshape(msg.height, msg.width, channels)
    if kind == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if kind == "rgba8":
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if kind == "bgra8":
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if kind == "mono8":
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


class Preview(Node):
    def __init__(self):
        super().__init__("handeye_aruco_preview_no_gate")
        self.image = None
        self.info = None
        preview_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Image, IMAGE_TOPIC, self.on_image, preview_qos
        )
        self.create_subscription(
            CameraInfo, INFO_TOPIC, self.on_info, preview_qos
        )

    def on_image(self, msg):
        self.image = msg

    def on_info(self, msg):
        self.info = msg


def main():
    rclpy.init()
    node = Preview()
    last_stamp = -1.0
    last_view = None
    last_log = 0.0
    window = "D435i ArUco preview (NO FR3 GATE)"
    print(f"[PREVIEW] image={IMAGE_TOPIC}")
    print(f"[TARGET] {MARKER_DICTIONARY_NAME}, ID={MARKER_ID}, "
          f"outer edge={MARKER_SIZE_M*1000:.1f} mm")
    print("[INFO] No controller/hardware checks, no TF lookups, no robot commands, no sample writes.")
    print("[KEY] q or ESC closes the preview.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            msg, info = node.image, node.info
            if msg is not None:
                stamp = image_stamp_sec(msg)
                if stamp != last_stamp:
                    last_stamp = stamp
                    try:
                        frame = frame_to_bgr(msg)
                        result = None
                        if info is not None:
                            if info.width != msg.width or info.height != msg.height:
                                raise ValueError("CameraInfo/image resolution mismatch")
                            K = np.asarray(info.k, dtype=float).reshape(3, 3)
                            D = np.asarray(info.d, dtype=float)
                            result = extract_board(frame, K, D, marker_size_m=MARKER_SIZE_M)
                        if result is None:
                            cv2.putText(frame, f"ARUCO ID {MARKER_ID}: NOT FOUND / WAIT INFO",
                                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                                        (0, 0, 230), 2)
                        else:
                            pts = np.round(result[2].reshape(4, 2)).astype(np.int32)
                            cv2.polylines(frame, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
                            cv2.putText(frame,
                                        f"ARUCO ID {MARKER_ID}: FOUND | RMSE {result[1]:.2f}px",
                                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                                        (0, 220, 0), 2)
                        last_view = frame
                        if time.monotonic() - last_log >= 1.0:
                            if result is None:
                                print("[DETECT] marker not found (preview continues)", flush=True)
                            else:
                                print(f"[DETECT] ID={MARKER_ID} RMSE={result[1]:.3f}px",
                                      flush=True)
                            last_log = time.monotonic()
                    except Exception as exc:
                        print(f"[WARN] preview frame: {type(exc).__name__}: {exc}", flush=True)
            if last_view is not None:
                cv2.imshow(window, last_view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
