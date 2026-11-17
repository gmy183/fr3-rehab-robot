"""
Reconstructed XHand ROS 2 compatibility node.

State sources:
  disabled        : ROS interface only; no state source
  fake            : pure-software full state pipeline
  vendor_readonly : real XHand read_state() only

Safety:
- vendor_readonly requires explicit hardware_enable=true
- no command subscriber exists in this node
- this node never calls send_command()
- failed reads publish nothing
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rehab_robot_interfaces.msg import XHandState

from .xhand_driver_core import XHandDriverCore
from .xhand_fake_backend import SoftwareFakeXHandBackend
from .xhand_readonly_session import (
    VendorXHandReadOnlyBackend,
    XHandReadOnlySession,
)
from .xhand_ros_state_bridge import mapped_state_to_ros
from .xhand_state_mapper import JOINT_NAMES, map_hand_state


XHAND_JOINT_NAMES = JOINT_NAMES

SUPPORTED_STATE_SOURCES = (
    "disabled",
    "fake",
    "vendor_readonly",
)


def validate_source_configuration(
    hardware_enable: bool,
    state_source: str,
) -> None:
    if state_source not in SUPPORTED_STATE_SOURCES:
        raise ValueError(
            "state_source must be one of: "
            + ", ".join(SUPPORTED_STATE_SOURCES)
        )

    if state_source == "vendor_readonly":
        if not hardware_enable:
            raise RuntimeError(
                "state_source=vendor_readonly requires "
                "hardware_enable=true"
            )
    else:
        if hardware_enable:
            raise RuntimeError(
                "hardware_enable=true is only valid with "
                "state_source=vendor_readonly in the current stage"
            )


class XHandROS2Node(Node):

    def __init__(self) -> None:
        super().__init__(
            "xhand_control_ros2",
            namespace="xhand_control",
        )

        self.declare_parameter("hardware_enable", False)
        self.declare_parameter("state_source", "disabled")
        self.declare_parameter(
            "interface",
            "enx6c1ff7641594",
        )
        self.declare_parameter("state_rate_hz", 100.0)
        self.declare_parameter("watchdog_s", 0.5)

        self._hardware_enable = bool(
            self.get_parameter("hardware_enable").value
        )
        self._state_source = str(
            self.get_parameter("state_source").value
        )
        self._interface = str(
            self.get_parameter("interface").value
        )
        self._state_rate_hz = float(
            self.get_parameter("state_rate_hz").value
        )
        self._watchdog_s = float(
            self.get_parameter("watchdog_s").value
        )

        if len(XHAND_JOINT_NAMES) != 12:
            raise RuntimeError(
                "XHand joint contract must contain 12 joints"
            )

        if self._state_rate_hz <= 0.0:
            raise ValueError(
                "state_rate_hz must be > 0"
            )

        if not 0.0 < self._watchdog_s <= 0.5:
            raise ValueError(
                "watchdog_s must be in (0, 0.5]"
            )

        # IMPORTANT:
        # This happens before any real vendor backend is constructed.
        validate_source_configuration(
            self._hardware_enable,
            self._state_source,
        )

        self._state_pub = self.create_publisher(
            XHandState,
            "xhand_state",
            10,
        )

        self._state_timer = None

        self._state_reader = None

        self._fake_backend = None
        self._fake_core = None

        self._vendor_readonly_backend = None
        self._vendor_readonly_session = None

        self._published_state_count = 0
        self._failed_state_count = 0

        if self._state_source == "fake":
            self._start_fake_state_source()

        elif self._state_source == "vendor_readonly":
            self._start_vendor_readonly_state_source()

        self.get_logger().info(
            "XHand ROS2 compatibility node ready"
        )

        self.get_logger().info(
            f"hardware_enable={str(self._hardware_enable).lower()}"
        )

        self.get_logger().info(
            f"state_source={self._state_source}"
        )

        self.get_logger().info(
            f"interface={self._interface}"
        )

        self.get_logger().info(
            f"state contract={self._state_rate_hz:.1f} Hz, "
            f"watchdog={self._watchdog_s:.3f} s"
        )

        self.get_logger().info(
            "state topic=/xhand_control/xhand_state"
        )

        self.get_logger().info(
            "command interface is disabled in this node"
        )

    def _create_state_timer(self) -> None:
        period = 1.0 / self._state_rate_hz

        self._state_timer = self.create_timer(
            period,
            self._publish_state_cycle,
        )

    def _start_fake_state_source(self) -> None:
        backend = SoftwareFakeXHandBackend()

        core = XHandDriverCore(
            backend=backend,
            command_timeout_s=self._watchdog_s,
        )

        # Calls only the in-memory fake backend.
        core.open("software_fake")

        self._fake_backend = backend
        self._fake_core = core
        self._state_reader = core

        self._create_state_timer()

        self.get_logger().info(
            "software fake state source enabled"
        )

    def _start_vendor_readonly_state_source(self) -> None:
        """
        This is the only path that can construct XHandControl and
        open the physical EtherCAT interface.

        It is reachable only after:
            hardware_enable=true
            state_source=vendor_readonly

        There is no send_command API in this path.
        """

        backend = VendorXHandReadOnlyBackend()

        session = XHandReadOnlySession(
            backend=backend,
        )

        try:
            device_id = session.open(
                self._interface
            )
        except Exception:
            # open() already closes the backend when it opened
            # successfully but no hand ID was found.
            try:
                session.close()
            except Exception:
                pass
            raise

        self._vendor_readonly_backend = backend
        self._vendor_readonly_session = session
        self._state_reader = session

        self._create_state_timer()

        self.get_logger().info(
            "REAL XHand READ-ONLY state source enabled"
        )
        self.get_logger().info(
            f"XHand device_id={device_id}"
        )
        self.get_logger().info(
            "send_command API is absent from the read-only path"
        )

    def _publish_state_cycle(self) -> None:
        if self._state_reader is None:
            return

        try:
            sample = self._state_reader.read_fresh_state(
                force_update=False
            )
        except Exception as exc:
            self._failed_state_count += 1

            if (
                self._failed_state_count == 1
                or self._failed_state_count % 100 == 0
            ):
                self.get_logger().error(
                    "XHand state read failed; "
                    "nothing published: "
                    f"{exc}"
                )

            return

        # Critical safety property:
        # failed SDK read -> NO publication.
        if sample is None:
            self._failed_state_count += 1

            if (
                self._failed_state_count == 1
                or self._failed_state_count % 100 == 0
            ):
                self.get_logger().warning(
                    "XHand state unavailable; "
                    "nothing published"
                )

            return

        try:
            device_id = getattr(
                self._state_reader,
                "device_id",
                0,
            )

            mapped = map_hand_state(
                sample.state,
                hand_id=(
                    int(device_id)
                    if device_id is not None
                    else 0
                ),
                hand_name="xhand",
                hand_type="R",
            )

            msg = mapped_state_to_ros(
                mapped,
                stamp=self.get_clock().now().to_msg(),
                frame_id="",
            )

        except Exception as exc:
            self._failed_state_count += 1

            self.get_logger().error(
                "XHand state mapping failed; "
                "nothing published: "
                f"{exc}"
            )

            return

        self._state_pub.publish(msg)
        self._published_state_count += 1

    def stop_state_source(self) -> None:
        if self._state_timer is not None:
            self.destroy_timer(
                self._state_timer
            )
            self._state_timer = None

        # Fake path:
        # NEVER call XHandDriverCore.close() because that core's
        # real-control safety shutdown sends powerless command frames.
        if self._fake_backend is not None:
            try:
                self._fake_backend.close_device()
            finally:
                self._fake_backend = None
                self._fake_core = None

        # Real read-only path:
        # XHandReadOnlySession.close() directly closes the vendor
        # device and contains zero send_command() calls.
        if self._vendor_readonly_session is not None:
            try:
                self._vendor_readonly_session.close()
            finally:
                self._vendor_readonly_session = None
                self._vendor_readonly_backend = None

        self._state_reader = None


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = XHandROS2Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop_state_source()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
