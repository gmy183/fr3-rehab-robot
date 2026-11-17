import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMonitor(Node):
    """Read-only monitor. It creates no command publisher."""

    def __init__(self):
        super().__init__("rehab_topic_monitor")
        self.declare_parameter("topic", "/joint_states")
        topic = self.get_parameter("topic").value
        self._count = 0
        self._start = time.monotonic()
        self._last_report = self._start
        self.create_subscription(JointState, topic, self._callback, 100)
        self.get_logger().info(f"read-only monitoring: {topic}")

    def _callback(self, _msg):
        self._count += 1
        now = time.monotonic()
        if now - self._last_report >= 5.0:
            elapsed = now - self._start
            rate = self._count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(f"messages={self._count}, average_rate={rate:.2f} Hz")
            self._last_report = now


def main():
    rclpy.init()
    node = JointStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
