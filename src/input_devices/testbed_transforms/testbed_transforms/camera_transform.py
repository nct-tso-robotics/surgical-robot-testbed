"""Record the camera TF pair at each accepted robot acquisition timestamp."""

from __future__ import annotations

from collections import deque
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from testbed_msgs.msg import RobotState
from tf2_ros import Buffer, TransformException, TransformListener

from testbed_transforms.camera_mount import CAMERA_FRAME

NANOSECONDS_PER_SECOND = 1_000_000_000


class CameraTransformNode(Node):
    """Bridge a connected TF tree to a timestamped scalar recording stream."""

    def __init__(self) -> None:
        super().__init__("camera_transform", namespace="robot1")
        namespace = self.get_namespace().strip("/")
        base_default = f"{namespace}/base_link" if namespace else "base_link"
        self.target_frame = self.declare_parameter("target_frame", base_default).value
        self.camera_frame = self.declare_parameter("camera_frame", CAMERA_FRAME).value
        if not self.target_frame or not self.camera_frame:
            raise ValueError("target_frame and camera_frame must be nonempty")
        source_topic = self.declare_parameter("robot_state_topic", "robot_state").value
        target_topic = self.declare_parameter(
            "transform_topic", "camera_transform"
        ).value
        rate_hz = float(self.declare_parameter("rate_hz", 30.0).value)
        max_age_seconds = float(self.declare_parameter("max_age_seconds", 0.25).value)
        pending_limit = self.declare_parameter("pending_limit", 32).value
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("rate_hz must be finite and positive")
        if not math.isfinite(max_age_seconds) or max_age_seconds <= 0:
            raise ValueError("max_age_seconds must be finite and positive")
        if pending_limit <= 0:
            raise ValueError("pending_limit must be positive")
        self.period_ns = max(1, round(NANOSECONDS_PER_SECOND / rate_hz))
        self.max_age_ns = round(max_age_seconds * NANOSECONDS_PER_SECOND)
        self.pending: deque[Time] = deque(maxlen=pending_limit)
        self.last_seen_stamp = 0
        self.last_queued_stamp = 0
        self.last_clock = self.get_clock().now().nanoseconds
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(TransformStamped, target_topic, 10)
        self.subscription = self.create_subscription(
            RobotState, source_topic, self.receive_state, qos_profile_sensor_data
        )
        self.timer = self.create_timer(min(0.01, 1.0 / rate_hz), self.publish_pending)

    def current_time(self) -> int:
        """Discard pending history when a simulation clock jumps backwards."""
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.pending.clear()
            self.last_seen_stamp = 0
            self.last_queued_stamp = 0
        self.last_clock = now
        return now

    def receive_state(self, state: RobotState) -> None:
        """Queue fresh source stamps while TF callbacks arrive independently."""
        stamp = Time.from_msg(state.header.stamp)
        now = self.current_time()
        age = now - stamp.nanoseconds
        if (
            state.header.frame_id != self.target_frame
            or stamp.nanoseconds <= self.last_seen_stamp
            or age < 0
            or age > self.max_age_ns
        ):
            return
        self.last_seen_stamp = stamp.nanoseconds
        if stamp.nanoseconds - self.last_queued_stamp < self.period_ns:
            return
        self.pending.append(stamp)
        self.last_queued_stamp = stamp.nanoseconds
        self.publish_pending()

    def publish_pending(self) -> None:
        """Publish exact-time TF results; expire missing data without blocking."""
        now = self.current_time()
        while self.pending:
            stamp = self.pending[0]
            age = now - stamp.nanoseconds
            if age < 0 or age > self.max_age_ns:
                self.pending.popleft()
                continue
            try:
                transform = self.buffer.lookup_transform(
                    self.target_frame, self.camera_frame, stamp
                )
            except TransformException:
                return
            self.pending.popleft()
            # Static-only TF queries may return a zero stamp; this is the sample time.
            transform.header.stamp = stamp.to_msg()
            self.publisher.publish(transform)


def main() -> None:
    """Run the per-robot recording bridge alongside the shared TF tree."""
    rclpy.init()
    node = CameraTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
