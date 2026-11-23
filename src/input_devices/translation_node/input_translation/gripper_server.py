"""Direct gripper commands and command-intent state for the Koala browser bridge.

State reports the latest requested command, even without a browser connection.
It is neither browser delivery confirmation nor measured physical feedback.
"""

import math

import rclpy
from rclpy.node import Node
from testbed_msgs.msg import KoalaCommand

from input_translation.teleop import spin_node
from testbed_input_core.gripper import GripperBridge


class GripperNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_websocket_bridge")
        self.declare_parameters(
            "",
            [
                ("host", "127.0.0.1"),
                ("port", 6789),
                ("command_topic", "/koala_grasper/open"),
                ("state_topic", "/koala_grasper/state"),
                ("publish_rate", 10.0),
            ],
        )
        rate = float(self.get_parameter("publish_rate").value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("publish_rate must be finite and positive")
        self.bridge = GripperBridge(
            self.get_parameter("host").value, self.get_parameter("port").value
        )
        self.bridge.start()
        self.publisher = self.create_publisher(
            KoalaCommand, self.get_parameter("state_topic").value, 1
        )
        self.create_subscription(
            KoalaCommand,
            self.get_parameter("command_topic").value,
            self.command_callback,
            1,
        )
        self.create_timer(1 / rate, self.publish_state)
        self.get_logger().info(
            f"Gripper server listening on ws://{self.bridge.host}:{self.bridge.port}"
        )
        self.get_logger().info(
            f"DIRECT command mode on {self.get_parameter('command_topic').value}: "
            "True -> OPEN (1), False -> CLOSE (0); initial request CLOSE"
        )
        self.get_logger().info(
            f"Publishing latest requested command on {self.publisher.topic_name} "
            f"at {rate:g} Hz, including while the browser is disconnected; "
            "this is command intent, not physical feedback"
        )

    def command_callback(self, message: KoalaCommand) -> None:
        previous = self.bridge.requested_state()
        self.bridge.command(message.open)
        if message.open != previous:
            self.get_logger().info(
                f"Requested gripper {'OPEN (1)' if message.open else 'CLOSE (0)'}"
            )

    def publish_state(self) -> None:
        message = KoalaCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.open = self.bridge.requested_state()
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.bridge.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    spin_node(GripperNode())
