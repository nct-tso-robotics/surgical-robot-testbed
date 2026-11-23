"""Publish snapshots from the independent Linux gamepad reader."""

from dataclasses import asdict

import rclpy
from rclpy.node import Node
from testbed_msgs.msg import Ps4State

from testbed_input_core.linux import GamepadReader


class PS4StatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("ps4_state_publisher", namespace="robot1")
        self.declare_parameters(
            "",
            [
                ("controller_id", 0),
                ("publish_rate", 30.0),
                ("state_topic", "ps4_controller/state"),
                ("device_path", ""),
                ("device_name", ""),
            ],
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0:
            raise ValueError("publish_rate must be positive")
        self.controller_id = int(self.get_parameter("controller_id").value)
        self.reader = GamepadReader(
            path=self.get_parameter("device_path").value,
            name=self.get_parameter("device_name").value,
            index=self.controller_id,
        )
        self.publisher = self.create_publisher(
            Ps4State, self.get_parameter("state_topic").value, 1
        )
        self.create_timer(1 / rate, self.callback)

    def callback(self) -> None:
        state = self.reader.read()
        message = Ps4State()
        message.header.stamp = self.get_clock().now().to_msg()
        message.controller_id = self.controller_id
        for name, value in asdict(state).items():
            setattr(message, name, value)
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.reader.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PS4StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
