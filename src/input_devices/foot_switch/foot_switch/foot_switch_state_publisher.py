"""Publish snapshots from the independent Linux foot-switch reader."""

from dataclasses import asdict

import rclpy
from rclpy.node import Node
from testbed_msgs.msg import FootSwitchState

from testbed_input_core.linux import FootSwitchReader


class KeyPublisher(Node):
    def __init__(self) -> None:
        super().__init__("foot_switch_node", namespace="robot1")
        self.declare_parameters(
            "",
            [
                ("publish_timer_period", 0.01),
                ("state_topic", "foot_switch/state"),
                ("device_path", ""),
                ("device_name", "HID 1a86:e026 Keyboard"),
            ],
        )
        period = float(self.get_parameter("publish_timer_period").value)
        if period <= 0:
            raise ValueError("publish_timer_period must be positive")
        self.reader = FootSwitchReader(
            path=self.get_parameter("device_path").value,
            name=self.get_parameter("device_name").value,
        )
        self.publisher = self.create_publisher(
            FootSwitchState, self.get_parameter("state_topic").value, 1
        )
        self.create_timer(period, self.callback)

    def callback(self) -> None:
        message = FootSwitchState()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, value in asdict(self.reader.read()).items():
            setattr(message, name, value)
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.reader.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
