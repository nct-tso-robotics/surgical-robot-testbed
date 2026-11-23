"""Publish language selected from Sensorium's shared phase legends."""

import argparse
import math
import random
import select
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from testbed_msgs.msg import LanguageInstruction
from tso_sensorium.episodes.legend import load_phase_instructions


class LanguagePublisher(Node):
    def __init__(
        self, topic: str = "/language_instruction", frequency: float = 30.0
    ) -> None:
        super().__init__("language_phase_publisher")
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("frequency must be finite and positive")
        self.instruction = None
        self.publisher = self.create_publisher(
            LanguageInstruction,
            topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(1 / frequency, self.publish_instruction)

    def publish_instruction(self) -> None:
        if self.instruction is not None:
            message = LanguageInstruction()
            message.header.stamp = self.get_clock().now().to_msg()
            message.instruction = self.instruction
            self.publisher.publish(message)

    def update(self, instruction: str) -> None:
        self.instruction = instruction
        self.publish_instruction()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config_path", default="package://instructions/endoscope_guidance.yaml"
    )
    parser.add_argument("--topic", default="/language_instruction")
    parser.add_argument("--frequency", type=float, default=30.0)
    arguments, ros_arguments = parser.parse_known_args(args)
    phases = load_phase_instructions(config_path=arguments.config_path)
    rclpy.init(args=ros_arguments)
    node = LanguagePublisher(arguments.topic, arguments.frequency)
    node.get_logger().info(
        f"Phase labels: {sorted(phases)}; enter a label or custom instruction"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                text = line.strip()
                if text:
                    node.update(
                        random.choice(phases[int(text)])
                        if text.isdigit() and int(text) in phases
                        else text
                    )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
