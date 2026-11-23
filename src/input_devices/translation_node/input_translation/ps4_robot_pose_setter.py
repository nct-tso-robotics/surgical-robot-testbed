"""Translate gamepad snapshots to robot delta commands."""

import rclpy
from testbed_msgs.msg import Ps4State

from input_translation.teleop import TeleopNode, spin_node


class Ps4RobotPoseSetter(TeleopNode):
    def __init__(self) -> None:
        super().__init__("ps4_pose_setter")
        self.declare_parameter("ps4_state_topic", "ps4_controller/state")
        self.create_subscription(
            Ps4State, self.value("ps4_state_topic"), self.input_device_callback, 1
        )

    def input_device_callback(self, message: Ps4State) -> None:
        if not self.accept_stamp(message.header, "ps4"):
            return
        roll = message.r2_trigger - message.l2_trigger
        self.publish_input(
            (message.left_joy_x, message.left_joy_y, message.right_joy_y),
            roll,
            (roll, message.left_joy_x, message.left_joy_y),
            message.right_joy_y,
            stamp=message.header.stamp,
        )
        self.publish_gripper(bool(message.r2), stamp=message.header.stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    spin_node(Ps4RobotPoseSetter())
