"""Translate Lambda state while its configured clutch pedal is held."""

import time

import rclpy
from testbed_msgs.msg import LambdaState, PedalBoardState

from input_translation.teleop import TeleopNode, spin_node


class LambdaRobotPoseSetter(TeleopNode):
    def __init__(self) -> None:
        super().__init__(
            "lambda_pose_setter", position_scaling=100.0, rotation_scaling=50.0
        )
        self.declare_parameters(
            "",
            [
                ("lambda_state_topic", "lambda/state"),
                ("pedal_board_topic", "pedal_board/state"),
                ("clutch_pedal", "top_grey_pedal"),
                ("require_clutch", True),
            ],
        )
        self.clutch_pressed = False
        self.clutch_received = 0.0
        self.create_subscription(
            LambdaState, self.value("lambda_state_topic"), self.input_device_callback, 1
        )
        self.create_subscription(
            PedalBoardState,
            self.value("pedal_board_topic"),
            self.pedal_board_state_callback,
            1,
        )

    def pedal_board_state_callback(self, message: PedalBoardState) -> None:
        if not self.accept_stamp(message.header, "pedal"):
            return
        self.clutch_pressed = bool(getattr(message, self.value("clutch_pedal")))
        self.clutch_received = time.monotonic()

    def input_device_callback(self, message: LambdaState) -> None:
        if not self.accept_stamp(message.header, "lambda"):
            return
        if self.value("require_clutch") and (
            not self.clutch_pressed
            or not self.fresh_stamp(self.source_stamps.get("pedal", 0))
            or time.monotonic() - self.clutch_received > self.value("state_timeout")
        ):
            return
        if len(message.joints.velocity) < 6:
            self.get_logger().warning(
                "Lambda wrist velocity is unavailable", throttle_duration_sec=1.0
            )
            return
        velocity = message.twist.linear
        roll = message.joints.velocity[5]
        self.publish_input(
            (velocity.y, -velocity.z, -velocity.x),
            roll,
            (roll / 10.0, -velocity.y, -velocity.z),
            -velocity.x,
            stamp=message.header.stamp,
        )
        self.publish_gripper(
            abs(message.gripper_angle) > 0.01, stamp=message.header.stamp
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    spin_node(LambdaRobotPoseSetter())
