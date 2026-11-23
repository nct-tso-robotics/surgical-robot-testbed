"""Publish calibrated camera axes at the robot model's endoscope tip."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from tso_sensorium.processing.camera_transform import (
    CameraMountCalibration,
    camera_quaternion_from_end_effector,
    load_camera_mount_calibration,
)

CAMERA_FRAME = "endoscope/camera_optical_frame"


def mount_transform(
    *,
    calibration: CameraMountCalibration,
    parent_frame: str,
    camera_frame: str,
) -> TransformStamped:
    """Orient a camera reference whose origin is the modeled endoscope tip."""
    if not math.isfinite(calibration.optical_tilt_radians):
        raise ValueError("optical_tilt_radians must be finite")
    if not parent_frame or not camera_frame or parent_frame == camera_frame:
        raise ValueError(
            "parent_frame and camera_frame must be distinct nonempty names"
        )
    rotation = camera_quaternion_from_end_effector(
        quaternion=(0.0, 0.0, 0.0, 1.0), calibration=calibration
    )
    transform = TransformStamped()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = camera_frame
    quaternion = transform.transform.rotation
    quaternion.x, quaternion.y, quaternion.z, quaternion.w = map(float, rotation)
    return transform


class CameraMountNode(Node):
    """Publish one static tip-to-camera rotation from the existing calibration."""

    def __init__(self) -> None:
        super().__init__("camera_mount", namespace="robot2")
        calibration_path = self.declare_parameter("calibration_path", "").value
        if not calibration_path:
            raise ValueError("calibration_path is required")
        namespace = self.get_namespace().strip("/")
        parent_default = f"{namespace}/tool_tip" if namespace else "tool_tip"
        parent_frame = self.declare_parameter("parent_frame", parent_default).value
        camera_frame = self.declare_parameter("camera_frame", CAMERA_FRAME).value
        transform = mount_transform(
            calibration=load_camera_mount_calibration(
                calibration_path=calibration_path
            ),
            parent_frame=parent_frame,
            camera_frame=camera_frame,
        )
        transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)


def main() -> None:
    """Keep the calibrated static transform available to late subscribers."""
    rclpy.init()
    node = CameraMountNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
