"""ROS 2 wiring around the middleware-independent motion mapper."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
import uuid

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    LookupException,
    ConnectivityException,
)
from testbed_msgs.msg import KoalaCommand, RobotState, TargetTipPose
from testbed_msgs.srv import KoalaCommand as KoalaService

from testbed_input_core.motion import (
    MotionConfig,
    MotionMapper,
    Quaternion,
    RobotSnapshot,
    TipDelta,
    camera_to_base_rotation,
)


STATE_HISTORY_LENGTH = 128


@dataclass(frozen=True)
class ReceivedRobotState:
    snapshot: RobotSnapshot
    stamp_nanoseconds: int
    received_monotonic: float


def point_values(point) -> tuple[float, float, float]:
    return (point.x, point.y, point.z)


class TeleopNode(Node):
    """Convert state messages to snapshots and publish validated delta commands."""

    def __init__(
        self,
        name: str,
        position_scaling: float = 1000.0,
        rotation_scaling: float = 100.0,
    ) -> None:
        super().__init__(name, namespace="robot1")
        namespace = self.get_namespace().strip("/")
        default_base_frame = f"{namespace}/base_link" if namespace else "base_link"
        self.declare_parameters(
            "",
            [
                ("control_mode", "tip_pose"),
                ("input_position_scaling", position_scaling),
                ("input_rotation_scaling", rotation_scaling),
                ("movement_time", 0.5),
                ("safety_limits_xyz", [0.05, 0.05, 0.05]),
                ("safety_roll_limit", 0.2),
                ("robot_state_topic", "robot_state"),
                ("robot_pose_topic", "set_tip_pose"),
                ("robot_base_frame", default_base_frame),
                ("camera_frame", "endoscope/camera_optical_frame"),
                ("camera_coordinates_control", False),
                ("state_timeout", 0.5),
                ("future_stamp_tolerance", 0.1),
                ("use_koala_grasp", False),
                ("gripper_transport", "service"),
                ("koala_command_topic", "/koala_controller/command"),
                ("gripper_command_topic", "/koala_grasper/open"),
            ],
        )
        self.mapper = MotionMapper(
            MotionConfig(
                position_scaling=float(self.value("input_position_scaling")),
                rotation_scaling=float(self.value("input_rotation_scaling")),
                movement_time=float(self.value("movement_time")),
                maximum_delta_xyz=tuple(self.value("safety_limits_xyz")),
                maximum_delta_roll=float(self.value("safety_roll_limit")),
            )
        )
        if self.value("control_mode") not in ("tip_pose", "pivot_rpy"):
            raise ValueError("control_mode must be tip_pose or pivot_rpy")
        if (
            not math.isfinite(self.value("state_timeout"))
            or self.value("state_timeout") <= 0
        ):
            raise ValueError("state_timeout must be finite and positive")
        if (
            not math.isfinite(self.value("future_stamp_tolerance"))
            or self.value("future_stamp_tolerance") < 0
        ):
            raise ValueError("future_stamp_tolerance must be finite and nonnegative")
        for frame_parameter in ("robot_base_frame", "camera_frame"):
            frame = self.value(frame_parameter)
            if not frame or frame.startswith("/"):
                raise ValueError(
                    f"{frame_parameter} must be a nonempty TF frame without a leading slash"
                )
        self.source_stamps: dict[str, int] = {}
        self._state_lock = threading.Lock()
        self._state_samples: deque[ReceivedRobotState] = deque(
            maxlen=STATE_HISTORY_LENGTH
        )
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=max(2.0, self.value("state_timeout"))),
            node=self,
        )
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.source_id = f"{name}-{uuid.uuid4()}"
        self.sequence = 0
        self.robot_pose_publisher = self.create_publisher(
            TargetTipPose, self.value("robot_pose_topic"), 1
        )
        self.create_subscription(
            RobotState, self.value("robot_state_topic"), self.robot_state_callback, 1
        )
        self.gripper_publisher = None
        self.gripper_client = None
        self.gripper_future = None
        self.last_gripper_command = None
        if self.value("use_koala_grasp"):
            if self.value("gripper_transport") == "topic":
                self.gripper_publisher = self.create_publisher(
                    KoalaCommand, self.value("gripper_command_topic"), 1
                )
            elif self.value("gripper_transport") == "service":
                self.gripper_client = self.create_client(
                    KoalaService, self.value("koala_command_topic")
                )
            else:
                raise ValueError("gripper_transport must be service or topic")

    def value(self, name: str):
        return self.get_parameter(name).value

    def fresh_stamp(self, stamp: int) -> bool:
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -self.value("future_stamp_tolerance") <= age <= self.value(
            "state_timeout"
        )

    def accept_stamp(self, header: Header, source: str) -> bool:
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        if not self.fresh_stamp(stamp) or stamp <= self.source_stamps.get(source, 0):
            self.get_logger().warning(
                f"Discarding stale, duplicate or out-of-order {source}",
                throttle_duration_sec=1.0,
            )
            return False
        self.source_stamps[source] = stamp
        return True

    def valid_base_frame(self, frame: str) -> bool:
        return frame == self.value("robot_base_frame")

    def robot_state_callback(self, message: RobotState) -> None:
        if not self.valid_base_frame(message.header.frame_id):
            self.get_logger().warning(
                f"RobotState frame {message.header.frame_id!r} does not match {self.value('robot_base_frame')!r}",
                throttle_duration_sec=1.0,
            )
            return
        try:
            state = RobotSnapshot(
                relative_tip_position=point_values(message.relative_tip_position),
                relative_pivot_rpy=point_values(message.relative_pivot_rpy),
                initial_tip_position=point_values(
                    message.robot_space_initial_tip_position
                ),
                rcm_position=point_values(message.robot_space_rcm_position),
                pivot_translation=message.relative_pivot_translation,
            )
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=1.0)
            return
        if self.accept_stamp(message.header, "robot_state"):
            sample = ReceivedRobotState(
                snapshot=state,
                stamp_nanoseconds=self.source_stamps["robot_state"],
                received_monotonic=time.monotonic(),
            )
            with self._state_lock:
                self._state_samples.append(sample)

    def _sample_is_fresh(self, sample: ReceivedRobotState) -> bool:
        return self.fresh_stamp(sample.stamp_nanoseconds) and (
            time.monotonic() - sample.received_monotonic <= self.value("state_timeout")
        )

    def require_state_sample(self) -> ReceivedRobotState:
        with self._state_lock:
            sample = self._state_samples[-1] if self._state_samples else None
        if sample is None or not self._sample_is_fresh(sample):
            raise ValueError("Robot state unavailable or stale")
        return sample

    def require_state(self) -> RobotSnapshot:
        return self.require_state_sample().snapshot

    def require_camera_state(self) -> tuple[ReceivedRobotState, Quaternion]:
        """Resolve the newest fresh state with exact-time TF, without waiting."""
        with self._state_lock:
            samples = tuple(self._state_samples)
        target_frame = self.value("robot_base_frame")
        source_frame = self.value("camera_frame")
        latest_error = "no fresh robot state"
        for sample in reversed(samples):
            if not self._sample_is_fresh(sample):
                continue
            stamp = Time(
                nanoseconds=sample.stamp_nanoseconds,
                clock_type=self.get_clock().clock_type,
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame=target_frame,
                    source_frame=source_frame,
                    time=stamp,
                    timeout=Duration(),
                )
            except (LookupException, ConnectivityException) as error:
                latest_error = str(error)
                break
            except TransformException as error:
                latest_error = str(error)
                continue
            rotation = transform.transform.rotation
            quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
            camera_to_base_rotation(quaternion)
            return sample, quaternion
        raise ValueError(
            f"TF {source_frame!r} -> {target_frame!r} is unavailable at a fresh RobotState timestamp: {latest_error}"
        )

    def publish_delta(
        self, command: TipDelta, stamp: TimeMessage | None = None
    ) -> None:
        message = TargetTipPose()
        message.header.stamp = (
            stamp if stamp is not None else self.get_clock().now().to_msg()
        )
        message.header.frame_id = self.value("robot_base_frame")
        self.sequence += 1
        message.source_id = self.source_id
        message.sequence = self.sequence
        (
            message.target_delta_position.x,
            message.target_delta_position.y,
            message.target_delta_position.z,
        ) = command.position
        message.target_delta_roll = command.roll
        message.movement_time = command.movement_time
        self.robot_pose_publisher.publish(message)

    def publish_input(
        self, position, roll: float, pivot_rpy, insertion: float, stamp: TimeMessage
    ) -> None:
        try:
            if self.value("control_mode") == "pivot_rpy":
                command = self.mapper.pivot(
                    self.require_state(), tuple(pivot_rpy), insertion
                )
            else:
                if self.value("camera_coordinates_control"):
                    _, camera = self.require_camera_state()
                else:
                    self.require_state()
                    camera = None
                command = self.mapper.delta(
                    tuple(position), roll, camera_quaternion=camera
                )
            self.publish_delta(command, stamp=stamp)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=1.0)

    def publish_gripper(self, opened: bool, stamp: TimeMessage | None = None) -> None:
        if self.gripper_publisher is not None:
            message = KoalaCommand()
            message.header.stamp = (
                stamp if stamp is not None else self.get_clock().now().to_msg()
            )
            message.open = opened
            self.gripper_publisher.publish(message)
        elif self.gripper_client is not None:
            if self.gripper_future is not None and not self.gripper_future.done():
                return
            if (
                not self.gripper_client.service_is_ready()
                or opened == self.last_gripper_command
            ):
                return
            request = KoalaService.Request()
            request.command = "open" if opened else "close"
            self.last_gripper_command = opened
            self.gripper_future = self.gripper_client.call_async(request)
            self.gripper_future.add_done_callback(self.gripper_response)

    def gripper_response(self, future) -> None:
        if future.exception() is not None or not future.result().success:
            self.last_gripper_command = None

    def destroy_node(self) -> bool:
        self.tf_listener.unregister()
        return super().destroy_node()


def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
