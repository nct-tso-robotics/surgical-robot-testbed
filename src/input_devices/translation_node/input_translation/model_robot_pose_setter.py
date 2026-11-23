"""ROS 2 observations and delta commands exposed through the policy RPC protocol."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy
import zmq
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from testbed_msgs.msg import KoalaCommand, LanguageInstruction
from tso_robotics_sockets import SocketClient, SocketServer
from tso_robotics_sockets.messages import InferenceRequestKey, ServerRoute, TransportKey
from versatil_constants.shared import (
    ActionComponent,
    ActionMetadataField,
    CoordinateSystem,
)
from versatil_constants.tso import TSOProprioKey
from tso_sensorium.processing.image import deinterlace_cv_image

from input_translation.teleop import TeleopNode, spin_node
from testbed_input_core.policy import (
    StereoConfig,
    StereoProcessor,
    observation_payload,
    parse_action,
)


def image_to_rgb(message: Image) -> np.ndarray:
    """Decode 8-bit ROS images with their declared row stride."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if message.encoding not in channels:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    count = channels[message.encoding]
    row_size = message.width * count
    if message.step < row_size or len(message.data) != message.height * message.step:
        raise ValueError("Image buffer does not match dimensions and stride")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    frame = rows[:, :row_size].reshape(message.height, message.width, count)
    conversions = {
        "bgr8": cv2.COLOR_BGR2RGB,
        "bgra8": cv2.COLOR_BGRA2RGB,
        "rgba8": cv2.COLOR_RGBA2RGB,
        "mono8": cv2.COLOR_GRAY2RGB,
    }
    return (
        frame.copy()
        if message.encoding == "rgb8"
        else cv2.cvtColor(frame, conversions[message.encoding])
    )


class ModelPoseSetter(TeleopNode):
    def __init__(self) -> None:
        super().__init__(
            "model_pose_setter", position_scaling=1.0, rotation_scaling=1.0
        )
        self.declare_parameters(
            "",
            [
                ("bind_ip", "127.0.0.1"),
                ("bind_port", 5550),
                ("depth_server_ip", "127.0.0.1"),
                ("depth_server_port", 5555),
                ("depth_timeout", 2.0),
                ("calibration_path", "package://storz_endoscope_calibration.yml"),
                ("rectify_images", False),
                ("crop_images", False),
                ("left_image_crop", [160, 0, 700, 540]),
                ("right_image_crop", [260, 0, 800, 540]),
                ("use_interlaced_image", False),
                ("left_odd", True),
                ("combined_width", 960),
                ("combined_height", 540),
                ("left_image_topic", "/endoscope/left/image_raw"),
                ("right_image_topic", "/endoscope/right/image_raw"),
                ("interlaced_image_topic", "/endoscope/capture/image_raw"),
                ("image_qos_reliability", "reliable"),
                ("maximum_stereo_skew", 0.02),
                ("gripper_state_topic", "/koala_grasper/state"),
                ("language_topic", "/language_instruction"),
            ],
        )
        reliability = self.value("image_qos_reliability")
        if reliability not in ("reliable", "best_effort"):
            raise ValueError("image_qos_reliability must be reliable or best_effort")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
            if reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_lock = threading.Lock()
        self.left = None
        self.right = None
        self.left_stamp = 0
        self.right_stamp = 0
        self.left_received = 0.0
        self.right_received = 0.0
        self.gripper = None
        self.gripper_received = 0.0
        self.language = None
        self.processor = StereoProcessor(
            StereoConfig(
                calibration_path=self.value("calibration_path"),
                rectify=self.value("rectify_images"),
                left_crop=tuple(self.value("left_image_crop"))
                if self.value("crop_images")
                else None,
                right_crop=tuple(self.value("right_image_crop"))
                if self.value("crop_images")
                else None,
            )
        )
        if self.value("use_interlaced_image"):
            self.create_subscription(
                Image, self.value("interlaced_image_topic"), self.combined_image, qos
            )
        else:
            self.create_subscription(
                Image, self.value("left_image_topic"), self.left_image, qos
            )
            self.create_subscription(
                Image, self.value("right_image_topic"), self.right_image, qos
            )
        self.create_subscription(
            KoalaCommand, self.value("gripper_state_topic"), self.gripper_state, 1
        )
        self.create_subscription(
            LanguageInstruction,
            self.value("language_topic"),
            self.language_instruction,
            1,
        )
        self.stop_event = threading.Event()
        self.socket_ready = threading.Event()
        self.socket_error = None
        self.socket_thread = threading.Thread(
            target=self.serve, name="policy-rpc", daemon=True
        )
        self.socket_thread.start()
        if not self.socket_ready.wait(timeout=5.0):
            raise RuntimeError("Policy socket server did not start")
        if self.socket_error is not None:
            raise RuntimeError(f"Policy socket server failed: {self.socket_error}")

    @staticmethod
    def stamp(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def combined_image(self, message: Image) -> None:
        if not self.accept_stamp(message.header, "stereo"):
            return
        left, right = deinterlace_cv_image(
            image_to_rgb(message), left_odd=self.value("left_odd")
        )
        size = (int(self.value("combined_width")), int(self.value("combined_height")))
        if min(size) <= 0:
            raise ValueError("Combined output dimensions must be positive")
        left = cv2.resize(left, size, interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, size, interpolation=cv2.INTER_AREA)
        with self.image_lock:
            self.left, self.right = left, right
            self.left_stamp = self.right_stamp = self.stamp(message)
            self.left_received = self.right_received = time.monotonic()

    def left_image(self, message: Image) -> None:
        if not self.accept_stamp(message.header, "left_image"):
            return
        frame = image_to_rgb(message)
        with self.image_lock:
            self.left, self.left_stamp, self.left_received = (
                frame,
                self.stamp(message),
                time.monotonic(),
            )

    def right_image(self, message: Image) -> None:
        if not self.accept_stamp(message.header, "right_image"):
            return
        frame = image_to_rgb(message)
        with self.image_lock:
            self.right, self.right_stamp, self.right_received = (
                frame,
                self.stamp(message),
                time.monotonic(),
            )

    def gripper_state(self, message: KoalaCommand) -> None:
        if not self.accept_stamp(message.header, "gripper"):
            return
        self.gripper = message.open
        self.gripper_received = time.monotonic()

    def language_instruction(self, message: LanguageInstruction) -> None:
        self.language = message.instruction

    def observe(self, flags: dict, depth_client: SocketClient) -> tuple[bool, dict]:
        try:
            requested = set(flags.get(InferenceRequestKey.REQUESTED_KEYS.value, []))
            if TSOProprioKey.CAMERA_FRAME_CARTESIAN_TIP_POS.value in requested:
                sample, camera = self.require_camera_state()
                state = sample.snapshot
            else:
                state = self.require_state()
                camera = None
            now = time.monotonic()
            with self.image_lock:
                fresh = (
                    self.fresh_stamp(self.left_stamp)
                    and self.fresh_stamp(self.right_stamp)
                    and now - min(self.left_received, self.right_received)
                    <= self.value("state_timeout")
                )
                synchronized = (
                    abs(self.left_stamp - self.right_stamp)
                    <= self.value("maximum_stereo_skew") * 1e9
                )
                left, right = (
                    (self.left, self.right) if fresh and synchronized else (None, None)
                )
            gripper = (
                self.gripper
                if self.fresh_stamp(self.source_stamps.get("gripper", 0))
                and now - self.gripper_received <= self.value("state_timeout")
                else None
            )
            result = observation_payload(
                flags,
                state,
                left,
                right,
                camera,
                gripper,
                self.language,
                self.processor,
                lambda request: depth_client.send_request(
                    ServerRoute.COMPUTE_STEREO.value, request
                ),
            )
            return True, result
        except (KeyError, TypeError, ValueError, TimeoutError) as error:
            return False, {TransportKey.ERROR_MSG.value: str(error)}

    def apply_action(self, payload: dict) -> tuple[bool, dict]:
        try:
            metadata = payload.get(InferenceRequestKey.ACTION_METADATA.value, {})
            position_metadata = metadata.get(ActionComponent.POSITION.value, {})
            if (
                position_metadata.get(ActionMetadataField.FRAME.value)
                == CoordinateSystem.CAMERA.value
            ):
                sample, camera = self.require_camera_state()
                state = sample.snapshot
            else:
                state = self.require_state()
                camera = None
            command, gripper = parse_action(payload, state, self.mapper, camera)
            self.publish_delta(command)
            if gripper is not None:
                self.publish_gripper(gripper)
            return True, {}
        except (KeyError, TypeError, ValueError) as error:
            return False, {TransportKey.ERROR_MSG.value: str(error)}

    def serve(self) -> None:
        context = zmq.Context()
        server = None
        client = None
        try:
            server = SocketServer(
                self.value("bind_ip"), self.value("bind_port"), context=context
            )
            client = SocketClient(
                self.value("depth_server_ip"),
                self.value("depth_server_port"),
                context=context,
                request_timeout_seconds=self.value("depth_timeout"),
            )
            server.add_route(
                ServerRoute.GET_OBSERVATION.value,
                lambda flags: self.observe(flags, client),
            )
            server.add_route(ServerRoute.SEND_ACTION.value, self.apply_action)
            server.add_route(
                ServerRoute.REGISTER_CLIENT.value, lambda request: (True, {})
            )
            self.socket_server = server
            self.socket_ready.set()
            server.run()
        except (zmq.ZMQError, ValueError, RuntimeError) as error:
            self.socket_error = error
            self.socket_ready.set()
        finally:
            if server is not None:
                server.executor.shutdown(wait=True)
                server.reply_socket.close()
            if client is not None:
                client.request_socket.close()
            context.term()

    def destroy_node(self) -> bool:
        if hasattr(self, "socket_server"):
            self.socket_server.stop()
        self.socket_thread.join(timeout=self.value("depth_timeout") + 1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    spin_node(ModelPoseSetter())
