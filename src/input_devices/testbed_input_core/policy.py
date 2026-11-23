"""Policy protocol conversion and Sensorium image preparation without ROS."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
from tso_robotics_sockets import CompressionType, compress_array
from tso_robotics_sockets.messages import (
    InferenceRequestKey,
    InferenceResponseKey,
    ServerStatus,
    StereoRequestKey,
    StereoResponseKey,
    TransportKey,
)
from tso_sensorium.processing.rectification import Rectifier
from tso_sensorium.resources import resolve_asset_path
from versatil_constants.shared import (
    ActionComponent,
    ActionComputationMethod,
    ActionMetadataField,
    CoordinateSystem,
    ObsKey,
    OrientationRepresentation,
)
from versatil_constants.tso import TSOCamera, TSOProprioKey

from testbed_input_core.motion import (
    MotionMapper,
    Quaternion,
    RobotSnapshot,
    TipDelta,
    camera_to_base_rotation,
)


@dataclass(frozen=True)
class StereoConfig:
    calibration_path: str = "package://storz_endoscope_calibration.yml"
    rectify: bool = False
    left_crop: tuple[int, int, int, int] | None = None
    right_crop: tuple[int, int, int, int] | None = None


class StereoProcessor:
    """Use the same rectifier and calibration assets as offline Sensorium jobs."""

    def __init__(self, config: StereoConfig) -> None:
        self.config = config
        self._rectifier: Rectifier | None = None

    def rectify(
        self, left: np.ndarray, right: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        if self._rectifier is None:
            self._rectifier = Rectifier(
                resolve_asset_path(self.config.calibration_path)
            )
        expected = self._rectifier.left_map_x.shape
        if left.shape[:2] != expected or right.shape[:2] != expected:
            raise ValueError(
                f"Stereo images must match calibration dimensions {expected}"
            )
        return self._rectifier.rectify(left=left, right=right)

    @staticmethod
    def crop(image: np.ndarray, bounds: tuple[int, int, int, int] | None) -> np.ndarray:
        if bounds is None:
            return image
        x0, y0, x1, y1 = bounds
        if not (0 <= x0 < x1 <= image.shape[1] and 0 <= y0 < y1 <= image.shape[0]):
            raise ValueError(f"Crop {bounds} outside image {image.shape}")
        return image[y0:y1, x0:x1].copy()

    def prepare(
        self, left: np.ndarray, right: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        if self.config.rectify:
            left, right = self.rectify(left, right)
        return self.crop(left, self.config.left_crop), self.crop(
            right, self.config.right_crop
        )


def parse_action(
    payload: dict[str, Any],
    state: RobotSnapshot,
    mapper: MotionMapper,
    camera_quaternion: Quaternion | None,
) -> tuple[TipDelta, bool | None]:
    """Validate one environment's action and convert to the refactor delta contract."""
    environments = payload[InferenceRequestKey.ACTIONS.value]
    if not isinstance(environments, dict) or len(environments) != 1:
        raise ValueError("Exactly one environment action is required")
    actions = next(iter(environments.values()))
    metadata = payload.get(InferenceRequestKey.ACTION_METADATA.value, {})
    position_meta = metadata.get(ActionComponent.POSITION.value, {})
    orientation_meta = metadata.get(ActionComponent.ORIENTATION.value, {})
    frame = position_meta.get(
        ActionMetadataField.FRAME.value, CoordinateSystem.ROBOT_BASE.value
    )
    if frame not in (CoordinateSystem.ROBOT_BASE.value, CoordinateSystem.CAMERA.value):
        raise ValueError(f"Unsupported action coordinate frame: {frame}")
    if frame == CoordinateSystem.CAMERA.value and camera_quaternion is None:
        raise ValueError("Camera transform unavailable")
    representation = orientation_meta.get(
        ActionMetadataField.ORIENTATION_REPRESENTATION.value,
        OrientationRepresentation.ROLL.value,
    )
    if representation != OrientationRepresentation.ROLL.value:
        raise ValueError("Only roll orientation is supported")
    position_type = position_meta.get(
        ActionMetadataField.ACTION_TYPE.value, ActionComputationMethod.DELTA.value
    )
    orientation_type = orientation_meta.get(
        ActionMetadataField.ACTION_TYPE.value, position_type
    )
    for action_type in (position_type, orientation_type):
        if action_type not in (
            ActionComputationMethod.DELTA.value,
            ActionComputationMethod.NEXT_TIMESTEP.value,
        ):
            raise ValueError(f"Unsupported action type: {action_type}")
    orientation = np.asarray(
        actions.get(ActionComponent.ORIENTATION.value, [0.0]), dtype=float
    )
    if orientation.shape != (1,):
        raise ValueError("Roll orientation must contain exactly one value")
    command = mapper.policy(
        position=actions[ActionComponent.POSITION.value],
        roll=float(orientation[0]),
        state=state,
        position_absolute=position_type == ActionComputationMethod.NEXT_TIMESTEP.value,
        roll_absolute=ActionComponent.ORIENTATION.value in actions
        and orientation_type == ActionComputationMethod.NEXT_TIMESTEP.value,
        camera_quaternion=camera_quaternion
        if frame == CoordinateSystem.CAMERA.value
        else None,
    )
    gripper = None
    if ActionComponent.GRIPPER.value in actions:
        values = actions[ActionComponent.GRIPPER.value]
        if len(values) != 1 or values[0] not in (0, 1):
            raise ValueError("Binary gripper action must contain zero or one")
        gripper = bool(values[0])
    return command, gripper


def observation_payload(
    flags: dict[str, Any],
    state: RobotSnapshot,
    left: np.ndarray | None,
    right: np.ndarray | None,
    camera_quaternion: Quaternion | None,
    gripper: bool | None,
    language: str | None,
    processor: StereoProcessor,
    stereo_request: Callable[[dict[str, Any]], dict[str, Any]],
) -> dict[str, Any]:
    requested = set(flags.get(InferenceRequestKey.REQUESTED_KEYS.value, []))
    compression = flags.get(
        InferenceRequestKey.COMPRESSION_TYPE.value, CompressionType.RAW.value
    )
    payload = {
        TransportKey.STATUS.value: ServerStatus.WAITING_ACTION.value,
        InferenceResponseKey.COMPRESSION_TYPE.value: compression,
    }
    camera_keys = {camera.value for camera in TSOCamera}
    if requested & camera_keys:
        if left is None or right is None:
            raise ValueError("Stereo images unavailable")
        want_stereo = requested & {
            TSOCamera.DEPTH.value,
            TSOCamera.DISPARITY_MAP.value,
            TSOCamera.POINT_CLOUD.value,
        }
        if want_stereo:
            rectified_left, rectified_right = processor.rectify(left, right)
            response = stereo_request(
                {
                    StereoRequestKey.LEFT_IMG.value: compress_array(
                        rectified_left, method=CompressionType.RAW.value, as_base64=True
                    ),
                    StereoRequestKey.RIGHT_IMG.value: compress_array(
                        rectified_right,
                        method=CompressionType.RAW.value,
                        as_base64=True,
                    ),
                    StereoRequestKey.REQUEST_DISPARITY.value: TSOCamera.DISPARITY_MAP.value
                    in requested,
                    StereoRequestKey.REQUEST_DEPTH.value: TSOCamera.DEPTH.value
                    in requested,
                    StereoRequestKey.REQUEST_POINT_CLOUD.value: TSOCamera.POINT_CLOUD.value
                    in requested,
                    StereoRequestKey.COMPRESSION_TYPE.value: CompressionType.RAW.value,
                }
            )
            if TransportKey.ERROR_MSG.value in response:
                raise ValueError(str(response[TransportKey.ERROR_MSG.value]))
            for camera, field in (
                (TSOCamera.DEPTH, StereoResponseKey.DEPTH_MAP),
                (TSOCamera.DISPARITY_MAP, StereoResponseKey.DISPARITY_MAP),
                (TSOCamera.POINT_CLOUD, StereoResponseKey.POINT_CLOUD),
            ):
                if camera.value in requested:
                    payload[camera.value] = response[field.value]
        left, right = processor.prepare(left, right)
        for camera, frame in ((TSOCamera.LEFT, left), (TSOCamera.RIGHT, right)):
            if camera.value in requested:
                payload[camera.value] = compress_array(
                    frame, method=compression, as_base64=True
                )
    if TSOProprioKey.ROBOT_FRAME_CARTESIAN_TIP_POS.value in requested:
        payload[TSOProprioKey.ROBOT_FRAME_CARTESIAN_TIP_POS.value] = list(
            state.relative_tip_position
        )
    if TSOProprioKey.CAMERA_FRAME_CARTESIAN_TIP_POS.value in requested:
        if camera_quaternion is None:
            raise ValueError("Camera transform unavailable")
        payload[TSOProprioKey.CAMERA_FRAME_CARTESIAN_TIP_POS.value] = (
            camera_to_base_rotation(camera_quaternion).T @ state.relative_tip_position
        ).tolist()
    if TSOProprioKey.RELATIVE_PIVOT_ROLL.value in requested:
        payload[TSOProprioKey.RELATIVE_PIVOT_ROLL.value] = [state.relative_pivot_rpy[0]]
    if TSOProprioKey.GRIPPER_STATE.value in requested:
        if gripper is None:
            raise ValueError("Gripper state unavailable")
        payload[TSOProprioKey.GRIPPER_STATE.value] = [float(gripper)]
    if ObsKey.LANGUAGE.value in requested:
        if language is None:
            raise ValueError("Language instruction unavailable")
        payload[ObsKey.LANGUAGE.value] = language
    return payload
