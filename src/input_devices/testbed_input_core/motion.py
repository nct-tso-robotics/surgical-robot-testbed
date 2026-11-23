"""Stateless delta mapping shared by teleoperation and policy adapters."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
# Must match rcm_solvers::convertRelativeTipPositionToPYT.
PIVOT_AXIS_REFERENCE_THRESHOLD = 0.9986


def vector3(values: tuple[float, ...] | list[float] | np.ndarray) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("Expected three finite coordinates")
    return result


def camera_to_base_rotation(quaternion: Quaternion) -> np.ndarray:
    """Return the camera orientation in base coordinates (xyzw quaternion)."""
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Expected a finite xyzw quaternion")
    norm = np.linalg.norm(values)
    if norm < 1e-12:
        raise ValueError("Camera quaternion must have nonzero norm")
    x, y, z, w = values / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


@dataclass(frozen=True)
class RobotSnapshot:
    relative_tip_position: Vector3
    relative_pivot_rpy: Vector3
    initial_tip_position: Vector3 = (0.0, 0.0, 1.0)
    rcm_position: Vector3 = (0.0, 0.0, 0.0)
    pivot_translation: float = 0.0

    def __post_init__(self) -> None:
        for values in (
            self.relative_tip_position,
            self.relative_pivot_rpy,
            self.initial_tip_position,
            self.rcm_position,
        ):
            vector3(values)
        if not math.isfinite(self.pivot_translation):
            raise ValueError("Pivot translation must be finite")


@dataclass(frozen=True)
class TipDelta:
    position: Vector3
    roll: float
    movement_time: float


@dataclass(frozen=True)
class MotionConfig:
    position_scaling: float = 1000.0
    rotation_scaling: float = 100.0
    movement_time: float = 0.5
    maximum_delta_xyz: Vector3 = (0.05, 0.05, 0.05)
    maximum_delta_roll: float = 0.2

    def __post_init__(self) -> None:
        values = (self.position_scaling, self.rotation_scaling, self.movement_time)
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("Scaling and movement time must be finite and positive")
        if np.any(vector3(self.maximum_delta_xyz) < 0):
            raise ValueError("Position limits must be nonnegative")
        if not math.isfinite(self.maximum_delta_roll) or self.maximum_delta_roll < 0:
            raise ValueError("Roll limit must be finite and nonnegative")


class MotionMapper:
    """Map each input to one delta without accumulating an absolute target."""

    def __init__(self, config: MotionConfig) -> None:
        self.config = config

    def delta(
        self,
        position: Vector3 | np.ndarray,
        roll: float,
        camera_quaternion: Quaternion | None = None,
        scale: bool = True,
    ) -> TipDelta:
        delta = vector3(position)
        if not math.isfinite(roll):
            raise ValueError("Roll must be finite")
        if scale:
            delta = delta / self.config.position_scaling
            roll = roll / self.config.rotation_scaling
        if camera_quaternion is not None:
            delta = camera_to_base_rotation(camera_quaternion) @ delta
        if np.any(np.abs(delta) > vector3(self.config.maximum_delta_xyz)):
            raise ValueError("Position delta exceeds configured limits")
        if abs(roll) > self.config.maximum_delta_roll:
            raise ValueError("Roll delta exceeds configured limit")
        return TipDelta(
            tuple(float(value) for value in delta),
            float(roll),
            self.config.movement_time,
        )

    def pivot(
        self,
        state: RobotSnapshot,
        rpy: Vector3,
        insertion: float,
    ) -> TipDelta:
        angles = (
            vector3(state.relative_pivot_rpy)
            + vector3(rpy) / self.config.rotation_scaling
        )
        initial_tip = vector3(state.initial_tip_position)
        pivot = vector3(state.rcm_position)
        shaft = initial_tip - pivot
        length = np.linalg.norm(shaft)
        if length < 1e-9:
            raise ValueError("Initial tip and pivot must be distinct")
        shaft = shaft / length
        reference = (
            np.array([0.0, 0.0, 1.0])
            if abs(shaft[2]) < PIVOT_AXIS_REFERENCE_THRESHOLD
            else np.array([1.0, 0.0, 0.0])
        )
        x_axis = np.cross(shaft, reference)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(shaft, x_axis)
        tilted = (
            axis_rotation(x_axis, angles[1]) @ axis_rotation(y_axis, angles[2]) @ shaft
        )
        distance = (
            length + state.pivot_translation + insertion / self.config.position_scaling
        )
        desired_relative = pivot + distance * tilted - initial_tip
        return self.delta(
            position=desired_relative - vector3(state.relative_tip_position),
            roll=rpy[0] / self.config.rotation_scaling,
            scale=False,
        )

    def policy(
        self,
        position: Vector3,
        roll: float,
        state: RobotSnapshot,
        position_absolute: bool = False,
        roll_absolute: bool = False,
        camera_quaternion: Quaternion | None = None,
    ) -> TipDelta:
        """Convert physical-unit policy targets to base-frame deltas exactly once."""
        target = vector3(position)
        if position_absolute:
            current = vector3(state.relative_tip_position)
            if camera_quaternion is not None:
                current = camera_to_base_rotation(camera_quaternion).T @ current
            target = target - current
        if roll_absolute:
            roll -= state.relative_pivot_rpy[0]
        return self.delta(
            position=target, roll=roll, camera_quaternion=camera_quaternion, scale=False
        )
