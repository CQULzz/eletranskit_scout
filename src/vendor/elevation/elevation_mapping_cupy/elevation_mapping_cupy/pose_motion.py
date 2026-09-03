"""Pose-motion estimation used to gate elevation-map Z-drift compensation."""

from dataclasses import dataclass
from typing import Dict, Hashable, Tuple

import numpy as np


@dataclass
class _PoseMotionState:
    translation: np.ndarray
    rotation: np.ndarray
    position_motion: float = 0.0
    orientation_motion: float = 0.0


class PoseMotionDetector:
    """Estimate smoothed frame-to-frame translation and rotation magnitudes.

    State is kept per point-cloud source so alternating sensors with different
    extrinsics cannot create a false movement event.  Rotation is measured on
    SO(3), so quaternion sign changes do not affect the result.
    """

    def __init__(self, position_alpha: float, orientation_alpha: float):
        self.position_alpha = self._validate_alpha(
            "position_alpha", position_alpha
        )
        self.orientation_alpha = self._validate_alpha(
            "orientation_alpha", orientation_alpha
        )
        self._states: Dict[Hashable, _PoseMotionState] = {}

    @staticmethod
    def _validate_alpha(name: str, value: float) -> float:
        value = float(value)
        if not np.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError(f"{name} must be finite and in (0, 1], got {value}.")
        return value

    @staticmethod
    def _validated_pose(
        translation: np.ndarray, rotation: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        translation = np.asarray(translation, dtype=np.float64)
        rotation = np.asarray(rotation, dtype=np.float64)
        if translation.shape != (3,):
            raise ValueError(
                f"translation must have shape (3,), got {translation.shape}."
            )
        if rotation.shape != (3, 3):
            raise ValueError(f"rotation must have shape (3, 3), got {rotation.shape}.")
        if not np.isfinite(translation).all() or not np.isfinite(rotation).all():
            raise ValueError("Pose contains a non-finite value.")
        return translation, rotation

    def update(
        self,
        source: Hashable,
        translation: np.ndarray,
        rotation: np.ndarray,
    ) -> Tuple[float, float]:
        """Return EMA-filtered translation [m] and rotation [rad] increments."""
        translation, rotation = self._validated_pose(translation, rotation)
        previous = self._states.get(source)
        if previous is None:
            self._states[source] = _PoseMotionState(
                translation=translation.copy(), rotation=rotation.copy()
            )
            return 0.0, 0.0

        position_delta = float(np.linalg.norm(translation - previous.translation))
        relative_rotation = previous.rotation.T @ rotation
        cosine = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
        orientation_delta = float(np.arccos(cosine))

        position_motion = (
            (1.0 - self.position_alpha) * previous.position_motion
            + self.position_alpha * position_delta
        )
        orientation_motion = (
            (1.0 - self.orientation_alpha) * previous.orientation_motion
            + self.orientation_alpha * orientation_delta
        )
        self._states[source] = _PoseMotionState(
            translation=translation.copy(),
            rotation=rotation.copy(),
            position_motion=position_motion,
            orientation_motion=orientation_motion,
        )
        return position_motion, orientation_motion

    def reset(self) -> None:
        self._states.clear()
