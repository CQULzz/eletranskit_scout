import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "elevation_mapping_cupy"
    / "pose_motion.py"
)
SPEC = importlib.util.spec_from_file_location("pose_motion_under_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
PoseMotionDetector = MODULE.PoseMotionDetector


def _yaw(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_first_pose_does_not_open_motion_gate():
    detector = PoseMotionDetector(0.2, 0.2)

    assert detector.update("lidar", np.zeros(3), np.eye(3)) == (0.0, 0.0)


def test_motion_is_lowpass_filtered_in_metres_and_radians():
    detector = PoseMotionDetector(0.2, 0.2)
    detector.update("lidar", np.zeros(3), np.eye(3))

    position, orientation = detector.update(
        "lidar", np.asarray([0.1, 0.0, 0.0]), _yaw(0.1)
    )

    assert position == pytest.approx(0.02)
    assert orientation == pytest.approx(0.02)


def test_stationary_motion_decays_instead_of_retriggering_raw_jump():
    detector = PoseMotionDetector(0.2, 0.2)
    detector.update("lidar", np.zeros(3), np.eye(3))
    detector.update("lidar", np.asarray([0.1, 0.0, 0.0]), _yaw(0.1))

    position, orientation = detector.update(
        "lidar", np.asarray([0.1, 0.0, 0.0]), _yaw(0.1)
    )

    assert position == pytest.approx(0.016)
    assert orientation == pytest.approx(0.016)


def test_sources_have_independent_pose_histories():
    detector = PoseMotionDetector(1.0, 1.0)
    detector.update("front", np.zeros(3), np.eye(3))

    assert detector.update("rear", np.ones(3), _yaw(math.pi)) == (0.0, 0.0)


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.1, float("nan")])
def test_invalid_lowpass_alpha_is_rejected(alpha):
    with pytest.raises(ValueError, match="in \\(0, 1\\]"):
        PoseMotionDetector(alpha, 0.2)
