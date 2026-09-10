import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from joint_calibration import JointCalibration

CALIBRATION_YAML = (
    Path(__file__).resolve().parents[2]
    / "army_manipulator_description"
    / "config"
    / "joint_calibration.yaml"
)


@pytest.fixture()
def calibration() -> JointCalibration:
    return JointCalibration(CALIBRATION_YAML)


def test_zero_offset_defaults_to_zero(calibration: JointCalibration):
    for joint_name in (
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_joint",
        "gripper_joint",
    ):
        assert calibration.zero_offset(joint_name) == 0.0


def test_raw_actual_roundtrip(calibration: JointCalibration):
    raw_angle = 1.2345
    actual_angle = calibration.raw_to_actual("wrist_joint", raw_angle)
    assert calibration.actual_to_raw("wrist_joint", actual_angle) == pytest.approx(raw_angle)


def test_actual_limits_match_raw_limits_when_offset_is_zero(calibration: JointCalibration):
    assert calibration.actual_limits("base_joint") == pytest.approx((-1.46955, 1.72113))
    assert calibration.actual_limits("shoulder_joint") == pytest.approx((-1.53450, 1.74900))
    assert calibration.actual_limits("elbow_joint") == pytest.approx((-1.63541, 1.66923))
    assert calibration.actual_limits("wrist_joint") == pytest.approx((-1.78041, 1.78041))
    assert calibration.actual_limits("gripper_joint") == pytest.approx((0.0, 2.59396))


def test_clamp_to_actual_limit(calibration: JointCalibration):
    assert calibration.clamp_to_actual_limit("elbow_joint", 10.0) == pytest.approx(1.66923)
    assert calibration.clamp_to_actual_limit("elbow_joint", -10.0) == pytest.approx(-1.63541)
    assert calibration.clamp_to_actual_limit("elbow_joint", 0.0) == pytest.approx(0.0)


def test_unknown_joint_falls_back_to_pi_range(calibration: JointCalibration):
    import math

    lower, upper = calibration.actual_limits("unknown_joint")
    assert lower == pytest.approx(-math.pi)
    assert upper == pytest.approx(math.pi)
