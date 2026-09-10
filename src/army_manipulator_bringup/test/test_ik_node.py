import math
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ik_node import (
    DISTANCE_GRASP_IK_SEEDS,
    GROUND_GRASP_IK_SEED,
    GRASP_WAIT_ARM_JOINTS,
    DEFAULT_BASE_HEIGHT_M,
    DEFAULT_BOX_GRASP_HEIGHT_RATIO,
    DEFAULT_SUPPLY_BOX_HEIGHT_M,
    DEFAULT_TCP_CALIBRATION_OFFSET_Z_M,
    PHYSICAL_FINGER_LENGTH_M,
    PINION_TO_TCP_DISTANCE_M,
    WRIST_TO_PINION_DISTANCE_M,
    WRIST_TO_TCP_LOCAL_OFFSET,
    compute_base_angle,
    fixed_grasp_z_from_geometry,
    required_arm_duration,
    select_distance_seed,
    supplybox_origin_to_tcp_target,
)

LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "ik_node_bringup.launch.py"
SUMMER_TCP_OFFSET_Z_M = -0.0055


def test_startup_grasp_wait_uses_actual_controller_angles():
    assert GRASP_WAIT_ARM_JOINTS["shoulder_joint"] == pytest.approx(0.0)
    assert GRASP_WAIT_ARM_JOINTS["elbow_joint"] == pytest.approx(math.pi / 2)
    assert GRASP_WAIT_ARM_JOINTS["wrist_joint"] == pytest.approx(math.pi / 2)
    assert "base_joint" not in GRASP_WAIT_ARM_JOINTS


def test_arm_duration_respects_slow_real_rmd_velocity():
    # A 90-degree shoulder move at the configured 15 deg/s needs 6 seconds,
    # plus the 0.5-second tracking margin.
    assert required_arm_duration(
        [0.0, -math.pi / 2, math.pi / 2, math.pi / 2],
        [0.0, 0.0, math.pi / 2, math.pi / 2],
        3.0,
    ) == pytest.approx(6.5)


def test_wrist_to_tcp_matches_90mm_pinion_tip_offset():
    assert PHYSICAL_FINGER_LENGTH_M == pytest.approx(0.080)
    assert WRIST_TO_PINION_DISTANCE_M == pytest.approx(0.150)
    assert PINION_TO_TCP_DISTANCE_M == pytest.approx(0.090)
    # 150 mm wrist->pinion + 90 mm pinion->TCP maps to wrist -X.
    assert WRIST_TO_TCP_LOCAL_OFFSET == pytest.approx((-0.240, 0.0, 0.0313))


def test_base_angle_reproduces_measured_sid_pose():
    assert compute_base_angle(-0.33212183, 0.00917221) == pytest.approx(
        -0.02761, abs=1e-5)


def test_fixed_grasp_z_is_box_side_center_relative_to_base_actuator():
    assert DEFAULT_BASE_HEIGHT_M == pytest.approx(0.350)
    assert DEFAULT_SUPPLY_BOX_HEIGHT_M == pytest.approx(0.095)
    assert DEFAULT_BOX_GRASP_HEIGHT_RATIO == pytest.approx(0.5)
    assert fixed_grasp_z_from_geometry(
        DEFAULT_BASE_HEIGHT_M,
        DEFAULT_SUPPLY_BOX_HEIGHT_M,
        DEFAULT_BOX_GRASP_HEIGHT_RATIO,
    ) == pytest.approx(-0.3025)


def test_fixed_z_replaces_only_detected_z_and_keeps_xy():
    assert supplybox_origin_to_tcp_target(
        (-0.25, 0.02, -0.40),
        DEFAULT_TCP_CALIBRATION_OFFSET_Z_M,
        -0.3025,
    ) == pytest.approx((-0.25, 0.02, -0.3025))


def test_detected_z_mode_remains_available_for_non_flat_ground():
    assert supplybox_origin_to_tcp_target(
        (-0.25, 0.02, -0.20), 0.01, None
    ) == pytest.approx((-0.25, 0.02, -0.19))


@pytest.mark.parametrize(
    "base_height,box_height,ratio",
    [(0.35, 0.0, 0.5), (0.35, 0.095, -0.1), (0.35, 0.095, 1.1)],
)
def test_invalid_fixed_grasp_geometry_is_rejected(base_height, box_height, ratio):
    with pytest.raises(ValueError):
        fixed_grasp_z_from_geometry(base_height, box_height, ratio)


def test_ground_seed_matches_bag_verified_downward_solution():
    assert GROUND_GRASP_IK_SEED == pytest.approx(
        [0.0, 0.9908576865, 1.6512680107, 0.7078743028]
    )
    assert sum(GROUND_GRASP_IK_SEED[1:]) == pytest.approx(3.35, abs=1e-7)


def test_summer_launch_lowers_fixed_z_by_5_5mm():
    text = LAUNCH_PATH.read_text(encoding="utf-8")
    match = re.search(
        r'"supplybox_tcp_offset_z",\s*default_value="([-0-9.]+)"', text)
    assert match is not None
    assert float(match.group(1)) == pytest.approx(SUMMER_TCP_OFFSET_Z_M)
    # ik_node.py itself stays generic; only the summer launch applies -5.5 mm.
    assert DEFAULT_TCP_CALIBRATION_OFFSET_Z_M == pytest.approx(0.0)
    assert supplybox_origin_to_tcp_target(
        (-0.30, 0.0, -0.35),
        SUMMER_TCP_OFFSET_Z_M,
        fixed_grasp_z_from_geometry(
            DEFAULT_BASE_HEIGHT_M,
            DEFAULT_SUPPLY_BOX_HEIGHT_M,
            DEFAULT_BOX_GRASP_HEIGHT_RATIO,
        ),
    ) == pytest.approx((-0.30, 0.0, -0.3080))


def test_distance_seeds_are_moveit_verified_summer_solutions():
    names = [entry[0] for entry in DISTANCE_GRASP_IK_SEEDS]
    assert names == ["near", "middle", "far"]
    for _, radius, (low, high), joints in DISTANCE_GRASP_IK_SEEDS:
        assert len(joints) == 4
        # base_joint is replaced by the target bearing at runtime.
        assert joints[0] == pytest.approx(0.0)
        assert low <= radius <= high
    by_name = {entry[0]: entry for entry in DISTANCE_GRASP_IK_SEEDS}
    # Near seed lives in the pitch-3.35 basin; middle/far in the pitch-pi basin.
    assert sum(by_name["near"][3][1:]) == pytest.approx(3.35, abs=1e-6)
    assert sum(by_name["middle"][3][1:]) == pytest.approx(math.pi, abs=1e-6)
    assert sum(by_name["far"][3][1:]) == pytest.approx(math.pi, abs=1e-6)
    # Bands overlap so the |X|<=0.30 m, |Y|<=0.03 m operating range is covered.
    assert by_name["near"][2] == (0.160, 0.290)
    assert by_name["middle"][2] == (0.215, 0.315)
    assert by_name["far"][2] == (0.215, 0.315)


@pytest.mark.parametrize(
    "radius,expected",
    [
        (0.158, "near"),   # below near band: nearest representative
        (0.160, "near"),
        (0.200, "near"),   # only near band contains 0.200 m
        (0.214, "near"),
        (0.225, "middle"),
        (0.250, "middle"),
        (0.270, "far"),
        (0.300, "far"),
        (0.315, "far"),
        (0.350, "far"),    # outside every band: nearest representative
    ],
)
def test_select_distance_seed_prefers_verified_band(radius, expected):
    assert select_distance_seed(radius)[0] == expected
