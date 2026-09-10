import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from maru_ik_node import (
    DEFAULT_WORKSPACE_MAX_TCP_DISTANCE_M,
    MEASURED_GROUND_IK_SEEDS,
    NEAR_IK_SEEDS,
    apply_grasp_target_correction,
    camera_depth_allows_direct_grasp,
    compute_approach_pitch,
    is_within_workspace_distance,
    ordered_static_ik_seeds,
    quaternion_from_rpy,
    target_forward_distance,
    target_requires_approach,
)
from grasp_wait_presets import GRASP_WAIT_PRESETS, get_grasp_wait_preset


def test_grasp_target_correction_matches_fixed_ground_ik_input():
    assert apply_grasp_target_correction(
        (0.5, -0.1, 0.8), (0.01, 0.02, -0.0475), True, -0.35,
    ) == pytest.approx((0.51, -0.08, -0.3975))


def test_grasp_target_correction_can_use_detected_height():
    assert apply_grasp_target_correction(
        (0.5, -0.1, 0.8), (0.01, 0.02, -0.0475), False, -0.35,
    ) == pytest.approx((0.51, -0.08, 0.7525))


def test_compute_approach_pitch_clamps_to_home_and_limit():
    assert pytest.approx(compute_approach_pitch(0.20, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55)) == -0.25
    assert pytest.approx(compute_approach_pitch(0.55, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55)) == -0.95
    assert pytest.approx(compute_approach_pitch(0.37, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55), abs=1e-3) == -0.59


def test_quaternion_from_rpy_returns_unit_length():
    q = quaternion_from_rpy(0.0, -0.5, 0.8)
    norm = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5
    assert pytest.approx(norm) == 1.0


def test_workspace_distance_precheck_rejects_only_absolute_outliers():
    assert is_within_workspace_distance(
        (0.70, 0.0, 0.0), DEFAULT_WORKSPACE_MAX_TCP_DISTANCE_M)
    assert not is_within_workspace_distance(
        (0.746, 0.001, -0.397), DEFAULT_WORKSPACE_MAX_TCP_DISTANCE_M)
    assert is_within_workspace_distance((99.0, 0.0, 0.0), 0.0)


def test_approach_gate_uses_base_frame_forward_x_distance_only():
    assert target_forward_distance((-0.85, 0.07, -0.40)) == pytest.approx(0.85)
    assert target_requires_approach((-0.22, 0.0, -0.40), 0.21)
    assert not target_requires_approach((-0.20, 9.0, -4.0), 0.21)
    assert not target_requires_approach((-99.0, 0.0, 0.0), 0.0)


def test_camera_depth_at_or_below_077_allows_direct_grasp():
    assert camera_depth_allows_direct_grasp(0.45, 1.0)
    assert camera_depth_allows_direct_grasp(1.0, 1.0)
    assert not camera_depth_allows_direct_grasp(1.001, 1.0)
    assert not camera_depth_allows_direct_grasp(0.0, 1.0)
    assert not camera_depth_allows_direct_grasp(0.45, 0.0)


def test_static_ik_seed_priority_depends_on_wrist_reach():
    assert ordered_static_ik_seeds(
        (-0.28, 0.02, -0.09))[0] == MEASURED_GROUND_IK_SEEDS[0]
    assert ordered_static_ik_seeds(
        (-0.20, 0.02, -0.09))[0] == NEAR_IK_SEEDS[0]


def test_grasp_wait_presets_include_legacy_and_five_candidates():
    assert tuple(GRASP_WAIT_PRESETS) == (
        "legacy", "grasp_wait1", "grasp_wait2", "grasp_wait3",
        "grasp_wait4", "grasp_wait5",
    )
    assert get_grasp_wait_preset(" GRASP_WAIT5 ") == pytest.approx({
        "shoulder_joint": 0.52735,
        "elbow_joint": 1.190575,
        "wrist_joint": 1.16187,
    })
    with pytest.raises(ValueError, match="unknown grasp_wait_preset"):
        get_grasp_wait_preset("bad")
