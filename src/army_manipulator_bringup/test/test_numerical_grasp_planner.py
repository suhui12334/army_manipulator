import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from numerical_grasp_planner import (
    GRASP_WAIT,
    apply_target_correction,
    forward_kinematics,
    plan_grasp,
    rpy_transform,
    solve_ik_dls,
    transform_point,
)


SID = np.array([-0.02761, 1.71566, 0.62151, 0.82519])


def test_fk_reproduces_moveit_verified_sid_tcp():
    xyz, pitch = forward_kinematics(SID)
    assert xyz == pytest.approx((-0.33212183, 0.00917221, -0.28461508), abs=1e-7)
    assert pitch == pytest.approx(sum(SID[1:]))


def test_camera_transform_and_fixed_ground_correction():
    tf = rpy_transform((1.0, 2.0, 3.0), (0.0, 0.0, 0.0))
    base = transform_point(tf, (0.1, 0.2, 0.3))
    assert base == pytest.approx((1.1, 2.2, 3.3))
    assert apply_target_correction(base) == pytest.approx((1.1, 2.2, -0.3975))


def test_dls_ik_recovers_sid_pose_from_nearby_seed():
    target, pitch = forward_kinematics(SID)
    result = solve_ik_dls(target, pitch, SID + np.array((0.03, -0.04, 0.03, -0.02)))
    assert result.success
    solved_xyz, solved_pitch = forward_kinematics(result.joints)
    assert solved_xyz == pytest.approx(target, abs=0.003)
    assert solved_pitch == pytest.approx(pitch, abs=0.025)


def test_far_x_requests_forward_without_running_ik():
    result = plan_grasp((-0.85, -0.06, -0.397), GRASP_WAIT)
    assert result.action == "forward"
    assert result.forward_x_distance_m == pytest.approx(0.85)
