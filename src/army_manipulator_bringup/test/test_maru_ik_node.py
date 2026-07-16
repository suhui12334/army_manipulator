import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from maru_ik_node import compute_approach_pitch, quaternion_from_rpy


def test_compute_approach_pitch_clamps_to_home_and_limit():
    assert pytest.approx(compute_approach_pitch(0.20, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55)) == -0.25
    assert pytest.approx(compute_approach_pitch(0.55, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55)) == -0.95
    assert pytest.approx(compute_approach_pitch(0.37, home_pitch=-0.25, max_pitch=-0.95, z_min=0.20, z_max=0.55), abs=1e-3) == -0.59


def test_quaternion_from_rpy_returns_unit_length():
    q = quaternion_from_rpy(0.0, -0.5, 0.8)
    norm = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5
    assert pytest.approx(norm) == 1.0
