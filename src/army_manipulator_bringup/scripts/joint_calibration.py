#!/usr/bin/env python3
"""관절별 raw 인코더 <-> actual(URDF/IK) 각도 변환 및 실측 리미트 유틸리티.

보정식 (army_manipulator_description/config/joint_calibration.yaml과 동일):
  actual_joint_angle = raw_encoder_angle - zero_offset
  actual_limit       = raw_limit         - zero_offset

zero_offset은 관절별 독립 상수이며, 실측 전까지는 joint_calibration.yaml에
0.0 placeholder로 들어있다. 이 모듈은 그 yaml 하나만 읽어서 값을 제공하므로,
실측 후에는 yaml 파일만 갱신하면 IK 노드를 포함한 모든 사용처에 반영된다.

TODO(zero-offset): joint_calibration.yaml의 zero_offset 실측값 반영.
"""

import math
import pathlib
from typing import Dict, Optional, Tuple

try:
    import yaml
except ModuleNotFoundError:  # pragma: no cover - PyYAML 미설치 환경 대비
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # pragma: no cover - 순수 파이썬 테스트 환경 대비
    get_package_share_directory = None

CALIBRATION_PACKAGE = "army_manipulator_description"
CALIBRATION_RELATIVE_PATH = "config/joint_calibration.yaml"


def _default_calibration_path() -> pathlib.Path:
    """army_manipulator_description 패키지의 joint_calibration.yaml 경로를 찾는다."""
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory(CALIBRATION_PACKAGE)
            return pathlib.Path(share_dir) / CALIBRATION_RELATIVE_PATH
        except Exception:
            pass
    # ament share 디렉터리를 못 찾는 경우(빌드 전 소스 트리에서 직접 테스트 등)
    # scripts/ 기준 상대 경로로 소스 트리의 yaml을 fallback으로 사용한다.
    return (
        pathlib.Path(__file__).resolve().parents[2]
        / CALIBRATION_PACKAGE
        / CALIBRATION_RELATIVE_PATH
    )


class JointCalibration:
    """joint_calibration.yaml을 읽어 raw <-> actual 변환/리미트 조회를 제공한다."""

    def __init__(self, path: Optional[pathlib.Path] = None):
        self.path = pathlib.Path(path) if path is not None else _default_calibration_path()
        self._joints: Dict[str, Dict[str, float]] = self._load(self.path)

    @staticmethod
    def _load(path: pathlib.Path) -> Dict[str, Dict[str, float]]:
        if yaml is None or not path.exists():
            return {}
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data.get("joints", {})

    def zero_offset(self, joint_name: str) -> float:
        return float(self._joints.get(joint_name, {}).get("zero_offset", 0.0))

    def has_joint(self, joint_name: str) -> bool:
        """Return whether calibration data for ``joint_name`` was loaded."""
        return joint_name in self._joints

    def raw_to_actual(self, joint_name: str, raw_angle: float) -> float:
        """actual_joint_angle = raw_encoder_angle - zero_offset"""
        return raw_angle - self.zero_offset(joint_name)

    def actual_to_raw(self, joint_name: str, actual_angle: float) -> float:
        """raw_encoder_target = actual_angle + zero_offset (역변환)"""
        return actual_angle + self.zero_offset(joint_name)

    def actual_limits(self, joint_name: str) -> Tuple[float, float]:
        """(actual_min, actual_max) = (raw_min - offset, raw_max - offset).

        joint_calibration.yaml에 없는 조인트는 안전한 fallback으로 ±pi를 반환한다.
        """
        joint = self._joints.get(joint_name)
        if joint is None:
            return (-math.pi, math.pi)
        offset = float(joint.get("zero_offset", 0.0))
        return (float(joint["raw_min"]) - offset, float(joint["raw_max"]) - offset)

    def clamp_to_actual_limit(self, joint_name: str, actual_angle: float) -> float:
        """actual 기준 조인트 각도를 joint_calibration.yaml의 actual_limit으로 clamp."""
        lower, upper = self.actual_limits(joint_name)
        return max(lower, min(upper, actual_angle))
