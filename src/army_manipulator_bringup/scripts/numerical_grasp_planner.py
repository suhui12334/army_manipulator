#!/usr/bin/env python3
"""MARU 자동 파지 흐름을 ROS 없이 수치 계산하는 단일 파일 도구.

포함하는 계산:

1. 카메라 3-D 점을 주어진 camera->base_actuator 강체변환으로 변환
2. grasp offset/fixed-ground 보정
3. base_actuator 정면 X 거리 0.21 m 기준 FORWARD 또는 IK 판단
4. URDF와 같은 4-DOF FK + 유한차분 Jacobian DLS(damped least squares) IK
5. 정확한 IK 실패 시 현재 TCP에서 목표 방향의 최대 부분 접근점 탐색

이 파일은 MoveIt planning scene/self-collision/controller 상태를 사용하지 않는다.
따라서 결과는 진단·파라미터 튜닝용이며, 계산된 관절값을 하드웨어에 직접
전송하지 말고 실제 실행 전에는 maru_ik_node/MoveIt의 충돌 검사를 거쳐야 한다.

예시:
  # 이미 /arm/target_point_base에서 읽은 보정 완료 좌표
  python3 numerical_grasp_planner.py --base-target -0.20 0.01 -0.30

  # 카메라 optical 좌표와 camera->base_actuator TF(tx ty tz roll pitch yaw)
  python3 numerical_grasp_planner.py \
    --camera-point 0.01 0.02 0.45 \
    --camera-to-base 0.10 0.00 0.30 0.0 0.2 3.14
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Iterable, Sequence

import numpy as np

from grasp_wait_presets import (
    DEFAULT_GRASP_WAIT_PRESET,
    GRASP_WAIT_PRESETS,
    get_grasp_wait_preset,
)


JOINT_NAMES = ("base_joint", "shoulder_joint", "elbow_joint", "wrist_joint")
JOINT_LOWER = np.array([-1.46955, -1.53450, -1.63541, -1.78041], dtype=float)
JOINT_UPPER = np.array([1.72113, 1.74900, 1.66923, 1.78041], dtype=float)

# army_manipulator_macro.xacro와 동일한 치수/오프셋(m).
SHOULDER_OFF = 0.1039
L1 = 0.180
L2 = 0.220
OFF_SHOULDER = 0.038
OFF_ELBOW = 0.0188
OFF_WRIST = -0.0313
OFF_GRIPPER = 0.038
WRIST_TO_TCP = np.array([-0.240, 0.0, 0.0313], dtype=float)

GRASP_START_X_DISTANCE_M = 0.21
PREGRASP_OFFSET_Z_M = 0.10
GRASP_OFFSET = np.array([0.0, 0.0, -0.0475], dtype=float)
GROUND_Z_M = -0.35
PITCH_CANDIDATES = (2.95, 3.05, 2.85, 3.15, 2.75, 3.25, 2.65, 3.35)
PARTIAL_REACH_FRACTIONS = (0.85, 0.70, 0.55, 0.40, 0.25)

def grasp_wait_joint_vector(name: str, base_joint: float = -0.02761) -> np.ndarray:
    """Return [base, shoulder, elbow, wrist] for a shared preset."""
    arm = get_grasp_wait_preset(name)
    return np.array([
        float(base_joint), arm["shoulder_joint"], arm["elbow_joint"], arm["wrist_joint"]
    ], dtype=float)


# Compatibility default for imports/tests; CLI can select any shared preset.
GRASP_WAIT = grasp_wait_joint_vector(DEFAULT_GRASP_WAIT_PRESET)
STATIC_SEEDS = (
    np.array([-0.01687, 1.42349, 1.30027, 0.48066]),
    np.array([-0.01687, 1.71007, 0.61104, 0.49585]),
    np.array([-0.02761, 1.71566, 0.62151, 0.82519]),  # sid
    np.array([-0.02915, 0.36652, 1.62316, 1.02974]),
    np.array([-0.02915, 1.39626, 1.27409, 0.31416]),
    np.array([-0.09357, 1.65806, 0.99484, 0.08727]),
    np.array([-0.02000, 0.30000, 0.10000, 1.00000]),
    np.array([-0.02000, 1.60000, 0.10000, 1.00000]),
)


class Action(str, Enum):
    FORWARD = "forward"
    GRASP = "grasp"
    PARTIAL_REACH_AND_REOBSERVE = "partial_reach_and_reobserve"
    NO_SOLUTION = "no_solution"


@dataclass
class IKResult:
    success: bool
    joints: list[float]
    pitch: float
    iterations: int
    position_error_m: float
    pitch_error_rad: float


@dataclass
class PlanResult:
    action: str
    target_base: list[float]
    forward_x_distance_m: float
    straight_3d_distance_m: float
    pregrasp: IKResult | None = None
    grasp: IKResult | None = None
    partial_fraction: float | None = None
    partial_target: list[float] | None = None
    message: str = ""


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((1, 0, 0), (0, c, -s), (0, s, c)), dtype=float)


def _ry(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)), dtype=float)


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)), dtype=float)


def _transform(xyz=(0.0, 0.0, 0.0), rotation=None) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.eye(3) if rotation is None else rotation
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def rpy_transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """ROS URDF fixed transform: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = map(float, rpy)
    return _transform(xyz, _rz(yaw) @ _ry(pitch) @ _rx(roll))


def transform_point(transform: np.ndarray, point: Sequence[float]) -> np.ndarray:
    homogeneous = np.append(np.asarray(point, dtype=float), 1.0)
    return (np.asarray(transform, dtype=float) @ homogeneous)[:3]


def forward_kinematics(joints: Sequence[float]) -> tuple[np.ndarray, float]:
    """Return TCP xyz in base_actuator and shoulder+elbow+wrist pitch.

    The transform order is copied directly from the current URDF joints. This
    implementation reproduces the verified sid TCP (-0.33212, 0.00917,
    -0.28462) to floating-point precision.
    """
    base, shoulder, elbow, wrist = map(float, joints)
    tf = _transform(rotation=_rz(base))
    tf = tf @ rpy_transform(
        (0.0, OFF_SHOULDER, SHOULDER_OFF),
        (math.pi / 2.0, math.pi / 2.0, 0.0),
    ) @ _transform(rotation=_rz(shoulder))
    tf = tf @ _transform((-L1, 0.0, OFF_GRIPPER + OFF_ELBOW))
    tf = tf @ _transform(rotation=_rz(elbow))
    tf = tf @ _transform((-L2, 0.0, -(OFF_ELBOW - OFF_WRIST)))
    tf = tf @ _transform(rotation=_rz(wrist))
    tf = tf @ _transform(WRIST_TO_TCP)
    return tf[:3, 3].copy(), shoulder + elbow + wrist


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _residual(
    joints: np.ndarray,
    target_xyz: np.ndarray,
    target_pitch: float,
    pitch_weight_m_per_rad: float,
) -> np.ndarray:
    xyz, pitch = forward_kinematics(joints)
    return np.append(
        xyz - target_xyz,
        pitch_weight_m_per_rad * _wrap_pi(pitch - target_pitch),
    )


def solve_ik_dls(
    target_xyz: Sequence[float],
    target_pitch: float,
    seed: Sequence[float],
    *,
    max_iterations: int = 250,
    position_tolerance_m: float = 0.003,
    pitch_tolerance_rad: float = 0.025,
    damping: float = 0.025,
    pitch_weight_m_per_rad: float = 0.08,
) -> IKResult:
    """Numerically solve one seed using finite-difference damped least squares."""
    target = np.asarray(target_xyz, dtype=float)
    q = np.clip(np.asarray(seed, dtype=float), JOINT_LOWER, JOINT_UPPER)
    finite_difference = 1.0e-5

    for iteration in range(1, max_iterations + 1):
        xyz, pitch = forward_kinematics(q)
        position_error = float(np.linalg.norm(xyz - target))
        pitch_error = abs(_wrap_pi(pitch - target_pitch))
        if position_error <= position_tolerance_m and pitch_error <= pitch_tolerance_rad:
            return IKResult(
                True, q.tolist(), float(target_pitch), iteration,
                position_error, pitch_error,
            )

        residual = _residual(q, target, target_pitch, pitch_weight_m_per_rad)
        jacobian = np.empty((4, 4), dtype=float)
        for column in range(4):
            perturbed = q.copy()
            perturbed[column] += finite_difference
            jacobian[:, column] = (
                _residual(perturbed, target, target_pitch, pitch_weight_m_per_rad)
                - residual
            ) / finite_difference

        normal = jacobian.T @ jacobian + (damping * damping) * np.eye(4)
        try:
            delta = np.linalg.solve(normal, -(jacobian.T @ residual))
        except np.linalg.LinAlgError:
            break
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > 0.25:
            delta *= 0.25 / delta_norm

        old_cost = float(residual @ residual)
        accepted = False
        step = 1.0
        for _ in range(10):
            candidate = np.clip(q + step * delta, JOINT_LOWER, JOINT_UPPER)
            candidate_residual = _residual(
                candidate, target, target_pitch, pitch_weight_m_per_rad)
            if float(candidate_residual @ candidate_residual) < old_cost:
                q = candidate
                accepted = True
                break
            step *= 0.5
        if not accepted:
            break

    xyz, pitch = forward_kinematics(q)
    return IKResult(
        False, q.tolist(), float(target_pitch), iteration,
        float(np.linalg.norm(xyz - target)),
        abs(_wrap_pi(pitch - target_pitch)),
    )


def _unique_seeds(current_joints: Sequence[float]) -> Iterable[np.ndarray]:
    seen = set()
    for seed in (np.asarray(current_joints, dtype=float), *STATIC_SEEDS):
        key = tuple(round(float(value), 6) for value in seed)
        if key not in seen:
            seen.add(key)
            yield seed


def solve_multi_seed(
    target_xyz: Sequence[float],
    pitch_candidates: Sequence[float],
    current_joints: Sequence[float],
) -> IKResult | None:
    """Try pitch candidates and deterministic seeds, returning the best valid result."""
    best_failure = None
    for pitch in pitch_candidates:
        for seed in _unique_seeds(current_joints):
            result = solve_ik_dls(target_xyz, pitch, seed)
            if result.success:
                return result
            if best_failure is None or result.position_error_m < best_failure.position_error_m:
                best_failure = result
    return None


def apply_target_correction(
    detected_base: Sequence[float],
    grasp_offset: Sequence[float] = GRASP_OFFSET,
    *,
    use_fixed_ground_z: bool = True,
    ground_z_m: float = GROUND_Z_M,
) -> np.ndarray:
    detected = np.asarray(detected_base, dtype=float)
    offset = np.asarray(grasp_offset, dtype=float)
    return np.array((
        detected[0] + offset[0],
        detected[1] + offset[1],
        (ground_z_m if use_fixed_ground_z else detected[2]) + offset[2],
    ))


def plan_grasp(
    target_base: Sequence[float],
    current_joints: Sequence[float] = GRASP_WAIT,
    *,
    grasp_start_x_distance_m: float = GRASP_START_X_DISTANCE_M,
    pregrasp_offset_z_m: float = PREGRASP_OFFSET_Z_M,
) -> PlanResult:
    """Run the current approach/IK/partial-reach state decision numerically."""
    target = np.asarray(target_base, dtype=float)
    current = np.asarray(current_joints, dtype=float)
    forward_distance = abs(float(target[0]))
    straight_distance = float(np.linalg.norm(target))
    if grasp_start_x_distance_m > 0.0 and forward_distance > grasp_start_x_distance_m:
        return PlanResult(
            Action.FORWARD.value, target.tolist(), forward_distance, straight_distance,
            message=(
                f"|x|={forward_distance:.3f}m > {grasp_start_x_distance_m:.3f}m: "
                "keep picking=false and publish /arm/forward_command"
            ),
        )

    pregrasp_target = target.copy()
    pregrasp_target[2] += pregrasp_offset_z_m
    pregrasp = solve_multi_seed(pregrasp_target, PITCH_CANDIDATES, current)
    if pregrasp is not None:
        grasp = solve_multi_seed(target, PITCH_CANDIDATES, pregrasp.joints)
        if grasp is not None:
            return PlanResult(
                Action.GRASP.value, target.tolist(), forward_distance, straight_distance,
                pregrasp=pregrasp, grasp=grasp,
                message="pregrasp and grasp numerical IK succeeded",
            )

    current_tcp, current_pitch = forward_kinematics(current)
    partial_pitches = (current_pitch, *PITCH_CANDIDATES)
    for fraction in PARTIAL_REACH_FRACTIONS:
        partial_target = current_tcp + fraction * (target - current_tcp)
        partial = solve_multi_seed(partial_target, partial_pitches, current)
        if partial is not None:
            return PlanResult(
                Action.PARTIAL_REACH_AND_REOBSERVE.value,
                target.tolist(), forward_distance, straight_distance,
                pregrasp=pregrasp, grasp=partial,
                partial_fraction=float(fraction),
                partial_target=partial_target.tolist(),
                message=(
                    f"exact IK failed; move to {fraction:.2f} interpolated point, "
                    "then discard the old target and re-observe"
                ),
            )

    return PlanResult(
        Action.NO_SOLUTION.value, target.tolist(), forward_distance, straight_distance,
        pregrasp=pregrasp,
        message="exact and partial numerical IK produced no joint-limit-valid solution",
    )


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--base-target", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="already-corrected /arm/target_point_base xyz",
    )
    source.add_argument(
        "--camera-point", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="raw camera optical-frame xyz",
    )
    parser.add_argument(
        "--camera-to-base", nargs=6, type=float,
        metavar=("TX", "TY", "TZ", "ROLL", "PITCH", "YAW"),
        help="camera optical frame -> base_actuator TF; required with --camera-point",
    )
    parser.add_argument(
        "--current-joints", nargs=4, type=float, default=None,
        metavar=("BASE", "SHOULDER", "ELBOW", "WRIST"),
        help="explicit current joints; overrides --grasp-wait-preset",
    )
    parser.add_argument(
        "--grasp-wait-preset", choices=tuple(GRASP_WAIT_PRESETS),
        default=DEFAULT_GRASP_WAIT_PRESET,
        help="default current pose when --current-joints is omitted",
    )
    parser.add_argument(
        "--grasp-wait-base", type=float, default=-0.02761,
        help="base_joint used with --grasp-wait-preset (automatic node holds live base)",
    )
    parser.add_argument("--grasp-start-x", type=float, default=GRASP_START_X_DISTANCE_M)
    parser.add_argument("--ground-z", type=float, default=GROUND_Z_M)
    parser.add_argument("--no-fixed-ground-z", action="store_true")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.base_target is not None:
        target_base = np.asarray(args.base_target, dtype=float)
    else:
        if args.camera_to_base is None:
            raise SystemExit("--camera-point requires --camera-to-base")
        tx, ty, tz, roll, pitch, yaw = args.camera_to_base
        raw_base = transform_point(
            rpy_transform((tx, ty, tz), (roll, pitch, yaw)),
            args.camera_point,
        )
        target_base = apply_target_correction(
            raw_base,
            use_fixed_ground_z=not args.no_fixed_ground_z,
            ground_z_m=args.ground_z,
        )

    current_joints = (
        args.current_joints
        if args.current_joints is not None
        else grasp_wait_joint_vector(
            args.grasp_wait_preset, args.grasp_wait_base).tolist()
    )
    result = plan_grasp(
        target_base,
        current_joints,
        grasp_start_x_distance_m=args.grasp_start_x,
    )
    payload = asdict(result)
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(f"action: {result.action}")
        print(f"target_base: {np.array(result.target_base)} m")
        print(f"forward |x|: {result.forward_x_distance_m:.4f} m")
        print(f"3-D distance: {result.straight_3d_distance_m:.4f} m")
        print(result.message)
        if result.pregrasp is not None:
            print(f"pregrasp joints: {np.array(result.pregrasp.joints)}")
        if result.grasp is not None:
            label = "partial" if result.partial_fraction is not None else "grasp"
            print(f"{label} joints: {np.array(result.grasp.joints)}")
        if result.partial_target is not None:
            print(f"partial target: {np.array(result.partial_target)} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
