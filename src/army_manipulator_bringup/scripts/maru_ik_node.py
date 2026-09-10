#!/usr/bin/env python3
"""D435 ``supplybox`` 3D 점을 MoveIt 계획과 ros2_control 실행으로 연결한다.

입력 점은 반드시 카메라 optical frame으로 publish한다. 이 노드가 TF로
``base_link``로 변환한 뒤, pre-grasp -> grasp -> close -> lift 순서를 MoveIt의
move_group action으로 실행한다. MoveIt이 만든 FollowJointTrajectory는
ros2_control을 거쳐 RMD(CAN) 및 Dynamixel 하드웨어 인터페이스로 전달된다.
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum, auto
from typing import Sequence

from joint_calibration import JointCalibration
from grasp_wait_presets import (
    DEFAULT_GRASP_WAIT_PRESET,
    GRASP_WAIT_PRESETS,
    get_grasp_wait_preset,
)
from numerical_grasp_planner import solve_ik_dls
from analytic_grasp_planner import solve_ik_analytic

try:
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from geometry_msgs.msg import PointStamped, PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Empty, Float64
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from tf2_geometry_msgs import do_transform_point
    import tf2_ros
    from pymoveit2 import MoveIt2, MoveIt2State
    from moveit_msgs.srv import GetPositionFK, GetPositionIK
    from moveit_msgs.msg import PositionIKRequest, RobotState as RobotStateMsg
except ModuleNotFoundError:  # unit-test import without a ROS installation
    rclpy = None
    Node = object
    MultiThreadedExecutor = None


ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
GRIPPER_JOINT_NAMES = ["gripper_joint"]
def load_arm_joint_limits(calibration: JointCalibration) -> dict[str, tuple[float, float]]:
    """Load actual (URDF/IK) limits from the single calibration source."""
    missing = [name for name in ARM_JOINT_NAMES if not calibration.has_joint(name)]
    if missing:
        raise RuntimeError(
            f"joint calibration unavailable ({calibration.path}): missing {', '.join(missing)}"
        )
    return {name: calibration.actual_limits(name) for name in ARM_JOINT_NAMES}
# AUTO storage pose. This is deliberately independent from MANUAL_EE's
# shoulder-only camera-clearance pose. The base joint holds its measured angle
# during the actual entry move so the arm does not sweep sideways.
# [갱신, 2026-08-31] capture_arm_pose.py로 실기에서 재캡처 - 기존 값은 부호
# 자체가 반대(shoulder=+1.71548)였던 잘못된 자세였음. shoulder는 새 리미트
# 경계(joint_calibration.yaml의 실제 리미트)에 걸리므로 그 값 그대로, elbow/wrist는 HOME/HOLD/
# GRASP_WAIT 세 자세 모두 하드스톱 근처에서 거의 동일하게 캡처돼(팔이 접힌
# 상태에서 그대로 유지되는 관절) 새 안전 리미트 값을 공유해서 쓴다.
HOME_ARM_JOINTS = {
    "shoulder_joint": -1.4818,
    "elbow_joint": 1.6383,
    "wrist_joint": 1.5329,
}
# 이름 붙인 4관절(base/shoulder/elbow/wrist) 전체 자세.
# STAND: 액추에이터 전부 0도(=IK_SEEDS의 "stand" seed와 동일).
STAND_JOINTS = [0.0, 0.0, 0.0, 0.0]
# AUTO HOME: 적재용 절대 관절 자세. 실제 진입 동작에서는 base를
# 현재 위치로 유지하고 shoulder를 먼저 상한으로 이동한 뒤 elbow/wrist를 이동한다.
HOME_JOINTS = [0.0, *HOME_ARM_JOINTS.values()]
# AUTO 파지 후 운반 자세. Base는 파지 시점의 실측 위치를 유지한다.
# [갱신, 2026-08-31] HOME_ARM_JOINTS와 동일한 재캡처 - shoulder만 다르고
# elbow/wrist는 위와 같은 이유로 새 안전 리미트 값 공유.
# [갱신, 2026-09-01] HOLD/GRASP_WAIT의 shoulder 값을 서로 교체(elbow/wrist는
# 원래 두 자세가 동일해서 교체해도 값이 안 바뀜).
HOLD_ARM_JOINTS = {
    "shoulder_joint": -0.76044,
    "elbow_joint": 1.6383,
    "wrist_joint": 1.5329,
}
# [추가, 2026-08-31] capture_arm_pose.py로 같이 캡처됨.
# [갱신, 2026-09-01] 위 HOLD_ARM_JOINTS와 shoulder 값 교체.
# Backward-compatible exported value. Runtime motions use the selected
# ``grasp_wait_preset`` copied into ``self.grasp_wait_arm_joints``.
GRASP_WAIT_ARM_JOINTS = get_grasp_wait_preset(DEFAULT_GRASP_WAIT_PRESET)
GRIPPER_OPEN = 0.07363
# [수정] 2026-08-30 실기 실측: 100mA 전류 임계값과 같이 쓸 닫힘 목표를
# 기계적 최대(2.59396, joint_calibration.yaml raw_max)에서 1.8294로 낮춤.
GRIPPER_CLOSED = 1.8294
# [추가, 2026-09-02] move_to_named_pose.py _close_gripper_with_feedback()과
# 동일한 OR 조건용 - GRIPPER_CLOSED 근처까지 도달했다고 볼 위치 오차 허용폭.
GRIPPER_POSITION_TOLERANCE_RAD = 0.01

# [추가] KDL(지역 수치해법)은 IK seed(시작 관절각)에 극도로 민감하다 - 이번
# 세션에서 실측 확인: move_to_pose()는 pymoveit2 내부에서 항상 "현재 상태"만
# seed로 쓰는데(공개 API로 다른 seed를 못 줌), HOME 이동 직후의 현재
# 상태(HOME 자세 근방)가 하필 나쁜 seed인 목표에서 pregrasp가 pitch 후보
# 7개를 다 시도해도 전부 실패하는 걸 확인했다(random_target_publisher.py가
# 이미 "도달 가능"으로 검증한 좌표인데도). random_target_publisher.py와
# 동일한 다중 seed 전략을 여기서도 써서, move_to_pose() 대신 compute_ik를
# 직접 여러 seed로 호출해 해를 구한 뒤 move_to_configuration()으로 실행한다.
# [갱신, 2026-08-31] "stand"/"grip_wait"는 지면 supplybox 자세와 거리가 멀어
# KDL이 거기서 출발해도 잘 못 찾는 걸 확인 - capture_arm_pose.py 실측
# 41개 중 서로 다른 영역(x0≈0.25, x0≈0.35)을 대표하는 실제 도달 가능
# 자세 2개로 교체했다(seed 개수는 유지 - 늘리면 pitch 후보 x seed 조합
# 시간이 급격히 늘어남).
# [갱신, 2026-09-01] capture_arm_pose.py로 실제 supplybox 파지 동작을 손으로
# 그대로 재현하며 18개 자세를 연속 캡처(base_joint는 전 구간 -0.01687로
# 고정) - "elbow가 접힌" 형태와 "elbow가 펴진" 형태를 각각 대표하는 자세 2개로
# 위 seed를 교체(seed 개수는 유지). pitch(=shoulder+elbow+wrist 합)도 각각
# 3.20/2.82로, 이 세션에서 실측된 pitch 범위(2.73~3.28)의 위/아래 근처를
# 대표한다.
# [갱신, 2026-09-02] 이전 ``far_reach`` seed는 FK 재검증 결과 x0=0.359m인
# 근거리 자세였다. 실제 파지 동작에서 쓰는 named pose 흐름은
# grasp_wait -> sid -> hold이고, 그중 sid는 손으로 박스 위치까지 이동해
# 캡처한 실측 관절값이다. 따라서 이름과 실측 의미가 맞지 않는 far_reach
# seed를 sid로 교체한다. grasp_wait/hold는 각각 IK 직전/직후의 현재 자세라서
# _solve_ik_multi_seed가 동적으로 맨 앞에 넣는 current seed로 이미 사용된다.
# [갱신, 2026-09-01 #3] 그런데도 x0≈0.746m 타겟이 avoid_collisions=False로도
# 계속 실패 - 실측(줄자)으로 이 팔의 최대 수평 리치가 약 0.77m임을 확인, 즉
# 이 타겟은 최대 리치의 97%대라 특이점 근처라서 유효 관절 조합이 매우 좁다.
# 그런데 당시 실측 seed 3개조차 전부 elbow가 꽤 굽어있어서
# (0.45~1.3rad) "거의 다 편" 형태를 하나도 대표 못 한다. 실측 없이 바로
# 커버하려고 elbow≈0.1(거의 폄)에 shoulder만 다르게 잡은 합성 seed 2개를
# 추가했다(seed 4->6개, "무조건 움직이게" 우선 - pitch 8개 기준 최악 시간
# 약 96s->144s로 증가하지만 성공률을 우선한 선택).
NEAR_IK_SEEDS = [
    [-0.01687, 1.42349, 1.30027, 0.48066],  # 실측(elbow 접힘, 근거리, pitch≈3.20)
    [-0.01687, 1.71007, 0.61104, 0.49585],  # 실측(elbow 펴짐, 근거리, pitch≈2.82)
    [-0.02761, 1.71566, 0.62151, 0.82519],  # 실측 sid(파지 위치, pitch≈3.16)
]

# [갱신, 2026-09-02] 지면 supplybox 방향으로 팔을 연속 이동하며 캡처한
# 44개 실측 자세 중 관절공간 초·중·최종 구간을 대표하는 3개다.
# 마지막 자세는 4회 연속 동일값으로 캡처된 수렴 자세다. 모든 샘플을
# seed로 넣지 않고 대표값만 써서 pitch x seed 탐색시간을 제한한다.
MEASURED_GROUND_IK_SEEDS = [
    [-0.02915, 0.36652, 1.62316, 1.02974],
    [-0.02915, 1.39626, 1.27409, 0.31416],
    [-0.09357, 1.65806, 0.99484, 0.08727],
]

SYNTHETIC_FAR_IK_SEEDS = [
    [-0.02000, 0.30000, 0.10000, 1.00000],  # 합성(거의 폄, shoulder 낮음 - 최대리치 대비)
    [-0.02000, 1.60000, 0.10000, 1.00000],  # 합성(거의 폄, shoulder 높음 - 최대리치 대비)
]

EXTENDED_GROUND_WRIST_RADIUS_M = 0.25


def ordered_static_ik_seeds(position) -> list[list[float]]:
    """Prioritize captured ground-approach seeds for an extended wrist target."""
    extended = (
        math.hypot(float(position[0]), float(position[1]))
        >= EXTENDED_GROUND_WRIST_RADIUS_M
    )
    if extended:
        ordered = [*MEASURED_GROUND_IK_SEEDS, *NEAR_IK_SEEDS,
                   *SYNTHETIC_FAR_IK_SEEDS, list(HOME_JOINTS)]
    else:
        ordered = [*NEAR_IK_SEEDS, list(HOME_JOINTS),
                   *MEASURED_GROUND_IK_SEEDS, *SYNTHETIC_FAR_IK_SEEDS]
    return [list(seed) for seed in ordered]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Sequence[float]:
    """Return a unit quaternion in ROS ``(x, y, z, w)`` order."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def compute_approach_pitch(
    target_z: float, home_pitch: float = -0.25, max_pitch: float = -0.95,
    z_min: float = 0.20, z_max: float = 0.55,
) -> float:
    """Legacy-compatible bounded pitch interpolation used by the unit tests."""
    if target_z <= z_min:
        return home_pitch
    if target_z >= z_max:
        return max_pitch
    return home_pitch + (target_z - z_min) * (max_pitch - home_pitch) / (z_max - z_min)


# [수정] army_manipulator_macro.xacro가 shoulder/elbow/wrist를 axis=X에서
# axis=Z로 재구성(링크 프레임 Ry(+90°) 보정 포함)한 뒤로 base_joint의 실질
# 회전축이 base_link 기준 Y(수직)가 아니라 Z(전방)라는 게 compute_fk 실측으로
# 확인됨 - base_joint를 스윕하면 z는 완전히 고정된 채 x,y가 함께 회전한다.
# 그래서 예전 quaternion_from_yaw_x_pitch(yaw=atan2(x,z), pitch)는 이 팔이
# 애초에 도달 불가능한 방향만 계산해서 대부분의 pose 목표가
# NO_IK_SOLUTION(-31)으로 실패했다.
#
# 실측(compute_fk)으로 확인된 사실 두 가지:
#  1) base_joint=0일 때 wrist_link의 Y좌표는 shoulder/elbow/wrist 값과 무관하게
#     거의 상수(Y0≈0.0313m)이고, X/Z만 shoulder/elbow/wrist에 따라 변한다.
#     즉 base_joint는 (x0, Y0) 벡터를 Z축 기준으로 회전시켜 최종 (x,y)를 만든다.
#  2) wrist_link의 orientation은 shoulder+elbow+wrist "합"에만 의존한다(각
#     관절의 개별 분배는 orientation에 영향 없음, FK로 검증됨) - 이 합을
#     "pitch"라 부르면, base=0/pitch=0에서의 orientation은 정확히
#     Q0=(0.5,0.5,-0.5,0.5)이고 q_local(pitch) = Q0 (x) Rz(pitch).
# 최종 orientation은 q_final = Rz(base_angle) (x) Q0 (x) Rz(pitch).
_Q0_X, _Q0_Y, _Q0_Z, _Q0_W = 0.5, 0.5, -0.5, 0.5
BASE_ROTATION_Y_OFFSET_M = 0.0313


# [추가, 2026-08-31] 지면 supplybox를 실제로 잡는 자세 41개를
# capture_arm_pose.py로 캡처하고 compute_fk로 wrist_link 실좌표를 뽑아서
# 아래 원래 공식(보정 전) 값과 실측 base_joint를 비교한 결과, 거의 상수인
# 오프셋(평균 2.9439rad, 표준편차 0.0156rad, N=41)만큼 항상 어긋나 있는 게
# 확인됨 - 숄더 오프셋 삼각형에서 반대쪽(가까운 쪽) 해를 실기가 쓰고
# 있는데 원래 공식은 반대쪽(먼 쪽) 해를 계산하고 있었던 것으로 추정.
# 원인을 기하학적으로 완전히 재도출하는 대신, 이 실측 상수로 직접
# 보정한다(보정 전: 원래 값 대비 최대 오차 ~3.0rad -> 보정 후 최대 오차
# ~0.043rad).
_BASE_ANGLE_EMPIRICAL_CORRECTION = 2.9439


def compute_base_angle(target_x: float, target_y: float) -> float:
    """target (x, y) -> base_joint가 향해야 할 각도.

    radius = hypot(x, y), x0 = sqrt(radius^2 - Y0^2) (shoulder/elbow/wrist가
    base=0에서 만들어야 하는 가상의 X), base_angle = atan2(y, x) - atan2(Y0, x0)
    에서 _BASE_ANGLE_EMPIRICAL_CORRECTION만큼 실측 보정 후 (-pi, pi]로 wrap.
    """
    radius = math.hypot(target_x, target_y)
    x0 = math.sqrt(max(radius * radius - BASE_ROTATION_Y_OFFSET_M ** 2, 0.0))
    raw = (
        math.atan2(target_y, target_x)
        - math.atan2(BASE_ROTATION_Y_OFFSET_M, x0)
        - _BASE_ANGLE_EMPIRICAL_CORRECTION
    )
    return (raw + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_base_pitch(base_angle: float, pitch: float) -> Sequence[float]:
    """Rotation ``Rz(base_angle) * Q0 * Rz(pitch)`` in ROS ``(x, y, z, w)`` order.

    compute_fk 실측으로 도출한 공식(위 주석 참고). pitch는 shoulder+elbow+wrist
    합이며, base_angle은 compute_base_angle()로 구한다.
    """
    cb, sb = math.cos(base_angle / 2.0), math.sin(base_angle / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    p = 0.5 * (cp + sp)
    q = 0.5 * (cp - sp)
    return (cb * p - sb * q, sb * p + cb * q, sb * p - cb * q, sb * q + cb * p)


# [추가, 2026-08-31] compute_fk 실측(STAND 자세, base=shoulder=elbow=wrist=0)으로
# 확인한 wrist_link -> tcp_link 로컬 오프셋(wrist_link 자신의 프레임 기준,
# base_angle/pitch와 무관한 상수): (-0.240, 0, 0.0313) = -(L3+tcp_off)=
# -(0.150+0.090), 0, BASE_ROTATION_Y_OFFSET_M와 정확히 일치(기하학적으로 검증됨).
# _solve_ik_multi_seed가 실제로 IK를 요청하는 대상은 wrist_link인데, _run_sequence가
# 넘기는 목표는 그리퍼가 실제로 닿아야 할 지점(=tcp_link 목표, 카메라가 찾은
# 박스 위치)이다 - 이 차이를 안 보정하면 "wrist_link를 그 지점까지 보내라"는
# 잘못된 명령이 되어(예: 지면 목표면 wrist를 지면 속으로 박으라는 뜻) 항상
# 도달 불가능했다. _move_arm이 pitch 후보마다 이 오프셋을 그 pitch의 orientation
# 으로 회전시켜서 tcp 목표를 wrist 목표로 변환한다.
WRIST_TO_TCP_LOCAL_OFFSET = (-0.240, 0.0, 0.0313)

# URDF의 base 축 -> shoulder 축(0.1039m), 두 arm link(0.180/0.220m),
# wrist -> TCP 보정 벡터의 길이를 모두 같은 방향으로 둔 보수적 상한이다.
# 실제 자세/관절 리미트는 이보다 작을 수 있지만 이보다 먼 TCP 목표에는 IK 해가
# 절대 없다. 3.8mm의 모델/측정 여유를 포함해 기본 파라미터를 0.720m로 둔다.
MODEL_MAX_TCP_DISTANCE_M = 0.1039 + 0.180 + 0.220 + math.hypot(*WRIST_TO_TCP_LOCAL_OFFSET)
DEFAULT_WORKSPACE_MAX_TCP_DISTANCE_M = MODEL_MAX_TCP_DISTANCE_M + 0.0038
IK_COLLISION_ERROR_CODES = {
    -10: "START_STATE_IN_COLLISION",
    -12: "GOAL_IN_COLLISION",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
}


class GripperCloseResult(Enum):
    """Outcome needed by the grasp retry state machine."""

    SUCCESS = auto()
    MISS = auto()
    EXECUTION_FAILED = auto()


def is_within_workspace_distance(
    position: Sequence[float], max_tcp_distance_m: float,
) -> bool:
    """Reject only goals beyond the model's absolute TCP-distance upper bound.

    A non-positive limit intentionally disables this fast pre-check for model
    calibration experiments. Passing this check does not promise IK success;
    joint limits, orientation, and collisions still apply.
    """
    if max_tcp_distance_m <= 0.0:
        return True
    return math.dist((0.0, 0.0, 0.0), position) <= max_tcp_distance_m


def target_forward_distance(position: Sequence[float]) -> float:
    """Return the base-actuator X-axis distance used for straight approach.

    The current arm mounting reports targets in front as negative X (for
    example ``x=-0.850``), so the forward *distance* is its magnitude.  Y and
    Z intentionally do not affect the chassis straight-ahead command.
    """
    return abs(float(position[0]))


def target_requires_approach(
    position: Sequence[float], grasp_start_x_distance_m: float,
) -> bool:
    """Return whether the target is too far ahead to begin grasping.

    A non-positive threshold disables the gate. Passing the X-distance gate
    does not guarantee IK; lateral offset, height, orientation, collision and
    joint limits are still validated by MoveIt before any arm motion.
    """
    return (
        grasp_start_x_distance_m > 0.0
        and target_forward_distance(position) > grasp_start_x_distance_m
    )


def camera_depth_allows_direct_grasp(
    camera_depth_m: float, direct_grasp_max_camera_depth_m: float,
) -> bool:
    """Use the YOLO bbox-center depth as the requested direct-arm gate.

    A non-positive limit disables this override. Invalid/non-positive camera
    depth never bypasses the base-frame X approach gate.
    """
    return (
        direct_grasp_max_camera_depth_m > 0.0
        and math.isfinite(float(camera_depth_m))
        and 0.0 < float(camera_depth_m) <= direct_grasp_max_camera_depth_m
    )


def apply_grasp_target_correction(
    detected_xyz: Sequence[float], grasp_offset: Sequence[float],
    use_fixed_ground_z: bool, ground_z: float,
) -> tuple[float, float, float]:
    """Return the exact base-frame TCP target consumed by the IK sequence."""
    return (
        float(detected_xyz[0]) + float(grasp_offset[0]),
        float(detected_xyz[1]) + float(grasp_offset[1]),
        (
            float(ground_z) + float(grasp_offset[2])
            if use_fixed_ground_z
            else float(detected_xyz[2]) + float(grasp_offset[2])
        ),
    )


def _rotate_vector_by_quaternion(q: Sequence[float], v: Sequence[float]) -> Sequence[float]:
    """v(x,y,z)를 단위 쿼터니언 q(x,y,z,w)로 정방향 회전(q*v*q^-1과 동일,
    STAND 자세 FK 실측으로 결과 검증됨)."""
    ux, uy, uz, w = q
    vx, vy, vz = v
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    return (
        vx + w * tx + (uy * tz - uz * ty),
        vy + w * ty + (uz * tx - ux * tz),
        vz + w * tz + (ux * ty - uy * tx),
    )


class MaruIKNode(Node):
    def __init__(self):
        super().__init__("maru_ik_node")
        self._joint_calibration = JointCalibration()
        self._arm_joint_limits = load_arm_joint_limits(self._joint_calibration)
        self.get_logger().info(
            f"IK/hardware joint limits loaded from {self._joint_calibration.path}"
        )
        group = ReentrantCallbackGroup()
        for name, default in (
            ("target_topic", "/arm/target_point"),
            ("target_depth_topic", "/arm/target_depth_m"),
            ("drive_detected_topic", "/drive/supplybox_detected"),
            ("picking_state_topic", "/picking"),
            ("calculation_failure_topic", "/arm/calculation_failed"),
            ("target_point_base_topic", "/arm/target_point_base"),
            ("target_distance_topic", "/arm/target_distance_m"),
            ("manual_override_topic", "/control/arm_manual_override"),
            ("manual_override_timeout_sec", 1.5),
            # [수정] "base_link"는 base_joint의 자식 링크라 base_joint 회전과
            # 함께 도는 프레임이다 - SRDF의 arm 체인 실제 루트는 base_actuator
            # (base_joint의 부모, 고정)다. planning_frame을 base_link로 두면
            # compute_ik/compute_fk가 "이 좌표가 무슨 뜻인지"를 요청 시점의
            # 실제 base_joint 값(회전하는 프레임 자신)에 의존해서 해석하게
            # 되어, 로봇이 물리적으로 움직여 base_joint가 바뀔 때마다 같은
            # 좌표의 실제 의미가 달라지는 자기참조적 버그가 생긴다 - 이번
            # 세션에서 compute_ik/compute_fk 왕복 검증으로 실측 확인함(고정된
            # base_actuator 기준으로 바꾸니 방금까지 계속 실패하던 타겟이 즉시
            # 성공). "도달 불가 영역"처럼 보였던 재현성 있는 실패들은 대부분
            # 이 프레임 버그의 증상이었다.
            ("planning_frame", "base_actuator"),
            ("pregrasp_offset_z", 0.10),
            # /arm/target_point는 perception 단계에서 이미 base_actuator 기준
            # supplybox 중심점으로 정의한다. 여기서 -95/2mm를 다시 적용하면
            # 중심점이 바닥면으로 내려가는 이중 보정이므로 기본 오프셋은 0이다.
            ("grasp_offset_x", 0.0),
            ("grasp_offset_y", 0.0),
            ("grasp_offset_z", 0.0),
            # 기본 동작은 인식된 중심점의 Z를 그대로 사용한다. true로 명시한
            # 진단/고정 바닥 운용에서만 detected Z 대신 ground_z를 사용한다.
            ("use_fixed_ground_z", False),
            ("ground_z", -0.35),
            # [갱신, 2026-08-31] 이전 기본값(-2.5, fallback -3.1~-0.9)은 완전히
            # 잘못된 부호/범위였던 것으로 확인됨. capture_arm_pose.py로 지면
            # supplybox를 실제로 손으로 잡는 자세 41개를 캡처하고 compute_fk로
            # pitch(=shoulder+elbow+wrist 합) vs 실제 도달 위치(x0,z)를 대조한
            # 결과, 이 팔은 4DOF로 위치(3)+자세(2, base_angle/pitch)를
            # 사실상 5개 제약으로 푸는 구조라 **같은 위치라도 여러 pitch가
            # 유효**하다는 게 확인됨(elbow-up/down류 중복해 - 예: x0≈0.29~0.30,
            # z≈-0.09~-0.10 근방에서도 실측 pitch가 2.90~3.20까지 갈림). 즉
            # "위치마다 유일한 pitch"가 아니라 "후보를 촘촘히 스캔해서 그중
            # 유효한 걸 찾는" 접근이 맞다 - 41개 샘플 전체의 실측 pitch 범위
            # (2.71~3.37)를 0.1 간격으로 촘촘히 덮도록 재구성했다.
            # pregrasp/descend/lift가 서로 다른 z(높이)를 목표하므로 한
            # pitch로 전부 안 풀릴 수 있어 _move_arm이 이 목록을 순서대로
            # 재시도한다.
            ("approach_pitch", 2.95),
            ("approach_pitch_fallbacks", [3.05, 2.85, 3.15, 2.75, 3.25, 2.65, 3.35]),
            # 모델 링크 길이로 만든 단순 구면 거리 검사는 실기에서 닿는
            # (-0.691, -0.031, -0.397)m 목표도 0.720m 밖이라고 오판해 IK를
            # 전혀 시도하지 않고 forward_command를 발행했다. 기본값 0.0으로
            # 비활성화하고 실제 IK를 최종 도달성 판정기로 사용한다. 실기 치수
            # 재검증 후 빠른 거절이 필요할 때만 양수로 명시해서 켠다.
            ("workspace_max_tcp_distance_m", 0.0),
            # 첫 좌표를 곧바로 /picking=true로 고정하지 않는다. 보정된
            # base_actuator 기준 정면 X축 거리가 이 값보다 멀면
            # /arm/forward_command만 발행하고 다음 좌표를 계속 받는다.
            # /arm/target_distance_m의 3-D 직선거리는 진단용으로만 유지한다.
            # 0 이하면 이 접근 게이트를 끈다.
            ("grasp_start_x_distance_m", 0.21),
            # YOLO bbox 중심의 camera optical Z가 이 거리 이하면 위 X축
            # 접근 게이트를 우회하고 즉시 팔 IK를 시도한다. 사용자가 확정한
            # 1.0m 기준이며, 0 이하면 이 우회 조건을 끈다.
            ("direct_grasp_max_camera_depth_m", 1.0),
            ("forward_command_min_interval_sec", 2.0),
            # 노드/컨트롤러가 뜬 직후 팔 카메라 시야를 먼저 확보한다.
            # 단순 wall-clock 1회 호출이 아니라 유효 joint_states와 AUTO 권한이
            # 준비될 때까지 기다리고, 이동 실패 시 다음 timer tick에 재시도한다.
            ("startup_grasp_wait_delay_sec", 2.0),
            # 시작 자세는 MoveIt 기동 지연/교착과 무관하게 arm_controller에
            # 직접 한 waypoint를 보낸다. 알려진 안전 자세로 이동하는 시간.
            ("startup_grasp_wait_duration_sec", 3.0),
            # true면 control_bringup의 TimerAction + move_to_named_pose.py가
            # 시작 자세를 담당하고 완료 토픽이 올 때까지 인식을 차단한다.
            ("startup_grasp_wait_external", False),
            ("startup_pose_complete_topic", "/arm/startup_pose_complete"),
            # 실기 후보 자세 선택. legacy는 기존 자세를 보존하며,
            # grasp_wait1~5는 grasp_wait_presets.py의 캡처/평균값을 사용한다.
            ("grasp_wait_preset", DEFAULT_GRASP_WAIT_PRESET),
            # 성공한 IK 해를 가까운 다음 목표의 seed로 재사용한다. 0이면 캐시를
            # 끄고 기존의 current + 고정 IK_SEEDS 동작만 사용한다.
            ("ik_solution_cache_size", 16),
            ("ik_cached_seed_count", 3),
            # 한 target/pitch에서 긴 랜덤 탐색 대신, 우선순위가 높은 seed를
            # 짧게 시도한다. 거리별 우선순위를 적용한 후
            # current + static 5개를 쓰도록 6을 기본으로 쓴다.
            ("ik_max_seed_attempts", 6),
            ("ik_request_timeout_sec", 0.5),
            # moveit: /compute_ik(KDL, 지역 수치해법 - seed에 따라 같은 목표도
            # 풀리거나 안 풀림, 서비스 왕복도 seed당 0.5~2s). numerical: 이
            # 프로세스 안의 유한차분 Jacobian DLS(마찬가지로 반복/seed 의존
            # 수치해법). analytic: [2026-09-03 신규] 이 팔이 "베이스 요 1개 +
            # 동일 평면 피치 3개" 구조라는 걸 이용한 닫힌해(law-of-cosines) -
            # 반복/seed가 전혀 없어 seed 민감도 문제 자체가 사라지고, 호출당
            # 1ms 미만이라 pitch 후보를 훨씬 촘촘히(수백 개) 스윕해도 빠르다.
            # analytic_grasp_planner.py에서 무작위 관절값 2만 개 왕복
            # 검증(FK->IK->FK) 오차 0, 실측 base_actuator 좌표(compute_fk)와도
            # 교차검증 완료 - 이번 세션에서 seed 의존성 때문에 반복적으로
            # 겪은 "가까운 곳도 못 찾음" 문제의 근본 해결책이라 기본값으로
            # 바꾼다. 어느 백엔드든 해를 찾은 다음 실제 실행은 MoveIt
            # move_to_configuration을 거쳐 planning scene 충돌검사와
            # ros2_control trajectory 실행을 그대로 사용한다(analytic도 예외
            # 아님 - 위치/자세만 닫힌해로 구하고 충돌 검사는 그대로 받음).
            ("ik_solver_backend", "analytic"),
            ("numerical_ik_max_iterations", 250),
            ("numerical_ik_position_tolerance_m", 0.003),
            ("numerical_ik_pitch_tolerance_rad", 0.025),
            ("numerical_ik_damping", 0.025),
            # 진단 때만 false로 내리고, 정상 운용에서는 IK 단계부터 충돌 해를
            # 거절한다. 이후 MoveIt 플래닝도 별도로 충돌 검사를 수행한다.
            ("ik_avoid_collisions", True),
            # [추가] 정밀 목표의 모든 pitch/seed가 실패했을 때 "아예 안
            # 움직이는" 대신, 현재 TCP와 목표 사이를 보간한 더 가까운 지점 중
            # IK가 풀리는 곳으로 실제 이동을 시도한다(target에 가까운 fraction
            # 부터 순서대로). 원래 목표 도달은 여전히 실패로 처리하고
            # 재시도/접근 상태머신은 그대로 진행한다 - "완전 정지"보다
            # "최대한 접근"이 낫다는 사용자 요청에 따른 폴백.
            ("ik_partial_reach_enabled", True),
            ("ik_partial_reach_fractions", [0.85, 0.7, 0.55, 0.4, 0.25, 0.15]),
            # [추가, 2026-09-03] 위 fraction 후보가 전부 실패해도 이동 없이
            # 포기하지 않는다 - "멀더라도 최대한 근사하게 이동" 요청에 따라
            # 0.0(현재 위치, 사실상 항상 풀림)과 가장 작은 실패 fraction 사이를
            # 이분탐색해서 IK가 풀리는 가장 가까운 지점을 강제로 찾는다.
            ("ik_partial_reach_bisection_iterations", 6),
            # 정밀인식+IK 또는 빈 파지가 같은 접근 위치에서 이 횟수 연속
            # 실패하면 /arm/forward_command로 접근을 다시 요청한다.
            ("precision_failures_before_approach", 2),
            # drive가 forward_command를 받아 nudge를 끝낼 보수적 대기 시간.
            # 완료 ack 토픽이 생기면 이 시간 대기 대신 그 ack를 구독해야 한다.
            ("approach_retry_wait_sec", 1.0),
            ("settle_time_sec", 2.0),
            ("motion_timeout_sec", 20.0),
            ("return_home_after_grasp", False),
            # [추가] descend 후 그리퍼를 닫기 전에 터미널에서 y/n으로 사람이
            # 확인하게 하는 안전 게이트. 서플라이박스 상단면-카메라 평행도를
            # 자동으로 검증하는 기능은 depth 노이즈/보정 로직이 더 필요해서
            # 당장은 이 수동 확인으로 대체했었다.
            # [수정, 2026-08-31] 실기 검증 끝나서 기본값을 false(완전 자동)로
            # 되돌림 - 필요하면 -p confirm_before_close:=true로 다시 켤 수 있다.
            ("confirm_before_close", False),
            # [추가] 그리퍼 전류 피드백 파지 제어 - 고정 위치까지 닫는 게 아니라
            # 닫는 도중 gripper_joint 전류가 임계값을 넘으면(=물체에 닿아 부하
            # 걸림) 즉시 정지시킨다. gripper_current_threshold_ma=100.0mA는
            # 2026-08-30 gripper_current_logger.py/gripper_safe_close.py로 실기
            # 서플라이박스 대상 실측한 값. 파지 대상이 바뀌면(무게/재질) 같은
            # 도구로 재측정해서 갱신할 것 - 너무 낮으면 그냥 닫히기만 해도
            # 오검출, 너무 높으면 물체 파손/모터 과부하 전까지 못 멈춤.
            ("gripper_current_threshold_ma", 100.0),
            ("gripper_current_lsb_ma", 2.69),
        ):
            self.declare_parameter(name, default)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.target_depth_topic = str(
            self.get_parameter("target_depth_topic").value)
        self.drive_detected_topic = str(self.get_parameter("drive_detected_topic").value)
        self.picking_state_topic = str(self.get_parameter("picking_state_topic").value)
        self.calculation_failure_topic = str(
            self.get_parameter("calculation_failure_topic").value)
        self.target_point_base_topic = str(
            self.get_parameter("target_point_base_topic").value)
        self.target_distance_topic = str(
            self.get_parameter("target_distance_topic").value)
        self.manual_override_topic = str(
            self.get_parameter("manual_override_topic").value)
        self.manual_override_timeout_sec = max(
            0.5, float(self.get_parameter("manual_override_timeout_sec").value))
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.pregrasp_offset_z = float(self.get_parameter("pregrasp_offset_z").value)
        self.grasp_offset = tuple(float(self.get_parameter(n).value) for n in (
            "grasp_offset_x", "grasp_offset_y", "grasp_offset_z"))
        self.use_fixed_ground_z = bool(self.get_parameter("use_fixed_ground_z").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.approach_pitch = float(self.get_parameter("approach_pitch").value)
        self.approach_pitch_fallbacks = [
            float(v) for v in self.get_parameter("approach_pitch_fallbacks").value]
        self.workspace_max_tcp_distance_m = float(
            self.get_parameter("workspace_max_tcp_distance_m").value)
        self.grasp_start_x_distance_m = float(
            self.get_parameter("grasp_start_x_distance_m").value)
        self.direct_grasp_max_camera_depth_m = float(
            self.get_parameter("direct_grasp_max_camera_depth_m").value)
        self.forward_command_min_interval_sec = max(
            0.0, float(self.get_parameter("forward_command_min_interval_sec").value))
        self.startup_grasp_wait_delay_sec = max(
            0.0, float(self.get_parameter("startup_grasp_wait_delay_sec").value))
        self.startup_grasp_wait_duration_sec = max(
            0.1, float(self.get_parameter("startup_grasp_wait_duration_sec").value))
        self.startup_grasp_wait_external = bool(
            self.get_parameter("startup_grasp_wait_external").value)
        self.startup_pose_complete_topic = str(
            self.get_parameter("startup_pose_complete_topic").value)
        self.grasp_wait_preset = str(
            self.get_parameter("grasp_wait_preset").value).strip().lower()
        self.grasp_wait_arm_joints = get_grasp_wait_preset(self.grasp_wait_preset)
        self.ik_solution_cache_size = max(
            0, int(self.get_parameter("ik_solution_cache_size").value))
        self.ik_cached_seed_count = max(
            0, int(self.get_parameter("ik_cached_seed_count").value))
        self.ik_max_seed_attempts = max(
            1, int(self.get_parameter("ik_max_seed_attempts").value))
        self.ik_request_timeout_sec = max(
            0.01, float(self.get_parameter("ik_request_timeout_sec").value))
        self.ik_solver_backend = str(
            self.get_parameter("ik_solver_backend").value).strip().lower()
        if self.ik_solver_backend not in ("moveit", "numerical", "analytic"):
            raise ValueError(
                "ik_solver_backend must be 'moveit', 'numerical', or "
                f"'analytic', got {self.ik_solver_backend!r}")
        self.numerical_ik_max_iterations = max(
            1, int(self.get_parameter("numerical_ik_max_iterations").value))
        self.numerical_ik_position_tolerance_m = max(
            1e-5, float(self.get_parameter("numerical_ik_position_tolerance_m").value))
        self.numerical_ik_pitch_tolerance_rad = max(
            1e-5, float(self.get_parameter("numerical_ik_pitch_tolerance_rad").value))
        self.numerical_ik_damping = max(
            1e-6, float(self.get_parameter("numerical_ik_damping").value))
        self.ik_avoid_collisions = bool(
            self.get_parameter("ik_avoid_collisions").value)
        self.ik_partial_reach_enabled = bool(
            self.get_parameter("ik_partial_reach_enabled").value)
        self.ik_partial_reach_fractions = [
            float(v) for v in self.get_parameter("ik_partial_reach_fractions").value
            if 0.0 < float(v) < 1.0
        ]
        self.ik_partial_reach_bisection_iterations = max(
            0, int(self.get_parameter("ik_partial_reach_bisection_iterations").value))
        self.precision_failures_before_approach = max(
            1, int(self.get_parameter("precision_failures_before_approach").value))
        self.approach_retry_wait_sec = max(
            0.0, float(self.get_parameter("approach_retry_wait_sec").value))
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.return_home = bool(self.get_parameter("return_home_after_grasp").value)
        self.confirm_before_close = bool(self.get_parameter("confirm_before_close").value)
        # [수정, 2026-09-02] "그랩 시퀀스 점유" 상태를 별도 threading.Lock()으로
        # 두면, "누가 점유 중인지/넘겨줄 대상이 있는지" 판단(_target_state_lock
        # 아래)과 실제 acquire/release가 서로 다른 락이라 그 사이에 경쟁이
        # 생긴다: A가 "넘겨줄 타겟 없음"으로 판단한 직후, B가 타겟을 latch하고
        # acquire를 시도했는데 A가 아직 release 전이라 실패 -> B는 "큐잉됨"만
        # 남기고 리턴 -> A가 그제서야 release. 이후로는 아무도 그 큐잉된
        # 타겟을 다시 못 잡아 _run_sequence가 영원히 시작되지 않는 채로
        # /picking=true만 남는 버그가 실기에서 확인됨(로그가 완전히 조용해진
        # 채 "Ignoring updated target_point..."만 반복). _busy_owned을
        # _target_state_lock이 보호하는 평범한 bool로 합쳐서, "점유 상태 확인"과
        # "점유권 이전/해제"를 같은 임계구역 안에서 원자적으로 처리한다
        # (_claim_busy/_release_or_handoff_busy 참고).
        self._busy_owned = False
        self._target_state_lock = threading.RLock()
        # 노드를 단독 실행해도 카메라 타겟을 바로 처리한다. 외부 안전/운영
        # 노드는 /control/auto_enabled=false를 publish해서 언제든 실행을
        # 중지할 수 있다.
        self._auto_enabled = True
        self._manual_override = False
        self._manual_override_last_seen = None
        # _move_arm이 마지막 호출에서 IK 해를 단 하나도 못 찾았는지(=진짜
        # 가동범위 밖) 표시. AUTO 비활성/모션 타임아웃 등 다른 실패 사유와
        # 구분해서, 진짜 "가동범위 밖"일 때만 forward_command를 쏘기 위함.
        self._last_ik_unreachable = False
        self._last_partial_reach_moved = False
        self._ik_collision_rejected = False
        self._precision_failure_count = 0
        self._picking_active = False
        self._latched_target = None
        self._pending_latched_sequence = False
        self._retry_latched_target = False
        # [추가, 2026-08-31] 구동부가 /drive/supplybox_detected로 "지면 박스
        # 발견"을 알리면 바로 GRASP_WAIT로 이동해둔다(주행 중엔 박스가 지면에
        # 있어 팔 카메라 시야 밖이라 미리 내려다보는 자세로 준비).
        # [수정, 2026-09-01] on_drive_detected -> GRASP_WAIT 사이의 HOME 경유
        # 단계를 제거했고(_run_grasp_wait_sequence), _run_sequence도 더 이상
        # HOME으로 먼저 이동하는 폴백 없이 항상 이 플래그를 소비만 한다 -
        # 그랩 시퀀스는 이제 오직 GRASP_WAIT를 거쳐서만 시작된다.
        self._at_grasp_wait = False
        # 시작 자세의 2초 지연은 프로세스 생성 시각이 아니라 실제 arm
        # controller action과 joint_states/AUTO가 모두 준비된 시각부터 센다.
        # /control/auto_enabled의 N번째 heartbeat만 세면 controller가 죽어
        # 있어도 시간이 지나 동작을 시도하므로 readiness를 직접 확인한다.
        self._startup_resources_ready_since = None
        self._startup_grasp_wait_started = False
        self._startup_grasp_wait_complete = False
        self._startup_grasp_wait_last_attempt = 0.0
        self._last_forward_command_time = 0.0
        self._latest_camera_depth_m = math.nan

        self.gripper_current_threshold_ma = float(
            self.get_parameter("gripper_current_threshold_ma").value)
        self.gripper_current_lsb_ma = float(self.get_parameter("gripper_current_lsb_ma").value)
        self._gripper_current_ma = 0.0
        self._gripper_position = None
        self._arm_positions = {}
        # (wrist_target, orientation, joint_solution). _busy가 파지 시퀀스를
        # 직렬화하므로 이 노드 안에서는 별도 lock 없이 안전하다.
        self._ik_solution_cache = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 시작 GRASP_WAIT은 MoveIt을 통해 실행하지만, 최종 실행 대상인
        # arm_controller action이 없으면 절대 움직일 수 없다. timer에서 이
        # client의 server_is_ready()를 readiness gate로만 사용한다.
        self._arm_controller_ready_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory", callback_group=group)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINT_NAMES, base_link_name=self.planning_frame,
            # [원복] end_effector_name="tcp_link"로 바꾸면 될 거라 판단했었는데
            # 실기 테스트에서 틀린 것으로 확인됨 - kinematics.yaml의 solver가
            # SRDF arm 그룹의 실제 tip_link(wrist_link)로만 IK를 풀 수 있어서,
            # pose 목표(move_to_pose)에서 target_link를 다르게 주면
            # "Unable to construct goal representation"으로 그냥 실패한다
            # (move_to_configuration처럼 IK를 안 타는 호출만 우연히 성공했었음).
            # tcp_link 기준 조작이 다시 필요하면, wrist_link IK를 그대로 쓰되
            # 호출 전에 wrist_link<->tcp_link 고정 오프셋(그리퍼 조인트값 고정
            # 가정)만큼 목표 좌표를 미리 보정하는 방식으로 가야 한다.
            end_effector_name="wrist_link", group_name="arm", callback_group=group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )
        self.gripper = MoveIt2(
            node=self, joint_names=GRIPPER_JOINT_NAMES, base_link_name="wrist_link",
            end_effector_name="tcp_link", group_name="gripper", callback_group=group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )
        self.create_subscription(PointStamped, self.target_topic, self.on_target, 10,
                                 callback_group=group)
        self.create_subscription(
            Float64, self.target_depth_topic, self.on_target_depth, 10,
            callback_group=group)
        self.create_subscription(
            Empty, self.drive_detected_topic, self.on_drive_detected, 10,
            callback_group=group)
        mode_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/control/auto_enabled', self.on_auto_enabled, mode_qos,
            callback_group=group)
        self.create_subscription(
            Bool, self.manual_override_topic, self.on_manual_override, mode_qos,
            callback_group=group)
        self.create_timer(0.2, self._check_manual_override_lease, callback_group=group)
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, qos_profile_sensor_data,
            callback_group=group)
        startup_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Empty, self.startup_pose_complete_topic, self.on_startup_pose_complete,
            startup_qos, callback_group=group)
        self.grasp_success_pub = self.create_publisher(Bool, "/arm/grasp_success", 10)
        self.picking_state_pub = self.create_publisher(
            Bool, self.picking_state_topic, mode_qos)
        # IK/workspace/planning 계산이 한 파지 시도에서 실패했음을 알리는
        # 1회성 이벤트. drive/mission 쪽에서 복귀 판단에 사용할 수 있다.
        self.calculation_failure_pub = self.create_publisher(
            Empty, self.calculation_failure_topic, 10)
        # UI/진단 좌표도 별도 perception 수식이 아니라 실제 IK 입력과 완전히
        # 동일한 보정 결과를 이 노드가 직접 발행한다.
        self.target_point_base_pub = self.create_publisher(
            PointStamped, self.target_point_base_topic, 10)
        self.target_distance_pub = self.create_publisher(
            Float64, self.target_distance_topic, 10)
        # 타겟 좌표가 팔 가동범위 밖일 때 "전진해도 됨" 트리거.
        self.forward_command_pub = self.create_publisher(Empty, "/arm/forward_command", 10)
        # 파지 시퀀스 완료 후 "피킹 완료, 주행 재개" 트리거(1회).
        self.picking_command_pub = self.create_publisher(Empty, "/arm/picking_command", 10)
        # 늦게 구독한 노드도 현재 대기/파지 상태를 즉시 알 수 있게 transient
        # local QoS로 초기 false를 남긴다.
        self.picking_state_pub.publish(Bool(data=False))
        self._startup_grasp_wait_timer = self.create_timer(
            0.2, self._on_startup_grasp_wait_timer, callback_group=group)
        self.get_logger().info(
            f"Ready: {self.target_topic} -> {self.planning_frame}, CAN via ros2_control. "
            f"Drive-detected trigger: {self.drive_detected_topic} -> GRASP_WAIT; "
            f"workspace pre-check={self.workspace_max_tcp_distance_m:.3f}m, "
            f"IK cache={self.ik_solution_cache_size}, max seeds={self.ik_max_seed_attempts}, "
            f"timeout={self.ik_request_timeout_sec:.2f}s, "
            f"IK backend={self.ik_solver_backend}, "
            f"collision checking={self.ik_avoid_collisions}, "
            f"grasp start X distance={self.grasp_start_x_distance_m:.3f}m, "
            f"direct grasp camera depth<={self.direct_grasp_max_camera_depth_m:.3f}m, "
            f"startup grasp_wait delay={self.startup_grasp_wait_delay_sec:.1f}s, "
            f"external startup pose={self.startup_grasp_wait_external}, "
            f"grasp_wait preset={self.grasp_wait_preset} "
            f"({self.grasp_wait_arm_joints}), "
            f"precision failures before approach={self.precision_failures_before_approach}."
        )

    def on_target_depth(self, msg: Float64) -> None:
        depth = float(msg.data)
        if math.isfinite(depth) and depth > 0.0:
            self._latest_camera_depth_m = depth

    def on_target(self, msg: PointStamped) -> None:
        if not self._auto_motion_allowed():
            self.get_logger().info('Ignoring target while AUTO is disabled or manually overridden.')
            return
        # 정상 파지 중에는 TF 조회조차 하지 않고 새 위치를 완전히 무시한다.
        # 아래 latch 직전에도 다시 검사해 동시 callback 경쟁을 막는다.
        with self._target_state_lock:
            if self._picking_active:
                self.get_logger().info(
                    "Ignoring updated target_point while /picking=true; using latched target.",
                    throttle_duration_sec=5.0,
                )
                return
            if not self._startup_grasp_wait_complete:
                self.get_logger().info(
                    "Ignoring target until startup GRASP_WAIT is complete.",
                    throttle_duration_sec=5.0,
                )
                return
        # /arm/target_point는 이제 base_actuator 좌표다. bbox 중심의 optical
        # depth는 summer_supply가 별도 Float64 토픽으로 먼저 발행한다.
        camera_depth_m = self._latest_camera_depth_m
        try:
            transformed_target = self._transform_target(msg)
        except Exception as exc:  # Never command a camera-frame point as base-frame data.
            self.get_logger().warn(f"Target TF transform failed; ignoring target: {exc}")
            self._publish_calculation_failure("target TF transform")
            return

        target = apply_grasp_target_correction(
            transformed_target, self.grasp_offset,
            self.use_fixed_ground_z, self.ground_z)

        # 진단 토픽은 latch보다 먼저 발행한다. 따라서 너무 먼 목표도 UI와
        # 구동부가 같은 보정 좌표/거리를 볼 수 있다.
        self._publish_corrected_target(target)
        target_distance = math.dist((0.0, 0.0, 0.0), target)
        forward_distance = target_forward_distance(target)
        direct_grasp = camera_depth_allows_direct_grasp(
            camera_depth_m, self.direct_grasp_max_camera_depth_m)
        if (
            not direct_grasp
            and target_requires_approach(target, self.grasp_start_x_distance_m)
        ):
            publish_forward = False
            now = time.monotonic()
            # TF 계산 중 다른 callback이 가까운 목표를 먼저 latch했으면 그
            # 파지를 방해하는 전진 명령을 절대 내리지 않는다.
            with self._target_state_lock:
                if self._picking_active:
                    return
                if now - self._last_forward_command_time >= self.forward_command_min_interval_sec:
                    self._last_forward_command_time = now
                    publish_forward = True
            if publish_forward:
                self.forward_command_pub.publish(Empty())
                self.get_logger().info(
                    f"Target forward X distance {forward_distance:.3f}m > "
                    f"{self.grasp_start_x_distance_m:.3f}m "
                    f"(3-D distance={target_distance:.3f}m); keeping /picking=false "
                    "and publishing /arm/forward_command.")
            return

        if direct_grasp:
            self.get_logger().info(
                f"Camera bbox depth {camera_depth_m:.3f}m <= "
                f"{self.direct_grasp_max_camera_depth_m:.3f}m; bypassing "
                f"base X approach gate (|x|={forward_distance:.3f}m) and starting arm IK.")

        # /picking=true의 의미는 "첫 위치 인식 성공 + 목표 고정 완료"다.
        # Reentrant callback에서 여러 target이 동시에 들어와도 첫 좌표 하나만
        # 채택하도록 별도 lock으로 latch 상태를 원자적으로 갱신한다.
        # [수정, 2026-09-02] latch(_picking_active/_pending_latched_sequence)와
        # busy 점유권 시도(_busy_owned)를 같은 _target_state_lock 임계구역
        # 안에서 같이 처리한다 - 예전에는 이 acquire가 lock 밖에서 별도로
        # 일어나서, _run_grasp_wait_sequence가 "넘겨줄 타겟 없음"으로 스냅샷을
        # 뜬 직후 여기서 latch+acquire가 끼어들면(그때는 아직 release 전이라
        # acquire 실패) 아무도 못 잡는 큐잉 타겟이 남아 /picking=true인 채
        # _run_sequence가 영원히 안 시작되는 버그가 있었다
        # (_busy_owned 초기화 주석/_release_or_handoff_busy 참고).
        with self._target_state_lock:
            if self._picking_active:
                self.get_logger().info(
                    "Ignoring updated target_point while /picking=true; using latched target.",
                    throttle_duration_sec=5.0,
                )
                return
            self._latched_target = target
            self._pending_latched_sequence = True
            self._set_picking_active(True)
            claimed_busy = not self._busy_owned
            if claimed_busy:
                self._busy_owned = True

        self.get_logger().info(
            "Recognition succeeded: target latched and /picking=true.")

        if not claimed_busy:
            # drive-detected에 따른 grasp_wait 이동 중 인식될 수 있다. 이
            # 좌표는 latch된 채로, grasp_wait worker가 끝나면
            # _release_or_handoff_busy가 busy 점유권을 파지 worker에 직접
            # 넘긴다.
            self.get_logger().info(
                "Target queued until the current grasp_wait motion completes.")
            return
        with self._target_state_lock:
            self._pending_latched_sequence = False
        if not self._start_sequence_worker(target):
            with self._target_state_lock:
                self._busy_owned = False

    def _start_sequence_worker(self, target) -> bool:
        """Start a worker while the caller owns ``_busy_owned``; return start success."""
        try:
            threading.Thread(
                target=self._run_sequence, args=(target,), daemon=True).start()
            return True
        except Exception as exc:
            # 스레드 생성 자체가 실패해 true 상태만 영구 잔류하지 않게 한다.
            self.get_logger().error(f"Could not start grasp worker: {exc}")
            self._publish_calculation_failure("worker start")
            self._finish_picking_cycle()
            return False

    def _set_picking_active(self, active: bool) -> None:
        """Publish the latched picking state only when it changes."""
        active = bool(active)
        if self._picking_active == active:
            return
        self._picking_active = active
        self.picking_state_pub.publish(Bool(data=active))
        self.get_logger().info(f"{self.picking_state_topic}={str(active).lower()}")

    def _claim_busy(self) -> bool:
        """Atomically claim grasp-sequence ownership if free.

        Must be used instead of touching ``_busy_owned`` directly so that a
        take-ownership attempt can never race with
        ``_release_or_handoff_busy``'s check-then-release on another thread
        (2026-09-02 fix; see ``_busy_owned`` init comment)."""
        with self._target_state_lock:
            if self._busy_owned:
                return False
            self._busy_owned = True
            return True

    def _release_or_handoff_busy(self) -> None:
        """Atomically decide whether to hand ownership straight to a target
        that got latched/queued while we were busy, or release it.

        The state check and the ownership write happen in one
        ``_target_state_lock`` critical section, so a concurrent
        ``_claim_busy``/latch from ``on_target`` can never land in the gap
        between "nothing to hand off" and the actual release (that gap was
        the root cause of the picking-forever-stuck bug: a queued target
        left with no one to start ``_run_sequence`` for it)."""
        queued_target = None
        with self._target_state_lock:
            if (
                (self._retry_latched_target or self._pending_latched_sequence)
                and self._picking_active
                and self._auto_motion_allowed()
                and self._latched_target is not None
            ):
                queued_target = self._latched_target
                self._retry_latched_target = False
                self._pending_latched_sequence = False
            else:
                self._busy_owned = False
        if queued_target is not None and not self._start_sequence_worker(queued_target):
            with self._target_state_lock:
                self._busy_owned = False

    def _finish_picking_cycle(self) -> None:
        """Return to target-accepting state after success or completed recovery."""
        with self._target_state_lock:
            self._precision_failure_count = 0
            self._retry_latched_target = False
            self._latched_target = None
            self._pending_latched_sequence = False
            self._set_picking_active(False)

    def _wait_for_fresh_recognition(self) -> None:
        """Drop the old target but preserve the current precision failure count."""
        with self._target_state_lock:
            self._retry_latched_target = False
            self._latched_target = None
            self._pending_latched_sequence = False
            self._set_picking_active(False)

    def _publish_calculation_failure(self, reason: str) -> None:
        """Publish one event for an IK/workspace/planning calculation failure."""
        self.calculation_failure_pub.publish(Empty())
        self.get_logger().warn(
            f"Published {self.calculation_failure_topic}: {reason}")

    def _publish_corrected_target(self, target) -> None:
        """Publish the same corrected base-frame target passed to MoveIt IK."""
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.planning_frame
        point.point.x, point.point.y, point.point.z = map(float, target)
        self.target_point_base_pub.publish(point)
        self.target_distance_pub.publish(Float64(data=math.dist((0.0, 0.0, 0.0), target)))

    def on_drive_detected(self, _msg: Empty) -> None:
        """구동부가 지면 서플라이박스를 먼저 발견하면(팔 카메라 시야 밖) 바로
        GRASP_WAIT로 이동해서 팔 카메라가 지면을 보게 준비한다(HOME 경유 없음
        - [수정, 2026-09-01] 그랩 시퀀스 진입 전 불필요한 중간 정차였다).
        이미 GRASP_WAIT에 있거나(_at_grasp_wait) 다른 시퀀스가 진행 중이면
        무시한다 - 어차피 다음 target_point 처리에서 그대로 활용된다."""
        if not self._auto_motion_allowed():
            self.get_logger().info(
                'Ignoring drive-detected trigger while AUTO is disabled or manually overridden.')
            return
        # 시작 자세는 D455 검출 신호와 무관하게 launch의 named-pose 동작이
        # 한 번만 수행한다. 그 완료 전 D455 신호는 선점하지 못하게 무시한다.
        if not self._startup_grasp_wait_complete:
            self.get_logger().info(
                "Ignoring D455 /drive/supplybox_detected until startup GRASP_WAIT completes.",
                throttle_duration_sec=5.0,
            )
            return
        if self._at_grasp_wait:
            return
        if not self._claim_busy():
            self.get_logger().info(
                "Grasp sequence is already active; ignoring drive-detected trigger.")
            return
        try:
            threading.Thread(target=self._run_grasp_wait_sequence, daemon=True).start()
        except Exception as exc:
            self.get_logger().error(f"Could not start grasp_wait worker: {exc}")
            with self._target_state_lock:
                self._busy_owned = False

    def on_startup_pose_complete(self, _msg: Empty) -> None:
        """Release perception only after the external named-pose tool succeeds."""
        if not self.startup_grasp_wait_external or self._startup_grasp_wait_complete:
            return
        with self._target_state_lock:
            self._at_grasp_wait = True
            self._precision_failure_count = 0
            self._startup_grasp_wait_complete = True
        self._startup_grasp_wait_timer.cancel()
        self.get_logger().info(
            f"External startup pose completed ({self.startup_pose_complete_topic}); "
            "accepting approach/grasp targets.")

    def _on_startup_grasp_wait_timer(self) -> None:
        """Enter GRASP_WAIT once, two seconds after startup resources are ready.

        A failed/missing-state attempt is retried instead of permanently losing
        the one-shot startup motion.
        """
        if self.startup_grasp_wait_external:
            return
        if self._startup_grasp_wait_complete or self._startup_grasp_wait_started:
            return
        if not self._auto_motion_allowed():
            self._startup_resources_ready_since = None
            return
        if self._at_grasp_wait:
            self._startup_grasp_wait_complete = True
            self._startup_grasp_wait_timer.cancel()
            return
        if any(name not in self._arm_positions for name in ARM_JOINT_NAMES):
            self._startup_resources_ready_since = None
            self.get_logger().info(
                "Startup GRASP_WAIT is waiting for complete arm joint_states.",
                throttle_duration_sec=5.0,
            )
            return
        now = time.monotonic()
        if not self._arm_controller_ready_client.server_is_ready():
            self._startup_resources_ready_since = None
            self.get_logger().warn(
                "Startup GRASP_WAIT is waiting for "
                "/arm_controller/follow_joint_trajectory action server; arm cannot move yet.",
                throttle_duration_sec=5.0,
            )
            return
        if self._startup_resources_ready_since is None:
            self._startup_resources_ready_since = now
            self.get_logger().info(
                "Startup resources ready; moving to selected GRASP_WAIT after "
                f"{self.startup_grasp_wait_delay_sec:.1f}s.")
            return
        if now - self._startup_resources_ready_since < self.startup_grasp_wait_delay_sec:
            return
        # move_group가 아직 올라오는 중일 때 0.2초마다 action goal을 난사하지
        # 않는다. 첫 mock 기동 검증에서 이 경우를 확인해 1초 간격으로 제한한다.
        if now - self._startup_grasp_wait_last_attempt < 1.0:
            return
        if not self._claim_busy():
            return
        self._startup_grasp_wait_last_attempt = now
        self._startup_grasp_wait_started = True
        try:
            threading.Thread(
                target=self._run_startup_grasp_wait_sequence, daemon=True).start()
        except Exception as exc:
            self._startup_grasp_wait_started = False
            self.get_logger().error(f"Could not start startup grasp_wait worker: {exc}")
            with self._target_state_lock:
                self._busy_owned = False

    def _run_startup_grasp_wait_sequence(self) -> None:
        succeeded = False
        try:
            self.get_logger().info("Startup delay complete; moving to GRASP_WAIT.")
            # 시작 자세는 IK가 필요 없는 검증된 관절 자세다. MoveIt의
            # /move_action이 늦게 뜨거나 멈춰도 시작 단계가 영원히 잠기지
            # 않도록 arm_controller에 직접 보낸다. 이후 인식 좌표 이동은
            # 기존대로 MoveIt/수치 IK 경로를 사용한다.
            succeeded = self._move_to_grasp_wait_direct()
            if succeeded:
                self._at_grasp_wait = True
                self._precision_failure_count = 0
                self._startup_grasp_wait_complete = True
                self._startup_grasp_wait_timer.cancel()
                self.get_logger().info(
                    "Startup GRASP_WAIT complete - accepting approach/grasp targets.")
            else:
                self.get_logger().warn(
                    "Startup GRASP_WAIT failed; it will be retried while AUTO remains enabled.")
        finally:
            self._startup_grasp_wait_started = False
            self._release_or_handoff_busy()

    def _move_to_grasp_wait_direct(self) -> bool:
        """Send the selected startup pose straight to arm_controller.

        This mirrors the proven ``move_to_named_pose.py`` action path while
        preserving the measured base joint. It is intentionally limited to
        startup; target-dependent motion still goes through MoveIt planning.
        """
        if not self._auto_motion_allowed():
            return False
        missing = [name for name in ARM_JOINT_NAMES if name not in self._arm_positions]
        if missing:
            self.get_logger().error(
                "Cannot directly move to startup grasp_wait; missing joint states: "
                + ", ".join(missing))
            return False
        if not self._arm_controller_ready_client.server_is_ready():
            self.get_logger().error(
                "Cannot directly move to startup grasp_wait; arm controller action is absent.")
            return False

        target = [
            self.grasp_wait_arm_joints.get(name, self._safe_current(name))
            for name in ARM_JOINT_NAMES
        ]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = target
        duration_ns = int(round(self.startup_grasp_wait_duration_sec * 1_000_000_000))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        goal.trajectory.points.append(point)

        self.get_logger().warn(
            f"DIRECT startup command -> /arm_controller/follow_joint_trajectory: "
            f"preset={self.grasp_wait_preset}, target={target}, "
            f"duration={self.startup_grasp_wait_duration_sec:.1f}s")
        send_future = self._arm_controller_ready_client.send_goal_async(goal)
        send_deadline = time.monotonic() + 5.0
        while not send_future.done():
            if not self._auto_motion_allowed() or time.monotonic() >= send_deadline:
                self.get_logger().error("DIRECT startup goal response timeout/cancelled.")
                return False
            time.sleep(0.01)
        if send_future.exception() is not None:
            self.get_logger().error(
                f"DIRECT startup goal send failed: {send_future.exception()}")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DIRECT startup goal was rejected by arm_controller.")
            return False

        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + max(
            self.motion_timeout_sec, self.startup_grasp_wait_duration_sec + 5.0)
        while not result_future.done():
            if not self._auto_motion_allowed():
                goal_handle.cancel_goal_async()
                return False
            if time.monotonic() >= result_deadline:
                self.get_logger().error("DIRECT startup execution timeout; cancelling goal.")
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.02)
        if result_future.exception() is not None:
            self.get_logger().error(
                f"DIRECT startup result failed: {result_future.exception()}")
            return False
        wrapped = result_future.result()
        if (
            wrapped is None
            or wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            status = None if wrapped is None else wrapped.status
            error_code = None if wrapped is None else wrapped.result.error_code
            self.get_logger().error(
                f"DIRECT startup execution failed: status={status}, error_code={error_code}")
            return False

        self.get_logger().info("DIRECT startup GRASP_WAIT controller execution succeeded.")
        return True

    def _run_grasp_wait_sequence(self) -> None:
        try:
            if not self._move_to_grasp_wait():
                self.get_logger().warn("Drive-detected trigger: GRASP_WAIT 진입 실패.")
                return
            self._at_grasp_wait = True
            self._precision_failure_count = 0
            self.get_logger().info("GRASP_WAIT 자세 진입 완료 - 팔 카메라 타겟 대기 중.")
        finally:
            # grasp_wait 이동 중 이미 인식에 성공했다면 lock을 풀어 새 좌표와
            # 경쟁시키지 않고, 고정된 첫 목표로 파지 worker를 바로 시작한다
            # (_release_or_handoff_busy가 이 판단과 점유권 이전/해제를 원자적으로
            # 처리 - _busy_owned 초기화 주석의 2026-09-02 버그 참고).
            self._release_or_handoff_busy()

    def on_auto_enabled(self, msg: Bool) -> None:
        self._auto_enabled = bool(msg.data)
        if not self._auto_enabled:
            self._at_grasp_wait = False
            self._finish_picking_cycle()
            if self._busy_owned:
                self.arm.cancel_execution()
                self.gripper.cancel_execution()

    def _auto_motion_allowed(self) -> bool:
        return self._auto_enabled and not self._manual_override

    def on_manual_override(self, msg: Bool) -> None:
        active = bool(msg.data)
        self._manual_override_last_seen = time.monotonic() if active else None
        if self._manual_override == active:
            return
        self._manual_override = active
        self.get_logger().info(
            f"Manual controller override={str(active).lower()} "
            f"({self.manual_override_topic})")
        if active:
            # Direct FollowJointTrajectory tools become the sole command owner.
            # Drop any latched perception target and cancel MoveIt before they send.
            self._at_grasp_wait = False
            self._finish_picking_cycle()
            if self._busy_owned:
                self.arm.cancel_execution()
                self.gripper.cancel_execution()

    def _check_manual_override_lease(self) -> None:
        """Release manual ownership if its heartbeat publisher disappeared."""
        if not self._manual_override or self._manual_override_last_seen is None:
            return
        if time.monotonic() - self._manual_override_last_seen <= self.manual_override_timeout_sec:
            return
        self._manual_override = False
        self._manual_override_last_seen = None
        self.get_logger().warn(
            "Manual controller override heartbeat timed out; AUTO command ownership restored.")

    def on_joint_states(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINT_NAMES and math.isfinite(position):
                self._arm_positions[name] = float(position)
        try:
            idx = msg.name.index("gripper_joint")
        except ValueError:
            return
        if idx < len(msg.position) and math.isfinite(msg.position[idx]):
            self._gripper_position = float(msg.position[idx])
        if idx >= len(msg.effort):
            return
        # effort_state_는 dynamixel_hardware_interface에서 raw Present Current
        # 값 그대로 나온다(변환계수 1.0) - 1 LSB ≈ 2.69mA를 곱해야 실제 mA.
        self._gripper_current_ma = abs(msg.effort[idx]) * self.gripper_current_lsb_ma

    def _transform_target(self, msg: PointStamped):
        if not msg.header.frame_id:
            # Fake publisher intentionally uses an empty frame for already-base-frame points.
            return (msg.point.x, msg.point.y, msg.point.z)
        if msg.header.frame_id == self.planning_frame:
            return (msg.point.x, msg.point.y, msg.point.z)
        transform_time = (
            Time.from_msg(msg.header.stamp)
            if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
            else Time()
        )
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, msg.header.frame_id, transform_time,
            timeout=Duration(seconds=0.5))
        point = do_transform_point(msg, transform)
        return (point.point.x, point.point.y, point.point.z)

    def _run_sequence(self, detected_xyz) -> None:
        sequence_start = time.monotonic()
        try:
            # on_target에서 TF와 grasp/fixed-ground 보정을 끝낸 동일 좌표를
            # /arm/target_point_base에 발행하고 latch했다. 재시도에서도 다시
            # 보정하지 않고 그 값을 그대로 IK에 사용한다.
            x, y, z = detected_xyz
            # base_joint의 실질 회전축은 base_link 기준 Z(전방)이다(compute_fk
            # 실측; quaternion_from_base_pitch 위 주석 참고). x,y로부터 base_angle을
            # 구하고, pitch(=shoulder+elbow+wrist 합)는 _move_arm이 후보 목록을
            # 순서대로 시도한다.
            base_angle = compute_base_angle(x, y)
            self.get_logger().info(f"Grasp point in {self.planning_frame}: ({x:.3f}, {y:.3f}, {z:.3f})")

            # grasp와 pregrasp 모두 TCP 목표다. 어느 하나라도 절대 거리 상한을
            # 넘으면 seed/pitch를 바꿔도 해가 없으므로 긴 IK 재시도를 생략한다.
            workspace_targets = ((x, y, z), (x, y, z + self.pregrasp_offset_z))
            outside = [target for target in workspace_targets if not is_within_workspace_distance(
                target, self.workspace_max_tcp_distance_m)]
            if outside:
                self.get_logger().warn(
                    "Target outside absolute TCP workspace bound "
                    f"({self.workspace_max_tcp_distance_m:.3f}m): {outside}; skipping IK.")
                self._register_precision_failure(
                    "workspace pre-check", calculation_failed=True)
                return

            # [수정, 2026-09-01] HOME으로 먼저 이동하는 폴백을 없앴다 - 그랩
            # 시퀀스는 이제 항상 on_drive_detected -> GRASP_WAIT를 거쳐서만
            # 시작되므로(_run_grasp_wait_sequence), 여기서는 그 상태 플래그만
            # 소비하고 곧바로 pregrasp로 들어간다.
            self._at_grasp_wait = False

            # 먼저 팔의 pre-grasp 경로가 가능한지 확인/실행한다. 이전에는
            # 그리퍼를 먼저 열어 두고 플래닝이 실패해, 사용자 입장에서는
            # "그리퍼만 움직인다"고 보이는 부작용이 있었다.
            if not self._move_arm((x, y, z + self.pregrasp_offset_z), base_angle, "pregrasp"):
                self._register_precision_failure(
                    "pregrasp IK/planning", calculation_failed=True,
                    reobserve_after_partial_reach=self._last_partial_reach_moved)
                return
            if not self._move_gripper(GRIPPER_OPEN, "open"):
                self._register_precision_failure("gripper open execution")
                return
            # pymoveit2's Cartesian helper performs its own blocking spin, which is
            # unsafe while this node is already in a MultiThreadedExecutor.  Asking
            # move_group for this short segment keeps planning/execution asynchronous
            # and still collision-checks the complete path.
            if not self._move_arm((x, y, z), base_angle, "descend"):
                self._register_precision_failure(
                    "descend IK/planning", calculation_failed=True,
                    reobserve_after_partial_reach=self._last_partial_reach_moved)
                return
            self.get_logger().info(f"Grasp point reached; settling for {self.settle_time_sec:.1f}s.")
            time.sleep(self.settle_time_sec)
            if not self._confirm_grasp_with_user():
                self.get_logger().warn("Grasp aborted by user (n/timeout) before closing gripper.")
                self._finish_picking_cycle()
                return
            close_result = self._close_gripper_with_current_feedback()
            if close_result is GripperCloseResult.EXECUTION_FAILED:
                self._register_precision_failure("gripper close execution")
                return
            if close_result is GripperCloseResult.MISS:
                if not self._move_gripper(GRIPPER_OPEN, "reopen after miss"):
                    self._register_precision_failure("gripper reopen execution")
                    return
                self._register_precision_failure("gripper current below threshold")
                return
            # 파지 후에는 AUTO 전용 HOLD 자세로 직접 이동한다. Base는 현재
            # 위치를 유지해 운반 자세 진입 중 불필요한 선회가 발생하지 않는다.
            hold_target = [
                HOLD_ARM_JOINTS.get(name, self._safe_current(name))
                for name in ARM_JOINT_NAMES
            ]
            self.get_logger().info("MoveIt hold pose after grasp")
            hold_start = time.monotonic()
            self.arm.move_to_configuration(hold_target)
            hold_ok = self._wait(self.arm, "post-grasp hold pose")
            self.get_logger().info(
                f"post-grasp hold pose: {time.monotonic() - hold_start:.2f}s 소요 (성공={hold_ok})")
            if not hold_ok:
                self._register_precision_failure("post-grasp hold execution")
                return
            if self.return_home:
                self.arm.move_to_configuration(HOME_JOINTS)
                self._wait(self.arm, "return home")
            self.get_logger().info("Grasp complete; payload remains held for transport.")
            self.picking_command_pub.publish(Empty())
            self._finish_picking_cycle()
        except Exception as exc:
            # 예외가 worker thread 밖으로 빠지면 /picking=true만 남고 이후의
            # target을 영원히 무시한다. 계산 실패 이벤트와 기존 재시도
            # 상태머신으로 회수해 latch된 목표를 계속 처리한다.
            self.get_logger().error(
                f"Unexpected grasp calculation exception: {type(exc).__name__}: {exc}")
            self._register_precision_failure(
                "unexpected calculation exception", calculation_failed=True)
        finally:
            self.get_logger().info(
                f"_run_sequence 전체 소요시간: {time.monotonic() - sequence_start:.2f}s "
                "(settle_time_sec 대기/사람 확인(y/n) 시간 포함, MoveIt 계산만의 시간은 "
                "위 pregrasp/descend/lift/home_pose/gripper 각 로그 참고)"
            )
            # 첫 실패는 새 카메라 좌표를 기다리지 않고 최초 latch 좌표로 즉시
            # 다시 시도한다. /picking=true라 외부 target callback은 이 사이에도
            # 새 값을 채택하지 않는다. _release_or_handoff_busy가 이 판단과
            # busy 점유권 이전/해제를 원자적으로 처리한다(_busy_owned 초기화
            # 주석의 2026-09-02 버그 참고).
            self._release_or_handoff_busy()

    def _register_precision_failure(
        self, reason: str, calculation_failed: bool = False,
        reobserve_after_partial_reach: bool = False,
    ) -> None:
        """Advance the requested precision-recognition/approach retry state machine.

        The current sequence keeps the busy lock until this method returns, so
        incoming target points are discarded while /picking=true. The first
        failure retries the latched position internally. After the configured
        consecutive-failure limit, /arm/forward_command performs one approach
        nudge; only a successful grasp_wait return clears /picking and permits a
        fresh target.
        """
        if not self._auto_motion_allowed():
            self.get_logger().info(
                f"Ignoring precision failure '{reason}' during manual override/AUTO off.")
            return
        if calculation_failed:
            self._publish_calculation_failure(reason)
        self._precision_failure_count += 1
        if reobserve_after_partial_reach:
            # 목표 방향으로 실제 진전했으므로 같은 좌표를 다시 쓰거나 별도의
            # seed 자세로 되돌리지 않는다. 새 차체/카메라 기하에서 재인식한다.
            self._precision_failure_count = 0
            self._wait_for_fresh_recognition()
            self.get_logger().info(
                "Partial reach moved toward the target; /picking=false and waiting "
                "for a fresh target_point.")
            return
        if self._precision_failure_count < self.precision_failures_before_approach:
            self._retry_latched_target = True
            self.get_logger().warn(
                f"Precision attempt failed ({reason}); retrying latched target "
                f"({self._precision_failure_count}/{self.precision_failures_before_approach}).")
            return

        self.get_logger().warn(
            f"Precision attempt failed ({reason}) for the {self._precision_failure_count}th time; "
            "publishing /arm/forward_command for a new approach.")
        self._precision_failure_count = 0
        self._retry_latched_target = False
        self._at_grasp_wait = False
        self.forward_command_pub.publish(Empty())
        if self.approach_retry_wait_sec:
            time.sleep(self.approach_retry_wait_sec)
        if not self._auto_motion_allowed():
            return
        if self._move_to_grasp_wait():
            self._at_grasp_wait = True
            self._finish_picking_cycle()
            self.get_logger().info("Approach retry complete; waiting for a fresh target_point.")
        else:
            # 이전에는 여기서 /picking=true와 latch만 남긴 채 worker가 끝나
            # 이후 target을 영구히 무시했다. 복귀 trajectory 실패도 명시적으로
            # 알리고, 같은 고정 좌표를 다음 precision 시도로 다시 계산한다.
            self._publish_calculation_failure("approach retry grasp_wait execution")
            self._retry_latched_target = True
            self.get_logger().error(
                "Approach retry could not return to grasp_wait; recalculating the "
                "latched target while /picking=true.")

    def _confirm_grasp_with_user(self) -> bool:
        """descend 완료 후 그리퍼를 닫기 전 사람 확인. confirm_before_close
        파라미터로 켜고 끌 수 있다 - 실기 검증/파라미터 튜닝 끝나면
        `-p confirm_before_close:=false`로 완전 자동으로 되돌리면 된다.
        터미널이 없어(launch를 백그라운드/비대화식으로 띄운 경우 등) 입력을
        못 받으면 안전하게 "n"(중단)으로 처리한다 - 확인 없이 그냥 닫아버리는
        쪽보다는 이쪽이 안전하다."""
        if not self.confirm_before_close:
            return True
        try:
            answer = input(
                "[maru_ik_node] descend 완료, 정지 중 - 그리퍼 닫을까요? [y/n]: ").strip().lower()
        except EOFError:
            self.get_logger().error(
                "confirm_before_close=true인데 터미널 입력을 못 받음(비대화식 실행?) - 안전하게 중단.")
            return False
        return answer == "y"

    def _cached_ik_seeds(self, position, orientation):
        """Return cached solutions ordered by proximity to this IK request."""
        if self.ik_cached_seed_count == 0 or not self._ik_solution_cache:
            return []

        def score(entry):
            cached_position, cached_orientation, _solution = entry
            position_distance = math.dist(position, cached_position)
            # q and -q denote the same orientation, hence abs(dot product).
            orientation_mismatch = 1.0 - min(
                1.0,
                abs(sum(a * b for a, b in zip(orientation, cached_orientation))),
            )
            return position_distance + 0.10 * orientation_mismatch

        nearest = sorted(self._ik_solution_cache, key=score)[:self.ik_cached_seed_count]
        return [list(solution) for _position, _orientation, solution in nearest]

    def _cache_ik_solution(self, position, orientation, solution) -> None:
        """Keep a bounded, recency-updated set of verified IK solutions."""
        if self.ik_solution_cache_size == 0:
            return
        entry = (tuple(position), tuple(orientation), tuple(solution))
        self._ik_solution_cache = [
            cached for cached in self._ik_solution_cache
            if cached[2] != entry[2]
        ]
        self._ik_solution_cache.insert(0, entry)
        del self._ik_solution_cache[self.ik_solution_cache_size:]

    def _solve_ik_multi_seed(self, position, orientation, target_pitch=None):
        """Solve with ordered seeds using the selected IK backend.

        ``moveit`` calls /compute_ik(KDL). ``numerical`` calls the local
        finite-difference Jacobian DLS solver and then returns the same joint
        configuration shape. Actual motion is always planned/executed later by
        MoveIt, so the numerical backend does not bypass trajectory collision
        checking or ros2_control.

        move_to_pose()와 달리 seed를 명시적으로 고정할 수 있어서, "현재 상태"가
        나쁜 seed인 경우에도 다른 seed로 찾은 해를 쓸 수 있다.
        스레드에서 직접 spin하지 않는다 - 이 노드를 이미 MultiThreadedExecutor가
        돌리고 있어서, future.done()만 폴링하면 그 executor가 알아서 콜백을
        처리해준다(중첩 spin은 예전에 random_target_publisher.py에서 겪은
        데드락 원인이라 반드시 피한다).

        빠른 실패가 필요한 운영 경로이므로 현재/캐시 seed부터 최대
        ``ik_max_seed_attempts``개만, 각각 ``ik_request_timeout_sec`` 동안
        시도한다. 더 긴 랜덤 탐색은 check_reachability.py의 진단용으로 남긴다.
        """
        # [갱신, 2026-09-01 #4] pregrasp 성공 직후 descend를 풀 때도 매번 이
        # 고정 IK_SEEDS만 썼다 - 그 시점엔 팔이 이미 목표 바로 근처(z만 10cm
        # 차이)에 있는데 그 "현재 자세"를 안 쓰는 건 낭비다. 현재 관절값을
        # 맨 앞 seed로 추가(다른 seed보다 먼저 시도) - 목표가 현재 위치에
        # 가까울수록 KDL이 훨씬 빨리/확실하게 수렴한다.
        # 현재 상태(특히 grasp_wait -> sid의 짧은 구간)를 가장 먼저 쓴다.
        # 그 다음에는 이전 성공 해 중 목표 pose와 가까운 것을 써서 static
        # seed의 지역해 편향을 줄이고, 마지막에 기존 고정 seed로 fallback한다.
        seeds_to_try = []
        current = [self._arm_positions.get(name) for name in ARM_JOINT_NAMES]
        if all(v is not None for v in current):
            seeds_to_try.append([self._safe_current(name) for name in ARM_JOINT_NAMES])
        seeds_to_try.extend(self._cached_ik_seeds(position, orientation))
        seeds_to_try.extend(ordered_static_ik_seeds(position))
        # 같은 seed를 반복하면 느려질 뿐이므로 순서를 보존하며 제거한다.
        unique_seeds = []
        seen_seeds = set()
        for seed in seeds_to_try:
            key = tuple(seed)
            if key not in seen_seeds:
                seen_seeds.add(key)
                unique_seeds.append(seed)
        seeds_to_try = unique_seeds[:self.ik_max_seed_attempts]

        if self.ik_solver_backend == "analytic":
            if target_pitch is None:
                self.get_logger().error(
                    "Analytic IK requires the pitch used to construct the target pose.")
                return None
            # numerical과 동일하게, 호출자가 넘긴 position(wrist 목표)에
            # orientation으로 회전시킨 wrist->TCP 오프셋을 다시 더해 TCP
            # 목표로 복원한다(analytic_grasp_planner.forward_kinematics는
            # TCP를 모델링함).
            tcp_offset_world = _rotate_vector_by_quaternion(
                orientation, WRIST_TO_TCP_LOCAL_OFFSET)
            tcp_target = tuple(
                float(position[i]) + float(tcp_offset_world[i]) for i in range(3))
            # 반복/seed가 없는 닫힌해라 seeds_to_try는 안 쓴다 - 후보(최대
            # 4개: base 2 x elbow-up/down 2) 전부가 이미 정확한 해라서 위치
            # 오차가 가장 작은 것부터 리미트 통과 여부만 확인하면 된다.
            for candidate in solve_ik_analytic(tcp_target, float(target_pitch)):
                solution = list(candidate.joints)
                out_of_range = [
                    name for name, value in zip(ARM_JOINT_NAMES, solution)
                    if not (self._arm_joint_limits[name][0]
                            <= value <= self._arm_joint_limits[name][1])
                ]
                if out_of_range:
                    continue
                self.get_logger().info(
                    "Analytic closed-form IK solved: "
                    f"position_error={candidate.position_error_m:.9f}m")
                self._cache_ik_solution(position, orientation, solution)
                return solution
            return None

        if self.ik_solver_backend == "numerical":
            if target_pitch is None:
                self.get_logger().error(
                    "Numerical IK requires the pitch used to construct the target pose.")
                return None
            # 호출자가 넘긴 position은 wrist_link 목표다. 수치 FK는 실제 TCP를
            # 모델링하므로 같은 orientation으로 wrist->TCP 오프셋을 다시 더해
            # 정확히 동일한 TCP 목표로 복원한다.
            tcp_offset_world = _rotate_vector_by_quaternion(
                orientation, WRIST_TO_TCP_LOCAL_OFFSET)
            tcp_target = tuple(
                float(position[i]) + float(tcp_offset_world[i]) for i in range(3))
            for seed in seeds_to_try:
                result = solve_ik_dls(
                    tcp_target,
                    float(target_pitch),
                    seed,
                    max_iterations=self.numerical_ik_max_iterations,
                    position_tolerance_m=self.numerical_ik_position_tolerance_m,
                    pitch_tolerance_rad=self.numerical_ik_pitch_tolerance_rad,
                    damping=self.numerical_ik_damping,
                )
                if not result.success:
                    continue
                solution = list(result.joints)
                out_of_range = [
                    name for name, value in zip(ARM_JOINT_NAMES, solution)
                    if not (self._arm_joint_limits[name][0]
                            <= value <= self._arm_joint_limits[name][1])
                ]
                if out_of_range:
                    continue
                self.get_logger().info(
                    "Numerical DLS IK solved: "
                    f"iterations={result.iterations}, "
                    f"position_error={result.position_error_m:.6f}m, "
                    f"pitch_error={result.pitch_error_rad:.6f}rad")
                self._cache_ik_solution(position, orientation, solution)
                return solution
            return None

        if not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/compute_ik 서비스를 못 찾음.")
            return None
        for seed in seeds_to_try:
            req = GetPositionIK.Request()
            req.ik_request = PositionIKRequest()
            req.ik_request.group_name = "arm"
            req.ik_request.pose_stamped = PoseStamped()
            req.ik_request.pose_stamped.header.frame_id = self.planning_frame
            req.ik_request.pose_stamped.pose.position.x = float(position[0])
            req.ik_request.pose_stamped.pose.position.y = float(position[1])
            req.ik_request.pose_stamped.pose.position.z = float(position[2])
            req.ik_request.pose_stamped.pose.orientation.x = float(orientation[0])
            req.ik_request.pose_stamped.pose.orientation.y = float(orientation[1])
            req.ik_request.pose_stamped.pose.orientation.z = float(orientation[2])
            req.ik_request.pose_stamped.pose.orientation.w = float(orientation[3])
            req.ik_request.avoid_collisions = self.ik_avoid_collisions
            timeout_nanosec = int(round(self.ik_request_timeout_sec * 1_000_000_000))
            req.ik_request.timeout.sec = timeout_nanosec // 1_000_000_000
            req.ik_request.timeout.nanosec = timeout_nanosec % 1_000_000_000
            js = JointState()
            js.name = list(ARM_JOINT_NAMES)
            js.position = list(seed)
            rs = RobotStateMsg()
            rs.joint_state = js
            rs.is_diff = False
            req.ik_request.robot_state = rs

            future = self._ik_client.call_async(req)
            deadline = time.monotonic() + self.ik_request_timeout_sec + 0.5
            while not future.done():
                if time.monotonic() >= deadline:
                    self.get_logger().warn(f"compute_ik(seed={seed}) 응답 타임아웃.")
                    break
                time.sleep(0.01)
            if not future.done():
                continue
            res = future.result()
            if res is not None and res.error_code.val == 1:
                by_name = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                solution = [by_name[name] for name in ARM_JOINT_NAMES]
                # compute_ik(KDL)가 실기 하드웨어 리미트를 살짝 넘는 해를 반환할
                # 수 있어(joint_calibration.yaml 기준) 여기서 직접
                # 재검증한다. 넘으면 이 seed는 버리고 다음 seed로 넘어간다 -
                # rmd_hardware_interface가 조용히 clamp한 채 목표에 영원히
                # 도달 못 하는 것보다 다른 seed/pitch로 재시도하는 게 낫다.
                out_of_range = [
                    name for name, value in zip(ARM_JOINT_NAMES, solution)
                    if not (self._arm_joint_limits[name][0] <= value <= self._arm_joint_limits[name][1])
                ]
                if out_of_range:
                    self.get_logger().warn(
                        f"compute_ik(seed={seed}) 해가 실기 리미트를 벗어남 "
                        f"({', '.join(out_of_range)}) - 이 seed는 버리고 다음으로."
                    )
                    continue
                self._cache_ik_solution(position, orientation, solution)
                return solution
            if res is not None:
                error_code = res.error_code.val
                collision_reason = IK_COLLISION_ERROR_CODES.get(error_code)
                if collision_reason:
                    self._ik_collision_rejected = True
                    self.get_logger().info(
                        f"compute_ik rejected seed due to {collision_reason} "
                        f"(collision checking={self.ik_avoid_collisions}).")
        return None

    def _get_current_tcp_position(self):
        """Return the current TCP (x, y, z) in ``planning_frame`` via one
        ``/compute_fk`` call on the live joint state, or None on any failure.

        Only reached from the partial-reach fallback below (at most once per
        fully-failed ``_move_arm`` call), so it stays far under the
        diagnostic call volume that has made move_group unreliable in past
        sessions when hammered directly."""
        current = [self._arm_positions.get(name) for name in ARM_JOINT_NAMES]
        if any(v is None for v in current):
            return None
        if not self._fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/compute_fk 서비스를 못 찾음.")
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = self.planning_frame
        req.fk_link_names = ["wrist_link"]
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [
            self._safe_current(name) for name in ARM_JOINT_NAMES]
        future = self._fk_client.call_async(req)
        deadline = time.monotonic() + 1.5
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warn("compute_fk 응답 타임아웃.")
                return None
            time.sleep(0.01)
        res = future.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        wrist_pose = res.pose_stamped[0].pose
        orientation = (
            wrist_pose.orientation.x, wrist_pose.orientation.y,
            wrist_pose.orientation.z, wrist_pose.orientation.w)
        offset_world = _rotate_vector_by_quaternion(orientation, WRIST_TO_TCP_LOCAL_OFFSET)
        return (
            wrist_pose.position.x + offset_world[0],
            wrist_pose.position.y + offset_world[1],
            wrist_pose.position.z + offset_world[2],
        )

    def _solve_ik_any_pitch(self, position, base_angle):
        """Solve-only variant of the pitch loop in ``_move_arm`` (no motion
        execution). Returns (joint_solution, pitch) for the first pitch
        candidate with a solution, or (None, None)."""
        # 부분 접근은 정확한 파지 자세가 아니라 "목표 방향으로 안전하게
        # 진전"하는 목적이다. 목표 pitch만 강제하면 현재 TCP에 가까운 보간
        # 지점조차 자세 제약 때문에 실패할 수 있으므로, 현재 실측 pitch를
        # 가장 먼저 보존해 위치 이동의 성공 가능성을 높인다.
        pitch_candidates = []
        if all(name in self._arm_positions for name in ARM_JOINT_NAMES):
            pitch_candidates.append(sum(
                self._safe_current(name)
                for name in ("shoulder_joint", "elbow_joint", "wrist_joint")
            ))
        pitch_candidates.extend((self.approach_pitch, *self.approach_pitch_fallbacks))
        unique_pitches = []
        for pitch in pitch_candidates:
            if not any(math.isclose(pitch, existing, abs_tol=1e-6)
                       for existing in unique_pitches):
                unique_pitches.append(pitch)

        for pitch in unique_pitches:
            if not self._auto_motion_allowed():
                return None, None
            orientation = quaternion_from_base_pitch(base_angle, pitch)
            offset_world = _rotate_vector_by_quaternion(orientation, WRIST_TO_TCP_LOCAL_OFFSET)
            wrist_target = (
                position[0] - offset_world[0],
                position[1] - offset_world[1],
                position[2] - offset_world[2],
            )
            joint_solution = self._solve_ik_multi_seed(
                wrist_target, orientation, target_pitch=pitch)
            if joint_solution is not None:
                return joint_solution, pitch
        return None, None

    def _attempt_partial_reach(self, target_tcp, label: str) -> bool:
        """When every pitch/seed for the exact target has failed, try moving
        to the closest-to-target point along the line from the current TCP
        to ``target_tcp`` that IK can actually solve, instead of not moving
        at all.

        [추가, 2026-09-03] 설정된 ``ik_partial_reach_fractions`` 후보가 전부
        실패해도 이동 없이 포기하지 않는다 - "멀더라도 최대한 근사하게
        이동시키고 싶다"는 요청에 따라, 0.0(현재 위치, 이미 그 자세로 있으므로
        사실상 항상 IK가 풀림)과 가장 작은 실패 fraction 사이를 이분탐색해서
        실제로 IK가 풀리는 가장 먼(=target에 가장 가까운) 지점을 강제로 찾는다.
        이 이분탐색까지 실패하는 경우는 compute_ik 자체가 응답하지 않는 등
        진짜 예외 상황뿐이다.

        The caller (``_move_arm``) still returns False after this - the
        original target was not reached. Return True only when a partial
        trajectory actually completed; the caller then drops the old target
        and waits for perception to measure the new geometry."""
        current_tcp = self._get_current_tcp_position()
        if current_tcp is None:
            self.get_logger().warn(
                f"{label}: 현재 TCP 위치를 못 구해 근접 이동 폴백을 건너뜀.")
            return False

        def solve_at_fraction(fraction: float):
            interp = tuple(
                current_tcp[i] + fraction * (target_tcp[i] - current_tcp[i])
                for i in range(3)
            )
            base_angle = compute_base_angle(interp[0], interp[1])
            joint_solution, pitch = self._solve_ik_any_pitch(interp, base_angle)
            if joint_solution is None and "base_joint" in self._arm_positions:
                # [추가, 2026-09-03] compute_base_angle()는 반경이 작을수록
                # 실측 오차가 커진다(실측: r≈0.10m에서 약 24°, r≈0.28~0.31m에서는
                # 1~2° 수준) - fraction이 작아 interp가 현재 위치에 아주
                # 가까워질수록 이 오차 때문에 실제로는 도달 가능한 자세도
                # 놓칠 수 있다. 이 경우 재계산 대신 실측 현재 base_joint 값을
                # 그대로 후보로 한 번 더 시도한다.
                actual_base_angle = self._safe_current("base_joint")
                if not math.isclose(actual_base_angle, base_angle, abs_tol=1e-3):
                    joint_solution, pitch = self._solve_ik_any_pitch(
                        interp, actual_base_angle)
            return joint_solution, pitch, interp

        def move_to_fraction(joint_solution, pitch, interp, fraction: float, note: str) -> bool:
            self.get_logger().warn(
                f"{label}: 정밀 목표 IK 실패 - {fraction:.3f} 지점 "
                f"({interp[0]:.3f}, {interp[1]:.3f}, {interp[2]:.3f}, "
                f"pitch={pitch:.2f})으로 대신 이동 시도{note}(원래 목표는 계속 실패로 처리).")
            self.arm.move_to_configuration(joint_solution)
            ok = self._wait(self.arm, f"{label} partial reach ({fraction:.3f})")
            if not ok:
                self.get_logger().error(f"{label}: 근접 지점 이동 실행 실패.")
            return ok

        for fraction in self.ik_partial_reach_fractions:
            if not self._auto_motion_allowed():
                return False
            joint_solution, pitch, interp = solve_at_fraction(fraction)
            if joint_solution is None:
                continue
            return move_to_fraction(joint_solution, pitch, interp, fraction, "")

        if not self.ik_partial_reach_fractions or self.ik_partial_reach_bisection_iterations <= 0:
            self.get_logger().warn(f"{label}: 보간 지점도 전부 IK 실패 - 이동 없이 포기.")
            return False

        self.get_logger().warn(
            f"{label}: 설정된 근접 후보({self.ik_partial_reach_fractions}) 전부 실패 - "
            "이분탐색으로 최대한 가까운 도달 지점을 강제로 찾는다.")
        low, high = 0.0, min(self.ik_partial_reach_fractions)
        best = None  # (joint_solution, pitch, interp, fraction)
        for _ in range(self.ik_partial_reach_bisection_iterations):
            if not self._auto_motion_allowed():
                break
            mid = (low + high) / 2.0
            joint_solution, pitch, interp = solve_at_fraction(mid)
            if joint_solution is not None:
                best = (joint_solution, pitch, interp, mid)
                low = mid
            else:
                high = mid
        if best is not None:
            joint_solution, pitch, interp, fraction = best
            return move_to_fraction(
                joint_solution, pitch, interp, fraction, " (이분탐색 강제 근접)")

        self.get_logger().warn(
            f"{label}: 이분탐색으로도 근접 지점을 못 찾음 - 이동 없이 포기 "
            f"(current_tcp={tuple(round(v, 3) for v in current_tcp)}, "
            f"target_tcp={tuple(round(v, 3) for v in target_tcp)}).")
        return False

    def _move_arm(self, position, base_angle: float, label: str) -> bool:
        if not self._auto_motion_allowed():
            return False
        # pregrasp/descend/lift가 서로 다른 z(전방거리)를 목표하고, 이 팔은
        # 4DOF라 pitch(=shoulder+elbow+wrist 합)가 그 z에 맞는 값이어야만
        # IK가 풀린다(compute_ik 실측으로 확인: pitch 유효 범위가 z에 따라
        # -3.1~-2.5 사이에서 갈렸음) - 그래서 한 후보로 실패하면 다음
        # 후보로 넘어간다.
        # [추가] MoveIt/pitch 재시도가 실제로 얼마나 걸리는지 실측하기 위한
        # 타이밍 로그 - "무브잇이 안 늦어질까" 질문에 추측 대신 숫자로 답하려는
        # 목적. 후보마다 걸린 시간 + 전체 누적 시간을 둘 다 찍는다.
        overall_start = time.monotonic()
        self._last_ik_unreachable = False
        self._last_partial_reach_moved = False
        self._ik_collision_rejected = False
        found_ik_solution = False
        for attempt, pitch in enumerate((self.approach_pitch, *self.approach_pitch_fallbacks), start=1):
            if not self._auto_motion_allowed():
                return False
            orientation = quaternion_from_base_pitch(base_angle, pitch)
            # position은 그리퍼(tcp_link)가 닿아야 할 목표(카메라가 찾은 박스
            # 위치) - IK는 wrist_link를 목표로 풀기 때문에, 이 pitch의 orientation
            # 으로 wrist->tcp 오프셋을 회전시켜 뺀 위치를 wrist 목표로 넘긴다
            # (WRIST_TO_TCP_LOCAL_OFFSET 주석 참고).
            tcp_offset_world = _rotate_vector_by_quaternion(orientation, WRIST_TO_TCP_LOCAL_OFFSET)
            wrist_target = (
                position[0] - tcp_offset_world[0],
                position[1] - tcp_offset_world[1],
                position[2] - tcp_offset_world[2],
            )
            attempt_start = time.monotonic()
            joint_solution = self._solve_ik_multi_seed(
                wrist_target, orientation, target_pitch=pitch)
            ik_elapsed = time.monotonic() - attempt_start
            if joint_solution is None:
                self.get_logger().warn(
                    f"{label}: pitch={pitch:.2f} 시도 {attempt} - IK seed 후보 전부 실패 "
                    f"({ik_elapsed:.2f}s, move_group 플래닝은 시도 안 함)"
                )
                continue
            found_ik_solution = True
            self.get_logger().info(
                f"MoveIt {label} (pitch={pitch:.2f}, 시도 {attempt}) - IK 해 찾음({ik_elapsed:.2f}s), 실행 중"
            )
            self.arm.move_to_configuration(joint_solution)
            ok = self._wait(self.arm, f"{label} (pitch={pitch:.2f})")
            attempt_elapsed = time.monotonic() - attempt_start
            if ok:
                total_elapsed = time.monotonic() - overall_start
                self.get_logger().info(
                    f"{label}: pitch={pitch:.2f}(시도 {attempt}번째)로 성공 - "
                    f"이번 시도 {attempt_elapsed:.2f}s, {label} 전체 {total_elapsed:.2f}s"
                )
                return True
            self.get_logger().warn(f"{label}: pitch={pitch:.2f} 실패 ({attempt_elapsed:.2f}s 소요)")
        total_elapsed = time.monotonic() - overall_start
        self.get_logger().error(
            f"{label}: no candidate pitch succeeded "
            f"({[self.approach_pitch, *self.approach_pitch_fallbacks]}), "
            f"총 {total_elapsed:.2f}s 소요."
        )
        if self.ik_partial_reach_enabled and self._auto_motion_allowed():
            self._last_partial_reach_moved = self._attempt_partial_reach(position, label)
        # 충돌 검사에 걸린 경우에는 좌표가 기구학적으로 도달 불가한 게 아니므로
        # 전진 명령을 내면 안 된다. 진짜 IK 해가 전혀 없고 충돌 거부도 없을
        # 때만 가동범위 밖으로 분류한다.
        self._last_ik_unreachable = not found_ik_solution and not self._ik_collision_rejected
        if self._ik_collision_rejected and not found_ik_solution:
            self.get_logger().warn(
                f"{label}: IK collision rejection occurred; not classifying target as out of workspace.")
        return False

    def _safe_current(self, name: str) -> float:
        """self._arm_positions[name]를 calibration 리미트 안으로 clamp해서
        반환한다. [추가, 2026-08-31] "현재 위치 유지"용 pass-through 목표를
        만들 때 팔이 실제로 방금 하드스톱 근처에 쉬고 있으면 그 raw 값
        자체가 (특히 리미트를 좁힌 직후) 범위를 벗어나 rmd_hardware_interface가
        영원히 clamp하며 멈추는 걸 반복 확인했다 - "유지"할 목표는 항상
        유효한 범위 안이어야 한다는 방어적 조치."""
        value = self._arm_positions[name]
        low, high = self._arm_joint_limits[name]
        return max(low, min(high, value))

    def _move_to_grasp_wait(self) -> bool:
        """[수정, 2026-09-01] 더 이상 HOME을 거쳐서 호출되지 않는다 -
        on_drive_detected -> _run_grasp_wait_sequence가 바로 이 메서드를
        호출한다. base는 현재 위치 유지, shoulder/elbow/wrist만 GRASP_WAIT
        자세로 한 번에 이동한다."""
        if not self._auto_motion_allowed():
            return False
        missing = [name for name in ARM_JOINT_NAMES if name not in self._arm_positions]
        if missing:
            self.get_logger().error(
                "Cannot move to grasp_wait; missing joint states: " + ", ".join(missing))
            return False
        target = [
            self.grasp_wait_arm_joints.get(name, self._safe_current(name))
            for name in ARM_JOINT_NAMES
        ]
        self.get_logger().info(
            f"MoveIt: moving to {self.grasp_wait_preset} pose")
        start = time.monotonic()
        self.arm.move_to_configuration(target)
        ok = self._wait(self.arm, "grasp_wait")
        self.get_logger().info(f"grasp_wait: {time.monotonic() - start:.2f}s 소요 (성공={ok})")
        return ok

    def _move_gripper(self, position: float, label: str) -> bool:
        if not self._auto_motion_allowed():
            return False
        start = time.monotonic()
        self.gripper.move_to_configuration([position])
        ok = self._wait(self.gripper, f"gripper {label}")
        self.get_logger().info(f"gripper {label}: {time.monotonic() - start:.2f}s 소요 (성공={ok})")
        return ok

    def _close_gripper_with_current_feedback(self) -> GripperCloseResult:
        """고정 위치(GRIPPER_CLOSED)까지 무조건 닫는 대신, 닫는 도중
        gripper_joint 전류가 threshold를 넘으면(물체에 닿아 부하 걸림) 그
        자리에서 즉시 정지한다 - 이게 "전류 피드백 기반 파지 제어"의 실제
        제어 루프다.

        [갱신, 2026-09-02] move_to_named_pose.py _close_gripper_with_feedback()
        과 동일한 OR 조건 추가 - 전류 스파이크가 없어도 gripper_joint 위치가
        GRIPPER_CLOSED 근처(GRIPPER_POSITION_TOLERANCE_RAD 이내)까지 도달하면
        그것도 grasp_success=True로 본다(사용자 요청). 이전에는 이 경우를
        전부 MISS로 판정해 재시도했지만, 지금은 "완전히 닫혔다"는 것 자체를
        성공 신호로도 인정한다 - 실제로 물체가 없는데도 우연히 위치
        허용오차 안에서 멈추면 오탐(false positive)으로 파지 성공 처리될 수
        있다는 점에 주의할 것(전류 트립만 쓰던 이전 판정보다 미탐지는
        줄지만 오탐 위험은 늘어나는 트레이드오프).
        """
        if not self._auto_motion_allowed():
            return GripperCloseResult.EXECUTION_FAILED
        self.get_logger().info("MoveIt gripper close (current feedback)")
        self.gripper.move_to_configuration([GRIPPER_CLOSED])

        deadline = time.monotonic() + self.motion_timeout_sec
        grasp_success = False
        stopped_by_current = False
        while self.gripper.query_state() != MoveIt2State.IDLE:
            if not self._auto_motion_allowed():
                self.get_logger().warn(
                    "gripper close cancelled because AUTO was disabled or manually overridden.")
                self.gripper.cancel_execution()
                return GripperCloseResult.EXECUTION_FAILED
            if self._gripper_current_ma >= self.gripper_current_threshold_ma:
                self.get_logger().info(
                    f"Gripper current {self._gripper_current_ma:.1f}mA reached threshold "
                    f"{self.gripper_current_threshold_ma:.1f}mA; stopping close motion (grasp).")
                self.gripper.cancel_execution()
                grasp_success = True
                stopped_by_current = True
                break
            if time.monotonic() >= deadline:
                self.get_logger().error("gripper close timed out; cancelling motion.")
                self.gripper.cancel_execution()
                return GripperCloseResult.EXECUTION_FAILED
            time.sleep(0.02)

        if not stopped_by_current:
            # 전류 트립 없이 IDLE에 도달 = GRIPPER_CLOSED까지 완전히 닫힘.
            if not self.gripper.motion_suceeded:
                self.get_logger().error("gripper close failed in MoveIt.")
                return GripperCloseResult.EXECUTION_FAILED
            grasp_success = (
                self._gripper_position is not None
                and self._gripper_position >= GRIPPER_CLOSED - GRIPPER_POSITION_TOLERANCE_RAD
            )
            if grasp_success:
                self.get_logger().warn(
                    "Gripper reached fully-closed position without a current spike, but "
                    f"position={self._gripper_position:.5f} is within tolerance of "
                    "GRIPPER_CLOSED - treating as grasp success (OR condition; object "
                    "presence is uncertain in this case).")
            else:
                self.get_logger().warn(
                    "Gripper reached fully-closed position without a current spike and "
                    f"position={self._gripper_position} is not within tolerance; "
                    "likely missed the object.")

        self.grasp_success_pub.publish(Bool(data=grasp_success))
        return GripperCloseResult.SUCCESS if grasp_success else GripperCloseResult.MISS

    def _wait(self, moveit: MoveIt2, label: str) -> bool:
        deadline = time.monotonic() + self.motion_timeout_sec
        while moveit.query_state() != MoveIt2State.IDLE:
            if not self._auto_motion_allowed():
                self.get_logger().warn(
                    f'{label} cancelled because AUTO was disabled or manually overridden.')
                moveit.cancel_execution()
                return False
            if time.monotonic() >= deadline:
                self.get_logger().error(f"{label} timed out; cancelling motion.")
                moveit.cancel_execution()
                return False
            time.sleep(0.05)
        if not moveit.motion_suceeded:
            self.get_logger().error(f"{label} failed in MoveIt.")
            return False
        return True


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 dependencies are not installed")
    rclpy.init(args=args)
    node = MaruIKNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
