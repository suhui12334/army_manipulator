#!/usr/bin/env python3
"""실제 인식된 좌표 하나에 대해, 관절 리미트 안에서 무작위 seed를 대량으로
뿌려가며 compute_ik를 반복 호출해서 "닿는 해가 정말 존재하는지"를 직접
확인한다 - FK로 이런저런 자세를 추측해서 근사하는 것보다 확실하다.

성공하면 그 즉시 멈추고 성공한 관절값을 출력한다(바로 새 IK_SEED 후보로
쓸 수 있음). 모든 pitch x seed 조합을 다 시도했는데도 실패하면 "이 예산
안에서는 해를 못 찾음"으로 보고한다(수학적으로 100% 불가능 증명은 아니지만,
충분히 강한 정황 증거).

maru_ik_node.py와 동일한 compute_base_angle/quaternion_from_base_pitch/
WRIST_TO_TCP_LOCAL_OFFSET 공식을 그대로 복사해서 쓴다(임포트 대신 복사한
이유: maru_ik_node.py는 ROS 의존성 없이 단위 테스트도 되게 try/except로
감싸져 있어 여기서 그대로 재사용하기 애매함).

사용:
    ros2 run army_manipulator_bringup check_reachability.py
"""
import math
import random
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState as RobotStateMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from joint_calibration import JointCalibration

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

def load_arm_joint_limits(calibration: JointCalibration):
    """Use the same actual limits as URDF, MoveIt, and maru_ik_node."""
    missing = [name for name in ARM_JOINT_NAMES if not calibration.has_joint(name)]
    if missing:
        raise RuntimeError(
            f"joint calibration unavailable ({calibration.path}): missing {', '.join(missing)}"
        )
    return {name: calibration.actual_limits(name) for name in ARM_JOINT_NAMES}

BASE_ROTATION_Y_OFFSET_M = 0.0313
_BASE_ANGLE_EMPIRICAL_CORRECTION = 2.9439
WRIST_TO_TCP_LOCAL_OFFSET = (-0.240, 0.0, 0.0313)

# [실측] 마지막으로 실패했던 실제 인식 좌표(base_actuator, maru_ik_node.py
# "Grasp point in base_actuator" 로그 값 그대로) - 다른 좌표를 확인하려면
# 여기를 바꾸면 된다.
TARGET_XYZ = (-0.746, 0.001, -0.397)

# pitch 후보 - maru_ik_node.py approach_pitch/approach_pitch_fallbacks와 동일.
PITCH_CANDIDATES = [2.95, 3.05, 2.85, 3.15, 2.75, 3.25, 2.65, 3.35]

RANDOM_SEEDS_PER_PITCH = 15
PER_ATTEMPT_TIMEOUT_SEC = 2.0


def compute_base_angle(target_x: float, target_y: float) -> float:
    radius = math.hypot(target_x, target_y)
    x0 = math.sqrt(max(radius * radius - BASE_ROTATION_Y_OFFSET_M ** 2, 0.0))
    raw = (
        math.atan2(target_y, target_x)
        - math.atan2(BASE_ROTATION_Y_OFFSET_M, x0)
        - _BASE_ANGLE_EMPIRICAL_CORRECTION
    )
    return (raw + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_base_pitch(base_angle: float, pitch: float):
    cb, sb = math.cos(base_angle / 2.0), math.sin(base_angle / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    p = 0.5 * (cp + sp)
    q = 0.5 * (cp - sp)
    return (cb * p - sb * q, sb * p + cb * q, sb * p - cb * q, sb * q + cb * p)


def rotate_vector_by_quaternion(q, v):
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


def random_seed(arm_joint_limits):
    return [
        random.uniform(*arm_joint_limits[name]) for name in ARM_JOINT_NAMES
    ]


def try_ik(client, node, position, orientation, seed, timeout_sec, arm_joint_limits):
    req = GetPositionIK.Request()
    req.ik_request = PositionIKRequest()
    req.ik_request.group_name = "arm"
    req.ik_request.pose_stamped = PoseStamped()
    req.ik_request.pose_stamped.header.frame_id = "base_actuator"
    req.ik_request.pose_stamped.pose.position.x = float(position[0])
    req.ik_request.pose_stamped.pose.position.y = float(position[1])
    req.ik_request.pose_stamped.pose.position.z = float(position[2])
    req.ik_request.pose_stamped.pose.orientation.x = float(orientation[0])
    req.ik_request.pose_stamped.pose.orientation.y = float(orientation[1])
    req.ik_request.pose_stamped.pose.orientation.z = float(orientation[2])
    req.ik_request.pose_stamped.pose.orientation.w = float(orientation[3])
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = int(timeout_sec)
    js = JointState()
    js.name = list(ARM_JOINT_NAMES)
    js.position = list(seed)
    rs = RobotStateMsg()
    rs.joint_state = js
    rs.is_diff = False
    req.ik_request.robot_state = rs

    future = client.call_async(req)
    deadline = time.monotonic() + timeout_sec + 1.0
    while not future.done():
        if time.monotonic() >= deadline:
            return None
        rclpy.spin_once(node, timeout_sec=0.01)
    res = future.result()
    if res is not None and res.error_code.val == 1:
        by_name = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        solution = [by_name[name] for name in ARM_JOINT_NAMES]
        if all(
            arm_joint_limits[name][0] <= value <= arm_joint_limits[name][1]
            for name, value in zip(ARM_JOINT_NAMES, solution)
        ):
            return solution
    return None


def main():
    rclpy.init()
    node = Node("check_reachability")
    try:
        calibration = JointCalibration()
        arm_joint_limits = load_arm_joint_limits(calibration)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info(f"Joint limits loaded from {calibration.path}")
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/compute_ik 서비스를 못 찾음 - move_group이 떠 있는지 확인하세요.")
        rclpy.shutdown()
        return

    x, y, z = TARGET_XYZ
    base_angle = compute_base_angle(x, y)
    print(f"타겟: {TARGET_XYZ}, base_angle={base_angle:.4f}rad")
    print(f"pitch {len(PITCH_CANDIDATES)}개 x 랜덤 seed {RANDOM_SEEDS_PER_PITCH}개 "
          f"(seed당 최대 {PER_ATTEMPT_TIMEOUT_SEC:.0f}s) - 최악 소요시간 약 "
          f"{len(PITCH_CANDIDATES) * RANDOM_SEEDS_PER_PITCH * PER_ATTEMPT_TIMEOUT_SEC:.0f}s")

    start = time.monotonic()
    attempts = 0
    for pitch in PITCH_CANDIDATES:
        orientation = quaternion_from_base_pitch(base_angle, pitch)
        tcp_offset_world = rotate_vector_by_quaternion(orientation, WRIST_TO_TCP_LOCAL_OFFSET)
        wrist_target = (
            x - tcp_offset_world[0], y - tcp_offset_world[1], z - tcp_offset_world[2],
        )
        for i in range(RANDOM_SEEDS_PER_PITCH):
            seed = random_seed(arm_joint_limits)
            attempts += 1
            solution = try_ik(
                client, node, wrist_target, orientation, seed,
                PER_ATTEMPT_TIMEOUT_SEC, arm_joint_limits,
            )
            if solution is not None:
                elapsed = time.monotonic() - start
                print(f"\n성공! pitch={pitch:.2f}, 시도 {attempts}번째, {elapsed:.1f}s 소요")
                print(f"관절해: {dict(zip(ARM_JOINT_NAMES, solution))}")
                print("(이 값을 그대로 IK_SEEDS에 추가하면 됩니다)")
                node.destroy_node()
                rclpy.shutdown()
                return
        print(f"pitch={pitch:.2f}: {RANDOM_SEEDS_PER_PITCH}개 랜덤 seed 전부 실패 "
              f"({time.monotonic() - start:.1f}s 누적)")

    elapsed = time.monotonic() - start
    print(f"\n총 {attempts}회 시도, {elapsed:.1f}s 동안 해를 하나도 못 찾음 - "
          "이 좌표는 (테스트한 예산 안에서는) 도달 불가능한 것으로 보입니다.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
