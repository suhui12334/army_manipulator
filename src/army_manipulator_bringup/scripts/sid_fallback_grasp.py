#!/usr/bin/env python3
"""[예비용] IK 기반 동적 파지(maru_ik_node.py)가 안 될 때 쓰는 단순 폴백.

카메라 좌표를 전혀 안 쓴다 - `/arm/target_point`에 뭔가 오면(좌표 값은
무시) 무조건 같은 고정 자세(sid, capture_arm_pose.py로 실측한 근거리
파지 자세)로 이동해서 그리퍼를 여닫고 hold로 옮긴다. IK_SEEDS/pitch 탐색이
전혀 없어서 실패할 일이 없는 대신, 박스가 항상 sid 자세로 딱 도달하는
"같은 자리"에 있다고 가정한다(카메라가 멀리 있는 박스를 봐도 그대로 sid로
돌진하니, 로봇을 그 정해진 위치까지 사람이 직접 붙여놓고 쓰는 용도).

maru_ik_node.py와 동시에 띄우면 둘 다 /arm/target_point를 받아서 서로
충돌하니 절대 같이 쓰지 말 것 - maru_ik_node 대신 이 노드만 띄워서 쓴다.

시퀀스: target_point 수신 -> sid 이동 -> 그리퍼 열기 -> 그리퍼 닫기(전류
피드백으로 파지 성공 판정) -> /arm/grasp_success 발행 -> hold 이동.

사용:
    ros2 run army_manipulator_bringup sid_fallback_grasp.py
"""
from __future__ import annotations

import threading
import time

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from pymoveit2 import MoveIt2, MoveIt2State
except ModuleNotFoundError:  # unit-test import without a ROS installation
    rclpy = None
    Node = object
    MultiThreadedExecutor = None

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
GRIPPER_JOINT_NAMES = ["gripper_joint"]

# capture_arm_pose.py "f" 시퀀스 최종 수렴 자세 - move_to_named_pose.py "sid"
# 및 verify_grasp_pose_fk.py의 값과 동일. 그쪽이 갱신되면 여기도 같이 맞출 것.
SID_ARM_JOINTS = {
    "shoulder_joint": 1.71566,
    "elbow_joint": 0.62151,
    "wrist_joint": 0.82519,
}
SID_BASE = -0.02761

# maru_ik_node.py HOLD_ARM_JOINTS(2026-09-01 shoulder/grasp_wait 교체 반영본)와 동일.
HOLD_ARM_JOINTS = {
    "shoulder_joint": -0.76044,
    "elbow_joint": 1.6383,
    "wrist_joint": 1.5329,
}

# maru_ik_node.py와 동일한 그리퍼 상수.
GRIPPER_OPEN = 0.07363
GRIPPER_CLOSED = 1.8294


class SidFallbackGrasp(Node):
    def __init__(self):
        super().__init__("sid_fallback_grasp")
        group = ReentrantCallbackGroup()
        for name, default in (
            ("target_topic", "/arm/target_point"),
            ("motion_timeout_sec", 20.0),
            ("gripper_current_threshold_ma", 100.0),
            ("gripper_current_lsb_ma", 2.69),
        ):
            self.declare_parameter(name, default)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.gripper_current_threshold_ma = float(
            self.get_parameter("gripper_current_threshold_ma").value)
        self.gripper_current_lsb_ma = float(self.get_parameter("gripper_current_lsb_ma").value)

        self._busy = threading.Lock()
        self._gripper_current_ma = 0.0
        self._arm_positions = {}

        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINT_NAMES, base_link_name="base_actuator",
            end_effector_name="wrist_link", group_name="arm", callback_group=group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )
        self.gripper = MoveIt2(
            node=self, joint_names=GRIPPER_JOINT_NAMES, base_link_name="wrist_link",
            end_effector_name="tcp_link", group_name="gripper", callback_group=group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )
        self.create_subscription(
            PointStamped, self.target_topic, self.on_target, 10, callback_group=group)
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10, callback_group=group)
        self.grasp_success_pub = self.create_publisher(Bool, "/arm/grasp_success", 10)

        self.get_logger().warn(
            f"[예비용 폴백] {self.target_topic} 좌표를 무시하고 항상 sid로 이동합니다 - "
            "maru_ik_node.py와 절대 동시에 띄우지 마세요."
        )

    def on_joint_states(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINT_NAMES:
                self._arm_positions[name] = float(position)
        try:
            idx = msg.name.index("gripper_joint")
        except ValueError:
            return
        if idx >= len(msg.effort):
            return
        self._gripper_current_ma = abs(msg.effort[idx]) * self.gripper_current_lsb_ma

    def on_target(self, _msg: PointStamped) -> None:
        if not self._busy.acquire(blocking=False):
            self.get_logger().info(
                "이미 파지 시퀀스 진행 중; 중복 감지 무시.", throttle_duration_sec=5.0)
            return
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self) -> None:
        try:
            sid_target = [SID_BASE, *SID_ARM_JOINTS.values()]
            self.get_logger().info("MoveIt: sid 이동")
            self.arm.move_to_configuration(sid_target)
            if not self._wait(self.arm, "sid"):
                return

            if not self._move_gripper(GRIPPER_OPEN, "open"):
                return

            if not self._close_gripper_with_current_feedback():
                return

            hold_target = [
                self._arm_positions.get(name, 0.0) if name == "base_joint"
                else HOLD_ARM_JOINTS[name]
                for name in ARM_JOINT_NAMES
            ]
            self.get_logger().info("MoveIt: hold 이동")
            self.arm.move_to_configuration(hold_target)
            self._wait(self.arm, "hold")
        finally:
            self._busy.release()

    def _move_gripper(self, position: float, label: str) -> bool:
        self.gripper.move_to_configuration([position])
        return self._wait(self.gripper, f"gripper {label}")

    def _close_gripper_with_current_feedback(self) -> bool:
        """maru_ik_node.py의 _close_gripper_with_current_feedback과 동일한 로직."""
        self.get_logger().info("MoveIt: 그리퍼 닫기(전류 피드백)")
        self.gripper.move_to_configuration([GRIPPER_CLOSED])

        deadline = time.monotonic() + self.motion_timeout_sec
        grasp_success = False
        stopped_by_current = False
        while self.gripper.query_state() != MoveIt2State.IDLE:
            if self._gripper_current_ma >= self.gripper_current_threshold_ma:
                self.get_logger().info(
                    f"그리퍼 전류 {self._gripper_current_ma:.1f}mA로 임계값 도달; 정지(파지).")
                self.gripper.cancel_execution()
                grasp_success = True
                stopped_by_current = True
                break
            if time.monotonic() >= deadline:
                self.get_logger().error("그리퍼 닫기 타임아웃; 정지.")
                self.gripper.cancel_execution()
                return False
            time.sleep(0.02)

        if not stopped_by_current:
            if not self.gripper.motion_suceeded:
                self.get_logger().error("그리퍼 닫기 실패(MoveIt).")
                return False
            self.get_logger().warn("전류 스파이크 없이 완전히 닫힘 - 파지 실패로 추정.")
            grasp_success = False

        self.grasp_success_pub.publish(Bool(data=grasp_success))
        return True

    def _wait(self, moveit: MoveIt2, label: str) -> bool:
        deadline = time.monotonic() + self.motion_timeout_sec
        while moveit.query_state() != MoveIt2State.IDLE:
            if time.monotonic() >= deadline:
                self.get_logger().error(f"{label} 타임아웃; 취소.")
                moveit.cancel_execution()
                return False
            time.sleep(0.05)
        if not moveit.motion_suceeded:
            self.get_logger().error(f"{label} 실패(MoveIt).")
            return False
        return True


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 dependencies are not installed")
    rclpy.init(args=args)
    node = SidFallbackGrasp()
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
