#!/usr/bin/env python3
"""D455 ``supplybox`` 3D 점을 MoveIt 계획과 ros2_control 실행으로 연결한다.

입력 점은 반드시 카메라 optical frame으로 publish한다. 이 노드가 TF로
``base_link``로 변환한 뒤, pre-grasp -> grasp -> close -> lift 순서를 MoveIt의
move_group action으로 실행한다. MoveIt이 만든 FollowJointTrajectory는
ros2_control을 거쳐 RMD(CAN) 및 Dynamixel 하드웨어 인터페이스로 전달된다.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Sequence

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from geometry_msgs.msg import PointStamped
    from tf2_geometry_msgs import do_transform_point
    import tf2_ros
    from pymoveit2 import MoveIt2, MoveIt2State
except ModuleNotFoundError:  # unit-test import without a ROS installation
    rclpy = None
    Node = object
    MultiThreadedExecutor = None


ARM_JOINT_NAMES = ["base_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"]
GRIPPER_JOINT_NAMES = ["gripper_joint"]
HOME_JOINTS = [0.0, 1.027, -1.2584, -1.1716]
GRIPPER_OPEN = -1.0647
GRIPPER_CLOSED = -2.5831


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


def quaternion_from_yaw_x_pitch(yaw: float, pitch: float) -> Sequence[float]:
    """Rotation ``Ry(yaw) * Rx(pitch)`` in ROS ``(x, y, z, w)`` order.

    The base rotates about Y and the three arm joints rotate about X in this URDF;
    using a fixed quaternion makes most 4-DoF pose goals unsolvable.
    """
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return (cy * sp, sy * cp, -sy * sp, cy * cp)


class MaruIKNode(Node):
    def __init__(self):
        super().__init__("maru_ik_node")
        group = ReentrantCallbackGroup()
        for name, default in (
            ("target_topic", "/arm/target_point"),
            ("planning_frame", "base_link"),
            ("pregrasp_offset_z", 0.10),
            # D455가 상면을 보는 장착을 기준으로 depth 점(상면)에서 박스 중앙까지.
            # 실제 렌즈 장착/검출면에 따라 반드시 hand-eye 캘리브레이션으로 보정한다.
            ("grasp_offset_x", 0.0),
            ("grasp_offset_y", 0.0),
            ("grasp_offset_z", -0.0475),
            ("approach_pitch", -1.57),
            ("settle_time_sec", 3.0),
            ("motion_timeout_sec", 20.0),
            ("return_home_after_grasp", False),
        ):
            self.declare_parameter(name, default)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.pregrasp_offset_z = float(self.get_parameter("pregrasp_offset_z").value)
        self.grasp_offset = tuple(float(self.get_parameter(n).value) for n in (
            "grasp_offset_x", "grasp_offset_y", "grasp_offset_z"))
        self.approach_pitch = float(self.get_parameter("approach_pitch").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.return_home = bool(self.get_parameter("return_home_after_grasp").value)
        self._busy = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINT_NAMES, base_link_name=self.planning_frame,
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
        self.get_logger().info(f"Ready: {self.target_topic} -> {self.planning_frame}, CAN via ros2_control.")

    def on_target(self, msg: PointStamped) -> None:
        if not self._busy.acquire(blocking=False):
            self.get_logger().info("Grasp sequence is already active; ignoring duplicate detection.")
            return
        try:
            target = self._transform_target(msg)
        except Exception as exc:  # Never command a camera-frame point as base-frame data.
            self._busy.release()
            self.get_logger().warn(f"Target TF transform failed; ignoring target: {exc}")
            return
        threading.Thread(target=self._run_sequence, args=(target,), daemon=True).start()

    def _transform_target(self, msg: PointStamped):
        if not msg.header.frame_id:
            # Fake publisher intentionally uses an empty frame for already-base-frame points.
            return (msg.point.x, msg.point.y, msg.point.z)
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, msg.header.frame_id, Time(), timeout=Duration(seconds=0.5))
        point = do_transform_point(msg, transform)
        return (point.point.x, point.point.y, point.point.z)

    def _run_sequence(self, detected_xyz) -> None:
        try:
            x = detected_xyz[0] + self.grasp_offset[0]
            y = detected_xyz[1] + self.grasp_offset[1]
            z = detected_xyz[2] + self.grasp_offset[2]
            # URDF 좌표: X=우측, Y=수직(하단 +), Z=전방. base_joint는 Y축
            # pan이므로 수평 평면(X-Z)에서 목표 방위를 계산해야 한다.
            yaw = math.atan2(x, z)
            orientation = quaternion_from_yaw_x_pitch(yaw, self.approach_pitch)
            self.get_logger().info(f"Grasp point in {self.planning_frame}: ({x:.3f}, {y:.3f}, {z:.3f})")

            # 먼저 팔의 pre-grasp 경로가 가능한지 확인/실행한다. 이전에는
            # 그리퍼를 먼저 열어 두고 플래닝이 실패해, 사용자 입장에서는
            # "그리퍼만 움직인다"고 보이는 부작용이 있었다.
            if not self._move_arm((x, y, z + self.pregrasp_offset_z), orientation, False, "pregrasp"):
                return
            if not self._move_gripper(GRIPPER_OPEN, "open"):
                return
            # pymoveit2's Cartesian helper performs its own blocking spin, which is
            # unsafe while this node is already in a MultiThreadedExecutor.  Asking
            # move_group for this short segment keeps planning/execution asynchronous
            # and still collision-checks the complete path.
            if not self._move_arm((x, y, z), orientation, False, "descend"):
                return
            self.get_logger().info(f"Grasp point reached; settling for {self.settle_time_sec:.1f}s.")
            time.sleep(self.settle_time_sec)
            if not self._move_gripper(GRIPPER_CLOSED, "close"):
                return
            if not self._move_arm((x, y, z + self.pregrasp_offset_z), orientation, False, "lift"):
                return
            if self.return_home:
                self.arm.move_to_configuration(HOME_JOINTS)
                self._wait(self.arm, "return home")
            self.get_logger().info("Grasp complete; payload remains held for transport.")
        finally:
            self._busy.release()

    def _move_arm(self, position, orientation, cartesian: bool, label: str) -> bool:
        self.get_logger().info(f"MoveIt {label}")
        self.arm.move_to_pose(position=position, quat_xyzw=orientation, cartesian=cartesian,
                              cartesian_fraction_threshold=0.98)
        return self._wait(self.arm, label)

    def _move_gripper(self, position: float, label: str) -> bool:
        self.gripper.move_to_configuration([position])
        return self._wait(self.gripper, f"gripper {label}")

    def _wait(self, moveit: MoveIt2, label: str) -> bool:
        deadline = time.monotonic() + self.motion_timeout_sec
        while moveit.query_state() != MoveIt2State.IDLE:
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
