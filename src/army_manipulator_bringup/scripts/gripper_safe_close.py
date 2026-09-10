#!/usr/bin/env python3
"""그리퍼를 천천히 닫으면서, 전류가 threshold를 넘으면 즉시 취소하는
안전한 수동 캘리브레이션 도구.

maru_ik_node.py의 _close_gripper_with_current_feedback()과 같은 전류 변환/
정지 로직을 쓰지만, 여기서는 MoveIt이 아니라 gripper_controller
(FollowJointTrajectory)에 직접 여러 중간 waypoint를 보내서 "서서히" 닫는다
(한 번에 목표점 하나만 보내면 컨트롤러 자체 가감속 프로파일대로 빠르게
움직인다 - 관찰하면서 손으로 물체를 넣기엔 너무 빠르다).

gripper_current_logger.py를 다른 터미널에 띄워서 곡선/CSV를 같이 보면서,
이 스크립트로 실제 "닫기" 동작을 안전하게 반복 실행하는 용도.

쓰는 법:
  ros2 run army_manipulator_bringup gripper_safe_close.py --ros-args \
    -p current_threshold_ma:=100.0 -p duration_sec:=8.0

  # 다른 값으로 반복 테스트하고 싶으면 threshold만 바꿔서 다시 실행.
  ros2 run army_manipulator_bringup gripper_safe_close.py --ros-args -p current_threshold_ma:=80.0

주의: 이 스크립트는 "닫기" 방향 전용이다. 열기(리셋)는 목표를 GRIPPER_OPEN
근처로 주고 다시 실행하거나, 직접 action goal을 보내면 된다:
  ros2 run army_manipulator_bringup gripper_safe_close.py --ros-args \
    -p target_position:=0.07363 -p current_threshold_ma:=100000
(open 방향은 물릴 위험이 없으니 threshold를 사실상 무제한으로 둬서 그냥
끝까지 열리게 하는 것.)
"""

import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# maru_ik_node.py의 GRIPPER_OPEN/GRIPPER_CLOSED와 동일(2026-08-30 갱신).
GRIPPER_CLOSED_DEFAULT = 1.8294
JOINT_NAME = "gripper_joint"


class GripperSafeClose(Node):
    def __init__(self):
        super().__init__("gripper_safe_close")

        self.declare_parameter("target_position", GRIPPER_CLOSED_DEFAULT)
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("steps", 40)
        self.declare_parameter("current_threshold_ma", 100.0)  # maru_ik_node.py와 동일 실측값
        self.declare_parameter("gripper_current_lsb_ma", 2.69)

        self.target_position = float(self.get_parameter("target_position").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.steps = max(2, int(self.get_parameter("steps").value))
        self.threshold_ma = float(self.get_parameter("current_threshold_ma").value)
        self.lsb_ma = float(self.get_parameter("gripper_current_lsb_ma").value)

        self._current_ma = 0.0
        self._start_position = None
        self._goal_handle = None
        self._tripped = False

        self._state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_state, qos_profile_sensor_data)
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")

    def _on_state(self, msg: JointState) -> None:
        try:
            idx = msg.name.index(JOINT_NAME)
        except ValueError:
            return
        if idx < len(msg.effort):
            self._current_ma = abs(msg.effort[idx]) * self.lsb_ma
        if self._start_position is None and idx < len(msg.position):
            self._start_position = msg.position[idx]

        if (
            self._goal_handle is not None
            and not self._tripped
            and self._current_ma >= self.threshold_ma
        ):
            self._tripped = True
            self.get_logger().info(
                f"Current {self._current_ma:.1f}mA reached threshold {self.threshold_ma:.1f}mA "
                "- cancelling motion NOW."
            )
            self._goal_handle.cancel_goal_async()

    def run(self) -> None:
        self.get_logger().info("Waiting for /joint_states to get current gripper position...")
        deadline = time.monotonic() + 5.0
        while self._start_position is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._start_position is None:
            self.get_logger().error("No /joint_states received; is control_bringup.launch.py running?")
            return

        self.get_logger().info("Waiting for /gripper_controller/follow_joint_trajectory action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available - is gripper_controller active?")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT_NAME]
        for i in range(1, self.steps + 1):
            frac = i / self.steps
            position = self._start_position + frac * (self.target_position - self._start_position)
            t = frac * self.duration_sec
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Closing gripper_joint {self._start_position:.4f} -> {self.target_position:.4f} rad "
            f"over {self.duration_sec:.1f}s ({self.steps} waypoints), "
            f"threshold={self.threshold_ma:.1f}mA."
        )

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by gripper_controller.")
            return

        result_future = self._goal_handle.get_result_async()
        last_print = 0.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                self.get_logger().info(f"  current={self._current_ma:6.1f}mA (threshold={self.threshold_ma:.1f}mA)")

        status = result_future.result().status
        if self._tripped:
            self.get_logger().info(
                f"Stopped by current spike at {self._current_ma:.1f}mA "
                f"(grasp-success 후보). status={status}"
            )
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                "Reached target position without a current spike - likely nothing was gripped."
            )
        else:
            self.get_logger().error(f"Motion ended with status={status} (not succeeded, not current-tripped).")


def main(args=None):
    rclpy.init(args=args)
    node = GripperSafeClose()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
