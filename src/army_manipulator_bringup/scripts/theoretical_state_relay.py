#!/usr/bin/env python3
"""제어 사이클마다 "이론상 명령값(reference)"을 별도 JointState로 중계하는
완전 독립 실행 스크립트 (path_visualizer.py와 동일한 패턴).

army_manipulator_bringup 패키지 의존성 없음 - rclpy + control_msgs +
sensor_msgs만 필요(전부 표준 ROS2 메시지 패키지). 그래서 colcon build로
설치될 필요 없이 워크스페이스만 source하면 `python3 theoretical_state_relay.py`로
바로 실행된다. 실기(jecs)에서 필요할 때만 따로 띄우는 보조 도구라 굳이
army_manipulator_bringup의 install(PROGRAMS) 목록에 넣어둘 이유가 없어서 뺐다.

joint_trajectory_controller(arm_controller/gripper_controller)는 매 사이클
control_msgs/JointTrajectoryControllerState를 발행하는데, 여기에 이미
reference(플래닝된 궤적이 이 순간 있어야 한다고 계산한 이론값)와
feedback(실제 엔코더로 읽은 값)이 같이 들어있다 - 새로 계산할 필요 없이
reference만 뽑아서 JointState로 재발행하면 된다.

이 토픽은 두 번째 robot_state_publisher(frame_prefix="theoretical_")가
구독해서 "이론상 자세" TF를 별도로 만드는 데 쓴다 - RViz에 반투명 고스트
로봇으로 겹쳐서, 실제 로봇(진짜 엔코더 기준)과 이론상 목표가 얼마나
차이나는지 한눈에 비교할 수 있다.

추가로 각 관절의 추적 오차(reference - feedback)를 라디안/도 단위로 매
report_interval_sec마다 터미널에도 찍는다 - RViz를 안 띄운 상태에서도
지금 얼마나 밀리고 있는지 숫자로 바로 확인할 수 있게.
"""

import math

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


RAD_TO_DEG = 180.0 / math.pi


class TheoreticalStateRelay(Node):
    def __init__(self):
        super().__init__("theoretical_state_relay")

        self.declare_parameter("output_topic", "/theoretical_joint_states")
        self.declare_parameter(
            "controller_state_topics",
            ["/arm_controller/controller_state", "/gripper_controller/controller_state"],
        )
        self.declare_parameter("report_interval_sec", 3.0)

        output_topic = str(self.get_parameter("output_topic").value)
        controller_topics = list(self.get_parameter("controller_state_topics").value)
        report_interval_sec = float(self.get_parameter("report_interval_sec").value)

        self._latest_error = {}  # joint_name -> (reference - feedback) rad, 최신 값

        self.pub = self.create_publisher(JointState, output_topic, 10)
        for topic in controller_topics:
            self.create_subscription(
                JointTrajectoryControllerState, topic,
                self._make_callback(topic), 10)
        self.create_timer(report_interval_sec, self._report_tracking_error)

        self.get_logger().info(
            f"theoretical_state_relay ready: {controller_topics} -> {output_topic} "
            f"(reference 필드를 중계, {report_interval_sec:.1f}s마다 추적오차도 출력)"
        )

    def _make_callback(self, topic: str):
        def callback(msg: JointTrajectoryControllerState) -> None:
            if not msg.reference.positions:
                return
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = list(msg.joint_names)
            out.position = list(msg.reference.positions)
            if msg.reference.velocities:
                out.velocity = list(msg.reference.velocities)
            self.pub.publish(out)

            # reference/feedback이 같은 메시지 안에 이미 다 있어서 새로 구독할
            # 필요 없이 여기서 바로 추적오차(reference - feedback)를 계산한다.
            if msg.feedback.positions and len(msg.feedback.positions) == len(msg.reference.positions):
                for name, ref, fb in zip(msg.joint_names, msg.reference.positions, msg.feedback.positions):
                    self._latest_error[name] = ref - fb
        return callback

    def _report_tracking_error(self) -> None:
        if not self._latest_error:
            return
        cols = [
            f"{name.replace('_joint', ''):>9}={err:+7.4f}rad({err*RAD_TO_DEG:+6.2f}deg)"
            for name, err in self._latest_error.items()
        ]
        self.get_logger().info("추적오차(이론-실제): " + "  ".join(cols))


def main(args=None):
    rclpy.init(args=args)
    node = TheoreticalStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
