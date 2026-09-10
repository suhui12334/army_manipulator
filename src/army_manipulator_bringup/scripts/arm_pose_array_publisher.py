#!/usr/bin/env python3
"""각 관절 프레임의 실제 TF 좌표를 PoseArray로 publish (DolbotZ-Center 로봇팔 시각화용).

DolbotZ-Center README 8절의 두 방식 중 tcp_trail_publisher.py와 동일한 패턴으로
"ROS 2 노드에서 TF를 조회해 각 관절의 위치를 geometry_msgs/msg/PoseArray로 발행" 쪽을
구현한다 - app.js의 링크 길이/영점 오프셋/회전 방향 톱니바퀴 설정(순수 각도 누적
근사)은 base_joint(터렛 회전)가 0이 아닐 때 실제 3차원 형상과 어긋나므로, 웹 UI가
계산 없이 그대로 그릴 수 있는 실좌표를 여기서 만들어 보낸다.

base_frame 기본값은 base_actuator다 - base_link는 base_joint의 자식이라 base_joint
회전과 함께 도는 프레임이라(SRDF arm 체인 고정 루트는 base_actuator, commit 686ee47
참고) 여기 쓰면 base_joint가 돌 때마다 이미 찍힌 값들의 기준 자체가 흔들린다.

[수정] DolbotZ-Center의 로봇팔 패널은 X-Z 평면 2D 캔버스다 - 순수 3D 좌표를
그대로 보내면 클라이언트가 y를 알아서 무시해야 하는데, "월드좌표 기준 X-Z
투영값"이라는 계약을 서버 쪽에서 명시적으로 만족시키기 위해 y는 항상 0으로
투영해서 보낸다(주석 없이 x/y/z를 다 채우면 나중에 이 토픽을 다른 소비자가
"진짜 3D 좌표"로 오해하기 쉬움). 같은 이유로 orientation도 평면 투영과 맞지
않는 3D 회전 정보라 항상 identity로 비워서 보낸다.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseArray, Pose

DEFAULT_FRAMES = ["base_actuator", "shoulder_link", "elbow_link", "wrist_link", "tcp_link"]


class ArmPoseArrayPublisher(Node):
    def __init__(self):
        super().__init__("arm_pose_array_publisher")

        self.declare_parameter("base_frame", "base_actuator")
        self.declare_parameter("link_frames", DEFAULT_FRAMES)
        self.declare_parameter("publish_hz", 20.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.link_frames = [str(f) for f in self.get_parameter("link_frames").value]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseArray, "/arm/joint_pose_array", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self.on_timer)

        self.get_logger().info(
            f"arm_pose_array_publisher ready: {self.base_frame} -> {self.link_frames}")

    def on_timer(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        for frame in self.link_frames:
            pose = Pose()
            if frame == self.base_frame:
                # 자기 자신(고정 루트)은 항상 원점 - lookup_transform(A, A, ...)이
                # identity를 주긴 하지만 TF 트리가 그 프레임을 안 갖고 있어도
                # (예: base_frame 오타) 원점으로 항상 표시되게 별도 처리.
                pose.orientation.w = 1.0
                msg.poses.append(pose)
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame,
                    rclpy.time.Time(),  # 최신 available transform
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                return  # 이번 주기는 스킵 - 일부 프레임만 채운 배열을 보내면 웹 UI가 관절 순서를 오해석함
            # 월드(base_actuator) 기준 X-Z 평면 투영만 보낸다 - y는 항상 0.0,
            # orientation도 항상 identity(위 모듈 docstring [수정] 참고).
            pose.position.x = tf.transform.translation.x
            pose.position.y = 0.0
            pose.position.z = tf.transform.translation.z
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmPoseArrayPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
