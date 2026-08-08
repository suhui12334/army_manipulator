#!/usr/bin/env python3
"""base_link -> tcp_link TF를 매 주기 조회해 누적, TCP 궤적을 nav_msgs/Path로 publish.

알고리즘:
  1. /joint_states가 갱신될 때마다(또는 고정 주기 타이머로) tf2 lookupTransform 호출
  2. base_link 기준 tcp_link의 현재 위치를 얻음 (forward kinematics 결과,
     별도 FK 계산 없이 robot_state_publisher가 이미 채워둔 TF 트리를 그대로 사용)
  3. 이전 위치와 일정 거리(min_dist) 이상 떨어졌을 때만 point 추가
     (거의 안 움직일 때 매 사이클 점 찍으면 UI 쪽 데이터量만 늘어나고 시각적 의미 없음)
  4. nav_msgs/Path (누적된 PoseStamped 배열)로 publish -> UI가 구독해서 3D 라인으로 그림
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class TcpTrailPublisher(Node):
    def __init__(self):
        super().__init__("tcp_trail_publisher")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "tcp_link")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("min_dist_m", 0.002)  # 2mm 이상 움직였을 때만 점 추가
        self.declare_parameter("max_points", 2000)   # 메모리/UI 부하 방지용 상한

        self.base_frame = self.get_parameter("base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value
        self.min_dist = self.get_parameter("min_dist_m").value
        self.max_points = self.get_parameter("max_points").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, "/maru/tcp_path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.base_frame
        self._last_point = None

        hz = self.get_parameter("publish_hz").value
        self.create_timer(1.0 / hz, self.on_timer)

        self.get_logger().info("tcp_trail_publisher ready")

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame,
                rclpy.time.Time(),  # 최신 available transform
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return  # TF 아직 준비 안 됐으면 이번 주기 skip

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        if self._last_point is not None:
            dx, dy, dz = x - self._last_point[0], y - self._last_point[1], z - self._last_point[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.min_dist:
                return  # 거의 안 움직였으면 점 추가 안 함

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = tf.transform.rotation

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses.pop(0)  # 오래된 점부터 버림 (링 버퍼처럼 동작)

        self.path_msg.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path_msg)
        self._last_point = (x, y, z)


def main():
    rclpy.init()
    node = TcpTrailPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()