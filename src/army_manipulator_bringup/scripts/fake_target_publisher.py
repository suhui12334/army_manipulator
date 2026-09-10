#!/usr/bin/env python3
"""시뮬레이션/테스트용 가짜 타겟 좌표 발행 노드.

실물 뎁스카메라 없이도 depth_camera_ik_bringup.launch.py의 다운스트림
(maru_ik_node의 MoveIt IK 역산 -> arm_controller/gripper_controller 실행)을
검증할 수 있도록, summer_supply(ArmPickupNode)가 하는 일(카메라 좌표 -> /arm/target_point
publish)을 파라미터로 받은 고정 좌표로 흉내낸다.

launch에서 sim_target:=true 로 켜면 realsense_bringup.launch.py +
summer_supply(dolbotz 패키지) 대신 이 노드가 뜬다(depth_camera_ik_bringup.launch.py 참고).
"""

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node


class FakeTargetPublisher(Node):
    """파라미터로 받은 (x, y, z)를 지연 후 한 번(또는 주기적으로) publish한다."""

    def __init__(self):
        super().__init__("fake_target_publisher")

        self.declare_parameter("target_topic", "/arm/target_point")
        self.declare_parameter("frame_id", "")
        # 기본값: FK로 실측 검증된 도달 가능 지점(base_joint=0, shoulder=elbow=
        # wrist=-0.4rad 벤드에서 나오는 wrist_link 위치). 다른 좌표를 던지고
        # 싶으면 launch 인자로 x/y/z를 덮어쓰면 된다.
        self.declare_parameter("x", 0.0313)
        self.declare_parameter("y", 0.2279)
        self.declare_parameter("z", 0.423)
        self.declare_parameter("delay_sec", 3.0)
        self.declare_parameter("repeat", False)
        self.declare_parameter("rate_hz", 1.0)

        self.target_topic = self.get_parameter("target_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.delay_sec = float(self.get_parameter("delay_sec").value)
        self.repeat = bool(self.get_parameter("repeat").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)

        # delay_sec: robot_state_publisher/move_group/maru_ik_node가 다 뜬 뒤에
        # publish하도록 기동 직후 바로 쏘지 않고 한 번 지연시킨다.
        self._startup_timer = self.create_timer(self.delay_sec, self._on_startup_timer)

        self.get_logger().info(
            f"fake_target_publisher ready. target=({self.x}, {self.y}, {self.z}) "
            f"-> {self.target_topic} in {self.delay_sec}s (repeat={self.repeat})"
        )

    def _on_startup_timer(self) -> None:
        self._startup_timer.cancel()
        self.publish_target()
        if self.repeat:
            self.create_timer(1.0 / self.rate_hz, self.publish_target)

    def publish_target(self) -> None:
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published fake target ({self.x}, {self.y}, {self.z}) to {self.target_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = FakeTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
