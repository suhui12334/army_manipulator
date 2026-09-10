#!/usr/bin/env python3
"""supplybox 인식 상태를 주기적으로 출력하는 모니터.

summer_supply(ArmPickupNode)는 인식 성공한 프레임에만 /arm/target_point를
publish하고, 실패한 프레임에는 아무것도 안 보낸다(침묵) - 그래서 "인식됨/
인식 안됨"은 명시적 신호가 아니라 "최근에 /arm/target_point가 왔는지"로
추론해야 한다. 이 노드는 그 마지막 수신 시각을 추적해서 report_interval_sec
(기본 3초)마다 현재 상태를 로그로 찍고, 자율주행 쪽에서 구독해서 정지
트리거로 쓸 수 있게 std_msgs/Bool도 같이 publish한다.

[수정] 인식됐을 때는 감지 여부뿐 아니라 마지막으로 받은 목표 좌표(x,y,z,
frame_id)도 같이 출력한다 - 정렬(align)된 좌표가 실제로 어디로 나오는지
터미널에서 바로 확인할 수 있게.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


class DetectionStatusMonitor(Node):
    def __init__(self):
        super().__init__("detection_status_monitor")

        self.declare_parameter("target_topic", "/arm/target_point")
        self.declare_parameter("detected_topic", "/arm/target_detected")
        # 인식 노드가 몇 프레임 연속으로 실패해도 바로 "안 됨"으로
        # 깜빡이지 않게, 이 시간 안에 한 번이라도 왔으면 "인식됨"으로 본다.
        self.declare_parameter("detection_timeout_sec", 1.0)
        self.declare_parameter("report_interval_sec", 3.0)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.detected_topic = str(self.get_parameter("detected_topic").value)
        self.detection_timeout_sec = float(self.get_parameter("detection_timeout_sec").value)
        self.report_interval_sec = float(self.get_parameter("report_interval_sec").value)

        self._last_seen = None  # rclpy.time.Time, 마지막으로 target_point가 온 시각
        self._last_point = None  # (x, y, z, frame_id) - 마지막으로 받은 좌표

        self.create_subscription(PointStamped, self.target_topic, self._on_target, 10)
        # 자율주행 쪽이 늦게 구독해도 최신 상태를 바로 받을 수 있게 latched.
        detected_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.detected_pub = self.create_publisher(Bool, self.detected_topic, detected_qos)
        self.create_timer(self.report_interval_sec, self._on_report_timer)

        self.get_logger().info(
            f"detection_status_monitor ready: {self.target_topic} -> 인식 상태를 "
            f"{self.report_interval_sec:.1f}s마다 출력, {self.detected_topic}(Bool)로도 publish "
            f"(timeout={self.detection_timeout_sec:.1f}s)."
        )

    def _on_target(self, msg: PointStamped) -> None:
        self._last_seen = self.get_clock().now()
        self._last_point = (msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)

    def _on_report_timer(self) -> None:
        now = self.get_clock().now()
        if self._last_seen is not None:
            elapsed_sec = (now - self._last_seen).nanoseconds / 1e9
        else:
            elapsed_sec = None

        detected = elapsed_sec is not None and elapsed_sec <= self.detection_timeout_sec
        self.detected_pub.publish(Bool(data=detected))

        if detected:
            x, y, z, frame_id = self._last_point
            self.get_logger().info(
                f"인식됨 (마지막 감지 {elapsed_sec:.2f}초 전) - 좌표({frame_id}): "
                f"x={x:.4f} y={y:.4f} z={z:.4f}"
            )
        elif elapsed_sec is not None:
            self.get_logger().info(f"인식 안됨 (마지막 감지로부터 {elapsed_sec:.1f}초 경과)")
        else:
            self.get_logger().info("인식 안됨 (아직 한 번도 감지 안됨)")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
