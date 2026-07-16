#!/usr/bin/env python3
"""타겟 검출 노드 스켈레톤.

입력: RealSense의 정렬된 color/depth 이미지 + camera_info.
출력: `geometry_msgs/PointStamped`(기본 `/maru/target/point`) — maru_ik_node가
이미 구독하고 있는 토픽과 동일하며, 여기서 publish하는 점의 header.frame_id를
카메라 optical frame으로 두면 maru_ik_node 쪽 TF lookup(`target_callback`)이
알아서 planning_frame(base_link)으로 변환해준다. 즉 이 노드는 카메라 프레임
기준 3D 좌표만 정확히 내보내면 된다.

TODO(detection): 실제 타겟 검출 알고리즘은 아직 미정(색상 기반/ArUco/딥러닝 등
사용자가 추후 결정). `detect_target_pixel()`에 색상 이미지를 넣고 픽셀 좌표
(u, v)를 반환하도록 구현할 것 — 지금은 항상 None을 반환하는 자리표시자다.
"""

import math
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    import message_filters
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import PointStamped
    from cv_bridge import CvBridge
except ModuleNotFoundError:  # pragma: no cover - 테스트 환경에서 ROS/cv_bridge 미설치 시에도 import 가능
    rclpy = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    message_filters = None
    Image = None
    CameraInfo = None
    PointStamped = None
    CvBridge = None


def deproject_pixel_to_point(
    u: float, v: float, depth_m: float, fx: float, fy: float, cx: float, cy: float
) -> Tuple[float, float, float]:
    """핀홀 카메라 모델로 픽셀(u, v) + depth(m)를 카메라 프레임 3D 점으로 변환한다.

    camera_info의 K = [fx, 0, cx, 0, fy, cy, 0, 0, 1] 값을 그대로 사용.
    광학 좌표계(OpenCV 관례: X-right, Y-down, Z-forward) 기준 결과를 반환하므로,
    카메라 optical frame(TF)에 그대로 실어 publish하면 된다.
    """
    if depth_m <= 0.0 or math.isnan(depth_m):
        raise ValueError(f"invalid depth value: {depth_m}")
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return (x, y, z)


def detect_target_pixel(color_image) -> Optional[Tuple[int, int]]:
    """TODO(detection): 색상 이미지에서 타겟 픽셀 좌표를 찾는 실제 알고리즘을
    여기에 구현할 것 (색상 기반 필터링 / ArUco 마커 / 딥러닝 검출기 등).
    지금은 검출 결과가 없다고 보고 항상 None을 반환한다.
    """
    return None


class TargetDetectorNode(Node):
    """정렬된 color/depth 이미지에서 타겟의 3D 위치를 뽑아 publish한다."""

    def __init__(self):
        super().__init__("target_detector_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("target_topic", "/maru/target/point")
        self.declare_parameter("depth_scale", 0.001)  # RealSense 기본: mm -> m

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.depth_scale = float(self.get_parameter("depth_scale").value)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.camera_info = None

        qos = QoSProfile(depth=10)
        if ReliabilityPolicy is not None and DurabilityPolicy is not None:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos
        )

        color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05
        )
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info(
            f"target_detector_node ready. color={self.color_topic}, depth={self.depth_topic}, "
            f"target_topic={self.target_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def image_callback(self, color_msg: Image, depth_msg: Image) -> None:
        if self.camera_info is None:
            self.get_logger().warn("camera_info not received yet; skipping frame")
            return

        pixel = detect_target_pixel(color_msg)
        if pixel is None:
            return

        u, v = pixel
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = float(depth_image[int(v), int(u)]) * self.depth_scale

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        try:
            x, y, z = deproject_pixel_to_point(u, v, depth_m, fx, fy, cx, cy)
        except ValueError as exc:
            self.get_logger().warn(f"deproject failed: {exc}")
            return

        point = PointStamped()
        point.header = color_msg.header
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.target_pub.publish(point)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is not available in this environment")

    rclpy.init(args=args)
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
