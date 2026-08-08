#!/usr/bin/env python3
"""타겟 검출 노드.

dolbotZ(9o9hz/dolbotZ)의 arm_pickup_node와 동일한 패턴으로 맞춘 것 —
army_manipulator가 나중에 dolbotZ/src/arm으로 합쳐질 예정이라, 토픽 이름/
파라미터/입력 포맷(CompressedImage)/깊이 샘플링 방식을 처음부터 통일해둔다.

입력: RealSense의 정렬된 color/depth 압축 이미지(CompressedImage,
compressedDepth) + camera_info. CompressedImage를 쓰는 이유는 원격/저대역폭
네트워크(무선 조종 등)에서도 raw Image보다 훨씬 가볍기 때문이다
(dolbotZ arm_pickup.py와 동일한 이유).

출력:
  /arm/target_point (geometry_msgs/PointStamped) — maru_ik_node가 구독하는
  토픽. header.frame_id를 카메라 optical frame으로 두면 maru_ik_node 쪽
  TF lookup(`target_callback`)이 알아서 planning_frame(base_link)으로
  변환해준다. 즉 이 노드는 카메라 프레임 기준 3D 좌표만 정확히 내보내면 된다.
  /arm/debug_image/compressed (sensor_msgs/CompressedImage) — 매 프레임
  RGB(탐지 성공 시 바운딩박스 오버레이). dolbotZ의 arm_visualizer_node로
  바로 볼 수 있다.

검출: 사전 학습된 YOLO(ultralytics) 가중치로 color 이미지에서 `target_class`
클래스(기본 "supplybox")를 찾아 confidence가 가장 높은 박스의 중심 픽셀
주변 depth ROI 중앙값을 깊이로 사용한다(단일 픽셀보다 노이즈에 강함).
가중치 파일은 기본적으로
`<share>/army_manipulator_bringup/config/models/supplybest_openvino_model`
(GPU 없는 환경 CPU 추론용 OpenVINO IR)에서 읽으며, `model_path` 파라미터로
덮어쓸 수 있다(config/models/README.md 참고).
"""

import math
import struct
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import message_filters
    from sensor_msgs.msg import CompressedImage, CameraInfo
    from geometry_msgs.msg import PointStamped
    from cv_bridge import CvBridge
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # pragma: no cover - 테스트 환경에서 ROS/cv_bridge 미설치 시에도 import 가능
    rclpy = None
    Node = object
    qos_profile_sensor_data = None
    message_filters = None
    CompressedImage = None
    CameraInfo = None
    PointStamped = None
    CvBridge = None
    get_package_share_directory = None

try:
    from ultralytics import YOLO
except ModuleNotFoundError:  # pragma: no cover - ultralytics는 pip 설치, 가중치 없는 환경에서도 import 가능해야 함
    YOLO = None


_COMPRESSED_DEPTH_HEADER = struct.Struct("=iff")
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def decode_compressed_depth(msg) -> np.ndarray:
    """image_transport의 compressedDepth 페이로드를 원본 depth 배열로 디코드한다.

    RealSense는 보통 16UC1 depth를 발행하고 compressedDepth는 이를 12바이트
    전송 헤더 뒤에 무손실 16비트 PNG로 담는다. 32FC1(역깊이) 페이로드는 그
    헤더의 양자화 파라미터로 복원한다.
    """
    format_text = str(msg.format)
    if "compressedDepth" not in format_text:
        raise ValueError(f"compressedDepth 형식이 아닙니다: {format_text!r}")

    payload = bytes(msg.data)
    png_offset = payload.find(_PNG_SIGNATURE, _COMPRESSED_DEPTH_HEADER.size)
    if png_offset < 0:
        raise ValueError("compressedDepth PNG 시그니처를 찾을 수 없습니다.")

    encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
    decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if decoded is None:
        raise ValueError("compressedDepth PNG 디코딩에 실패했습니다.")
    if decoded.ndim != 2 or decoded.dtype != np.uint16:
        raise ValueError(
            f"예상하지 못한 compressedDepth 이미지: shape={decoded.shape}, dtype={decoded.dtype}"
        )

    source_encoding = format_text.split(";", 1)[0].strip()
    if source_encoding in ("16UC1", "mono16"):
        return decoded
    if source_encoding == "32FC1":
        if len(payload) < _COMPRESSED_DEPTH_HEADER.size:
            raise ValueError("compressedDepth 설정 헤더가 손상되었습니다.")
        _compression_format, depth_quant_a, depth_quant_b = _COMPRESSED_DEPTH_HEADER.unpack_from(payload)
        depth = np.zeros(decoded.shape, dtype=np.float32)
        valid = decoded != 0
        depth[valid] = depth_quant_a / (decoded[valid].astype(np.float32) + depth_quant_b)
        return depth

    raise ValueError(f"지원하지 않는 Depth 인코딩입니다: {source_encoding!r}")


def to_meters(depth_raw: np.ndarray) -> np.ndarray:
    """16UC1(mm) depth는 m로 변환, 32FC1(이미 m 단위)은 그대로 float32로."""
    if depth_raw.dtype == np.uint16:
        return depth_raw.astype(np.float32) * 0.001
    return depth_raw.astype(np.float32)


def sample_depth(depth_m: np.ndarray, u: int, v: int, radius: int, max_depth_m: float) -> float:
    """bbox 중심 주변 patch의 유효 깊이 중앙값(m). 유효 샘플 부족하면 0.0."""
    h, w = depth_m.shape
    patch = depth_m[max(0, v - radius) : min(h, v + radius + 1), max(0, u - radius) : min(w, u + radius + 1)]
    valid = patch[(patch > 0.05) & (patch < max_depth_m)]
    return float(np.median(valid)) if valid.size >= 3 else 0.0


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


def _package_share_dir() -> Optional[Path]:
    if get_package_share_directory is None:
        return None
    try:
        return Path(get_package_share_directory("army_manipulator_bringup"))
    except Exception:  # noqa: BLE001 - 패키지 미설치/미빌드 환경에서도 노드 자체는 뜰 수 있게
        return None


def _repo_root_dev() -> Optional[Path]:
    """colcon build 없이 소스에서 바로 실행하는 개발 환경 폴백.

    이 파일(__file__) 위치부터 위로 올라가며 config/와 scripts/가 모두
    있는 첫 조상 디렉토리를 army_manipulator_bringup 패키지 루트로 판정한다.
    """
    here = Path(__file__).resolve()
    for candidate in here.parents:
        if (candidate / "config").is_dir() and (candidate / "scripts").is_dir():
            return candidate
    return None


def default_model_path() -> str:
    """config/models/supplybest_openvino_model의 절대경로를 실행 환경과 무관하게 계산.

    dolbotZ(9o9hz/dolbotZ) dolbotz.utils.paths.get_models_dir()와 동일한
    우선순위: colcon install 환경(ament_index) -> 소스 개발 환경(__file__ 상위 탐색).
    과거 dolbotZ에서 이 경로가 절대경로로 하드코딩되어 있다가 사용자/머신이
    바뀌며 깨졌던 전례(커밋 6056cf8)를 여기서도 원천적으로 피한다.
    """
    share_dir = _package_share_dir()
    if share_dir is not None:
        return str(share_dir / "config" / "models" / "supplybest_openvino_model")
    dev_root = _repo_root_dev()
    if dev_root is not None:
        return str(dev_root / "config" / "models" / "supplybest_openvino_model")
    return ""


class TargetDetectorNode(Node):
    """정렬된 color/depth 압축 이미지에서 타겟의 3D 위치를 뽑아 publish한다."""

    def __init__(self):
        super().__init__("target_detector_node")

        self.declare_parameter("model_path", default_model_path())
        self.declare_parameter("target_class", "supplybox")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("infer_size", 320)
        self.declare_parameter("depth_roi_radius", 5)
        # 팔의 집기 작업반경을 넘는 검출(예: 1.7 m)을 MoveIt에 보내면
        # 계획은 반드시 실패한다. 모바일 베이스가 물자 근처로 접근한 뒤
        # 검출하도록 기본 상한을 0.8 m로 둔다.
        self.declare_parameter("max_depth_m", 0.8)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("target_topic", "/arm/target_point")
        self.declare_parameter("debug_topic", "/arm/debug_image/compressed")

        model_path = str(self.get_parameter("model_path").value) or default_model_path()
        self.target_class = str(self.get_parameter("target_class").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.infer_size = int(self.get_parameter("infer_size").value)
        self.depth_roi_radius = int(self.get_parameter("depth_roi_radius").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = self._load_model(model_path)
        self.fx = self.fy = self.cx = self.cy = None
        self._camera_info_logged = False

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )

        color_sub = message_filters.Subscriber(
            self, CompressedImage, self.color_topic, qos_profile=qos_profile_sensor_data
        )
        depth_sub = message_filters.Subscriber(
            self, CompressedImage, self.depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=30, slop=0.20
        )
        self.sync.registerCallback(self._on_frames)

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, 10)

        self.get_logger().info(
            f"target_detector_node ready. color={self.color_topic}, depth={self.depth_topic}, "
            f"target_class={self.target_class}, conf>={self.conf_threshold}, "
            f"target_topic={self.target_topic}"
        )

    def _load_model(self, model_path: str):
        if not model_path:
            self.get_logger().error(
                "model_path가 비어 있다. config/models/에 가중치를 배치하거나 "
                "model_path 파라미터로 경로를 지정할 것."
            )
            return None
        if YOLO is None:
            self.get_logger().error(
                "ultralytics가 설치되어 있지 않아 YOLO 검출을 사용할 수 없다 (pip install ultralytics)."
            )
            return None
        try:
            model = YOLO(model_path)
        except Exception as exc:  # noqa: BLE001 - 가중치 로드 실패는 노드를 죽이지 않고 detect를 비활성화
            self.get_logger().error(f"YOLO 가중치 로드 실패 ({model_path}): {exc}")
            return None
        self.get_logger().info(f"모델 로드 완료: {model_path}")
        return model

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        if not self._camera_info_logged:
            self.get_logger().info(f"camera_info 수신 | fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
            self._camera_info_logged = True

    def _on_frames(self, color_msg: CompressedImage, depth_msg: CompressedImage) -> None:
        if self.fx is None:
            self.get_logger().warn("camera_info not received yet; skipping frame", throttle_duration_sec=5.0)
            return
        if self.model is None:
            self.get_logger().error("YOLO model이 None이어서 프레임 처리를 중단합니다.", throttle_duration_sec=5.0)
            return

        try:
            color = self.bridge.compressed_imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_m = to_meters(decode_compressed_depth(depth_msg))
        except Exception as exc:  # noqa: BLE001 - 손상된 프레임 하나 때문에 노드가 죽으면 안 됨
            self.get_logger().error(f"압축 카메라 이미지 디코딩 실패: {exc}", throttle_duration_sec=5.0)
            return

        results = self.model(color, imgsz=self.infer_size, verbose=False)

        best = None  # (conf, X, Y, Z, bbox, u, v)
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue
                cls_name = self.model.names.get(int(box.cls[0]), "")
                if cls_name != self.target_class:
                    continue
                if best is not None and conf <= best[0]:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                z = sample_depth(depth_m, u, v, self.depth_roi_radius, self.max_depth_m)
                if z <= 0.0:
                    continue

                try:
                    x, y, z = deproject_pixel_to_point(u, v, z, self.fx, self.fy, self.cx, self.cy)
                except ValueError:
                    continue
                best = (conf, x, y, z, (x1, y1, x2, y2), u, v)

        if best is not None:
            conf, x, y, z, bbox, u, v = best

            point = PointStamped()
            point.header = color_msg.header
            point.point.x = x
            point.point.y = y
            point.point.z = z
            self.target_pub.publish(point)

            bx1, by1, bx2, by2 = (int(c) for c in bbox)
            cv2.rectangle(color, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
            cv2.circle(color, (u, v), 6, (0, 0, 255), -1)
            cv2.putText(
                color,
                f"{self.target_class} {conf:.2f} | Z={z:.2f}m",
                (bx1, by1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        debug_msg = self.bridge.cv2_to_compressed_imgmsg(color, dst_format="jpg")
        debug_msg.header = color_msg.header
        self.debug_pub.publish(debug_msg)


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
