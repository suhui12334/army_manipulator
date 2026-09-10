"""3D 좌표 추출(서플라이박스 인식) bringup - 카메라 드라이버는 포함하지 않는다.

[2026-08-30] 예전엔 여기서 realsense_bringup.launch.py(카메라)까지 같이
띄웠는데, 실기 디버깅 중 카메라만 따로 켜고 끄고 확인해야 할 일이 계속
생기면서(시리얼/네임스페이스 문제 등) 자연스럽게 "카메라 / 인식 / 제어"
세 터미널로 굳어졌다. 그래서 카메라 기동은 완전히 분리했다 - 이 launch는
이제 인식 노드(또는 sim_target:=true일 때 fake_target_publisher)만
담당한다. 카메라는 따로 띄울 것:
  ros2 launch army_manipulator_bringup realsense_bringup.launch.py
(realsense_bringup.launch.py는 원래도 독립 실행 가능했다 - 이 파일에서
include를 뺀 것뿐, 새 파일을 만들진 않았다.)

[2026-08-30, 인식 노드 교체] army_manipulator_bringup 자체의
target_detector_node.py를 지우고, dolbotz 패키지가 관리하는
summer_supply(ArmPickupNode, dolbotz/missions/summer_supply.py)로
바꿨다 - 같은 역할(서플라이박스 검출 -> /arm/target_point,
/arm/debug_image/compressed publish)이 두 곳에 중복 구현돼 있었는데,
summer_supply.py가 팀에서 계속 관리/개선 중인 쪽(YOLO/RF-DETR 백엔드 선택,
최신 모델 supplyboxv3.pt 자동 사용)이라 이쪽으로 통일했다. dolbotz
패키지가 빌드돼 있어야 한다(colcon build --packages-select dolbotz).

이 launch 혼자서는 아무것도 움직이지 않는다 - 결과는 /arm/target_point
(PointStamped)로 publish만 되고, 이걸 구독해서 TF+IK+실행을 하는 쪽은
control_bringup.launch.py(maru_ik_node)다.

  (sim_target:=false, 기본) summer_supply(ArmPickupNode)가 compressed
    color/depth를 구독해(카메라는 realsense_bringup.launch.py를 따로
    띄워야 함) 3D 타겟 좌표를 뽑아 /arm/target_point로 publish
  (sim_target:=true) fake_target_publisher가 실물 카메라 없이 파라미터로
    받은 고정 좌표를 지연 후 /arm/target_point로 publish

카메라+인식을 한 번에 띄우고 싶으면(빠른 시뮬레이션 검증 등)
depth_camera_ik_bringup.launch.py를 쓸 것 - 그건 여전히 카메라까지 같이
묶어서 띄운다.

detection: summer_supply(ArmPickupNode)는 YOLO(ultralytics, supplyboxv3.pt -
2026-08-30 RF-DETR와 비교 후 이걸로 확정, detector_backend 선택지는 제거됨)로
color 이미지에서 서플라이박스를 검출하고, confidence가 가장 높은 박스
중심 주변 depth ROI 중앙값을 깊이로 쓴다. 가중치 파일(*.pt)은 git으로
동기화 안 되니(.gitignore) 실물 카메라 모드(sim_target:=false) 전에
config/models/에 준비됐는지 먼저 확인할 것.

주의(compressed transport): color/depth 기본 토픽이 CompressedImage라서,
realsense2_camera가 이 토픽을 내보내려면 ros-humble-compressed-image-transport
+ ros-humble-compressed-depth-image-transport가 설치되어 있어야 한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "sim_target",
            default_value="false",
            description=(
                "true면 realsense2_camera 드라이버/summer_supply 대신 "
                "fake_target_publisher가 target_x/y/z 좌표를 /arm/target_point로 "
                "publish한다. 실물 카메라 없이 다운스트림만 시뮬레이션으로 검증할 때 사용."
            ),
        ),
        # ----- 실제 주행: 뎁스카메라 + 서플라이박스 검출 (sim_target:=false일 때만 사용) -----
        # [2026-08-30] JECS 팔 카메라는 realsense_bringup.launch.py에서
        # camera_namespace=arm으로 뜬다 - 기본 네임스페이스로 두면 JECS의
        # 다른 카메라를 잘못 구독하는 원인이 됐었다.
        DeclareLaunchArgument(
            "color_topic",
            default_value="/arm/camera/color/image_raw/compressed",
            description="summer_supply로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/arm/camera/aligned_depth_to_color/image_raw/compressedDepth",
            description="summer_supply로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/arm/camera/color/camera_info",
            description="summer_supply로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description=(
                "summer_supply로 그대로 전달(sim_target:=false일 때만 사용). "
                "빈 문자열이면 기본 파일(config/models/supplyboxv3.pt) 사용."
            ),
        ),
        DeclareLaunchArgument(
            "target_class",
            default_value="supplybox",
            description="summer_supply로 그대로 전달. 결과에 붙일 클래스 이름(로그/오버레이 표시용).",
        ),
        DeclareLaunchArgument(
            "conf_threshold",
            default_value="0.5",
            description="summer_supply로 그대로 전달. 검출 confidence 임계값.",
        ),
        DeclareLaunchArgument(
            "infer_size",
            default_value="320",
            description="summer_supply로 그대로 전달. YOLO 추론 해상도(GPU 없는 환경 속도용, rfdetr는 미사용).",
        ),
        DeclareLaunchArgument(
            "depth_roi_radius",
            default_value="5",
            description="summer_supply로 그대로 전달. 타겟 중심 픽셀 주변 깊이 샘플링 반경(px).",
        ),
        DeclareLaunchArgument(
            "max_depth_m",
            default_value="1.0",
            description=(
                "summer_supply로 전달하는 YOLO bbox 중심 depth의 직접 팔 진입 상한(m). "
                "1.0m 이하는 /arm/target_point, 초과는 /arm/forward_command 발행."
            ),
        ),
        # ----- 시뮬레이션: 가짜 타겟 발행 (sim_target:=true일 때만 사용) -----
        DeclareLaunchArgument(
            "target_x",
            default_value="0.0313",
            description="sim_target:=true일 때 fake_target_publisher가 쏘는 목표 x(m).",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="0.2279",
            description="sim_target:=true일 때 fake_target_publisher가 쏘는 목표 y(m).",
        ),
        DeclareLaunchArgument(
            "target_z",
            default_value="0.423",
            description="sim_target:=true일 때 fake_target_publisher가 쏘는 목표 z(m).",
        ),
        DeclareLaunchArgument(
            "sim_target_delay",
            default_value="15.0",
            description=(
                "sim_target:=true일 때, 파이프라인이 다 뜰 시간을 준 뒤 목표를 쏘기까지의 지연(초). "
                "control_bringup.launch.py의 /compute_ik가 뜨는 데 이 환경에서 ~13초 걸리는 걸 "
                "실측해서 여유를 두고 15초로 설정 - 너무 짧으면 fake_target_publisher가 IK 서비스가 "
                "뜨기 전에 쏴서 'IK service not ready'로 무시된다."
            ),
        ),
    ]

    sim_target = LaunchConfiguration("sim_target")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    model_path = LaunchConfiguration("model_path")
    target_class = LaunchConfiguration("target_class")
    conf_threshold = LaunchConfiguration("conf_threshold")
    infer_size = LaunchConfiguration("infer_size")
    depth_roi_radius = LaunchConfiguration("depth_roi_radius")
    max_depth_m = LaunchConfiguration("max_depth_m")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    sim_target_delay = LaunchConfiguration("sim_target_delay")

    summer_supply_node = Node(
        package="dolbotz",
        executable="summer_supply",
        name="arm_pickup_node",
        output="screen",
        parameters=[
            {
                "color_topic": color_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "model_path": model_path,
                "target_class": target_class,
                "conf_threshold": conf_threshold,
                "infer_size": infer_size,
                "depth_roi_radius": depth_roi_radius,
                "max_depth_m": max_depth_m,
            }
        ],
        condition=UnlessCondition(sim_target),
    )

    fake_target_publisher_node = Node(
        package="army_manipulator_bringup",
        executable="fake_target_publisher.py",
        output="screen",
        parameters=[
            {
                "x": target_x,
                "y": target_y,
                "z": target_z,
                "delay_sec": sim_target_delay,
            }
        ],
        condition=IfCondition(sim_target),
    )

    return LaunchDescription(
        declared_arguments
        + [
            summer_supply_node,
            fake_target_publisher_node,
        ]
    )
