"""뎁스카메라 좌표 -> MoveIt IK 역산 -> 팔 경로 실행까지의 전체 파이프라인 bringup.

기동 순서:
  mock_bringup.launch.py (robot_state_publisher -> ros2_control_node ->
  joint_state_broadcaster -> arm_controller/gripper_controller -> move_group ->
  RViz -> maru_ik_node)
  + (sim_target:=false, 기본) realsense_bringup.launch.py (RealSense 드라이버
    + cam_link<->camera_link TF) + target_detector_node (compressed color/depth
    -> 3D 타겟 좌표 -> /arm/target_point)
  + (sim_target:=true) fake_target_publisher (실물 카메라 없이 파라미터로 받은
    고정 좌표를 지연 후 /arm/target_point로 publish - 카메라/검출 노드 없이도
    maru_ik_node 이후 다운스트림 전체를 시뮬레이션으로 검증할 때 사용)

즉 이 launch 자체는 새 로직이 없고, 이미 각각 검증된 조각들
(팔+MoveIt, 카메라 드라이버+타겟 검출 또는 그 자리의 가짜 타겟 발행)을
하나로 묶기만 한다:
  카메라 픽셀+깊이 -> target_detector_node가 3D 점으로 변환(또는 sim_target
  모드에서 fake_target_publisher가 고정 좌표로 대체)해 /arm/target_point
  (PointStamped)로 publish
  -> maru_ik_node가 그 점을 구독해 MoveIt GetPositionIK로 관절해를 구하고
  arm_controller/gripper_controller로 전송
  -> planned_encoder_trajectory가 MoveIt 경로를 raw encoder-radian 궤적으로
  /arm/planned_encoder_trajectory에 관측용 발행

detection: target_detector_node는 dolbotZ(9o9hz/dolbotZ)의 arm_pickup_node와
동일한 패턴 — 사전 학습된 YOLO(ultralytics) 가중치로 color 이미지에서
target_class 클래스(기본 "supplybox")를 검출해 confidence가 가장 높은 박스
중심 주변 depth ROI 중앙값을 깊이로 쓴다. 가중치 파일이
config/models/supplybest_openvino_model/(또는 model_path로 지정한 경로)에
없으면 검출이 항상 실패해 팔이 움직이지 않으니, 실물 카메라 모드
(sim_target:=false) 실행 전에 가중치가 준비됐는지 먼저 확인할 것. 가중치 없이
다운스트림(IK/실행) 파이프라인만 검증하려면 sim_target:=true를 사용.

주의(compressed transport): color/depth 기본 토픽이 CompressedImage
(`.../compressed`, `.../compressedDepth`)라서, realsense2_camera가 이 토픽을
내보내려면 `ros-humble-compressed-image-transport` +
`ros-humble-compressed-depth-image-transport`가 설치되어 있어야 한다
(image_transport 플러그인이라 안 깔려 있으면 base raw 토픽만 존재하고
compressed 파생 토픽은 아예 발행되지 않는다).

real hardware:
  use_mock_hardware:=false 로 전환 시 army_manipulator_ros2_control.xacro의
  usb_port/ifname/actuator_id를 실제 배선에 맞게 먼저 수정할 것
  (mock_bringup.launch.py 상단 주석 참고).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="mock_bringup.launch.py로 그대로 전달. false면 실제 하드웨어 플러그인 사용.",
        ),
        DeclareLaunchArgument(
            "use_mesh",
            default_value="true",
            description="mock_bringup.launch.py로 그대로 전달.",
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/camera/camera/color/image_raw/compressed",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description=(
                "target_detector_node로 그대로 전달(sim_target:=false일 때만 사용). "
                "빈 문자열이면 노드 기본값"
                "(<share>/army_manipulator_bringup/config/models/supplybest_openvino_model) 사용."
            ),
        ),
        DeclareLaunchArgument(
            "target_class",
            default_value="supplybox",
            description="target_detector_node로 그대로 전달. YOLO 검출 결과 중 이 클래스명만 타겟으로 사용.",
        ),
        DeclareLaunchArgument(
            "conf_threshold",
            default_value="0.5",
            description="target_detector_node로 그대로 전달. YOLO confidence 임계값.",
        ),
        DeclareLaunchArgument(
            "infer_size",
            default_value="320",
            description="target_detector_node로 그대로 전달. YOLO 추론 해상도(GPU 없는 환경 속도용).",
        ),
        DeclareLaunchArgument(
            "depth_roi_radius",
            default_value="5",
            description="target_detector_node로 그대로 전달. 타겟 중심 픽셀 주변 깊이 샘플링 반경(px).",
        ),
        DeclareLaunchArgument(
            "max_depth_m",
            default_value="0.8",
            description="target_detector_node로 그대로 전달. 팔 집기 반경 내 유효 깊이 상한(m).",
        ),
        DeclareLaunchArgument(
            "sim_target",
            default_value="false",
            description=(
                "true면 realsense2_camera 드라이버/target_detector_node 대신 "
                "fake_target_publisher가 target_x/y/z 좌표를 /arm/target_point로 "
                "publish한다. 실물 카메라 없이 IK/실행 파이프라인만 시뮬레이션으로 "
                "검증할 때 사용."
            ),
        ),
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
                "move_group의 /compute_ik가 뜨는 데 이 환경에서 ~13초 걸리는 걸 실측해서 "
                "여유를 두고 15초로 설정 - 너무 짧으면 fake_target_publisher가 IK 서비스가 "
                "뜨기 전에 쏴서 'IK service not ready'로 무시된다."
            ),
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    model_path = LaunchConfiguration("model_path")
    target_class = LaunchConfiguration("target_class")
    conf_threshold = LaunchConfiguration("conf_threshold")
    infer_size = LaunchConfiguration("infer_size")
    depth_roi_radius = LaunchConfiguration("depth_roi_radius")
    max_depth_m = LaunchConfiguration("max_depth_m")
    sim_target = LaunchConfiguration("sim_target")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    sim_target_delay = LaunchConfiguration("sim_target_delay")

    arm_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "mock_bringup.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "use_mesh": use_mesh,
        }.items(),
    )

    depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "realsense_bringup.launch.py"]
            )
        ),
        condition=UnlessCondition(sim_target),
    )

    target_detector_node = Node(
        package="army_manipulator_bringup",
        executable="target_detector_node.py",
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

    planned_encoder_trajectory_node = Node(
        package="army_manipulator_bringup",
        executable="planned_encoder_trajectory.py",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            arm_and_moveit,
            depth_camera,
            target_detector_node,
            fake_target_publisher_node,
            planned_encoder_trajectory_node,
        ]
    )
