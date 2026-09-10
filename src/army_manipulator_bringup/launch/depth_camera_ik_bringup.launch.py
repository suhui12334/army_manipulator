"""뎁스카메라 좌표 -> MoveIt IK 역산 -> 팔 경로 실행까지의 전체 파이프라인 bringup.

[2026-08-30 분리] 원래 이 파일 하나에 다 있던 걸 터미널/역할별로 4개로 쪼갰다:
  - realsense_bringup.launch.py  : 카메라 드라이버만(RealSense + cam_link TF)
  - perception_bringup.launch.py : 서플라이박스 인식(dolbotz의 summer_supply) +
                                    좌표추출 (-> /arm/target_point, 카메라는
                                    포함 안 함 - 위 걸 따로 띄워야 함)
  - control_bringup.launch.py    : TF 변환 -> MoveIt/IK -> 실제 명령 실행 +
                                    엔코더/CAN 송신값 관측용 병렬 출력
  - ui_bringup.launch.py         : 로컬 RViz + 구동부 연동 참고(실제 릴레이 노드는
                                    없음 - control_bringup이 관련 토픽을 직접 publish)
카메라와 인식이 별도 launch로 나뉜 이유: 실기 디버깅 중 카메라(시리얼/
네임스페이스 문제 등)만 따로 켜고 끄며 확인해야 할 일이 많아서, 인식
로직과 분리해두는 게 실전에서 훨씬 편했다.
이 파일은 그 넷 중 카메라 + perception + control 세 개를 한 번에 띄우는
하위호환용 래퍼일 뿐, 새 로직은 없다. 빠른 시뮬레이션 검증
(sim_target:=true)이나 "전부 한 프로세스 트리로" 띄우고 싶을 때 이거
하나만 실행하면 되고, 터미널을 역할별로 나누고 싶으면 위 네 launch를
따로따로 실행하면 된다.

기동 순서:
  mock_bringup.launch.py (robot_state_publisher -> ros2_control_node ->
  joint_state_broadcaster -> arm_controller/gripper_controller -> move_group ->
  RViz -> maru_ik_node)
  + (sim_target:=false, 기본) realsense_bringup.launch.py (RealSense 드라이버
    + cam_link<->camera_link TF) + summer_supply/ArmPickupNode(dolbotz 패키지,
    compressed color/depth -> 3D 타겟 좌표 -> /arm/target_point)
  + (sim_target:=true) fake_target_publisher (실물 카메라 없이 파라미터로 받은
    고정 좌표를 지연 후 /arm/target_point로 publish - 카메라/검출 노드 없이도
    maru_ik_node 이후 다운스트림 전체를 시뮬레이션으로 검증할 때 사용)

detection: summer_supply(ArmPickupNode, dolbotz/missions/summer_supply.py)는
YOLO(ultralytics, supplyboxv3.pt - 2026-08-30 RF-DETR와 비교 후 이걸로
확정)로 color 이미지에서 서플라이박스를 검출하고, confidence가 가장 높은
박스 중심 주변 depth ROI 중앙값을 깊이로 쓴다. 가중치 파일(*.pt)은 git으로
동기화 안 되니(.gitignore) 실물 카메라 모드(sim_target:=false) 실행 전에
config/models/에 준비됐는지 먼저 확인할 것. 가중치 없이 다운스트림(IK/실행)
파이프라인만 검증하려면 sim_target:=true를 사용.

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
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        # ----- 공통 (control_bringup.launch.py로 그대로 전달) -----
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("use_mesh", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="true",
            description="control_bringup.launch.py로 그대로 전달. AUTO 자동 활성화 여부.",
        ),
        # ----- perception_bringup.launch.py로 그대로 전달 -----
        DeclareLaunchArgument("sim_target", default_value="false"),
        # ----- realsense_bringup.launch.py로 그대로 전달 -----
        DeclareLaunchArgument("serial_no", default_value="_243322074693"),
        DeclareLaunchArgument("camera_namespace", default_value="arm"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x15"),
        DeclareLaunchArgument("color_topic", default_value="/arm/camera/color/image_raw/compressed"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/arm/camera/aligned_depth_to_color/image_raw/compressedDepth",
        ),
        DeclareLaunchArgument("camera_info_topic", default_value="/arm/camera/color/camera_info"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("target_class", default_value="supplybox"),
        DeclareLaunchArgument("conf_threshold", default_value="0.5"),
        DeclareLaunchArgument("infer_size", default_value="320"),
        DeclareLaunchArgument("depth_roi_radius", default_value="5"),
        DeclareLaunchArgument("max_depth_m", default_value="1.0"),
        DeclareLaunchArgument("target_x", default_value="0.0313"),
        DeclareLaunchArgument("target_y", default_value="0.2279"),
        DeclareLaunchArgument("target_z", default_value="0.423"),
        DeclareLaunchArgument("sim_target_delay", default_value="15.0"),
    ]

    control_keys = ("use_mock_hardware", "use_mesh", "launch_rviz", "auto_enable")
    camera_keys = (
        "serial_no", "camera_namespace", "camera_name", "color_profile", "depth_profile",
    )
    perception_keys = (
        "sim_target", "color_topic", "depth_topic", "camera_info_topic",
        "model_path", "target_class", "conf_threshold", "infer_size",
        "depth_roi_radius", "max_depth_m",
        "target_x", "target_y", "target_z", "sim_target_delay",
    )
    passthrough = {
        name: LaunchConfiguration(name) for name in control_keys + camera_keys + perception_keys
    }
    sim_target = passthrough["sim_target"]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "control_bringup.launch.py"]
            )
        ),
        launch_arguments={k: passthrough[k] for k in control_keys}.items(),
    )

    # sim_target:=true면 카메라 자체를 안 띄운다(fake_target_publisher가 대신함).
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "realsense_bringup.launch.py"]
            )
        ),
        launch_arguments={k: passthrough[k] for k in camera_keys}.items(),
        condition=UnlessCondition(sim_target),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "perception_bringup.launch.py"]
            )
        ),
        launch_arguments={k: passthrough[k] for k in perception_keys}.items(),
    )

    # AUTO 게이트(맴/mission 안전 확인 포함)는 control_bringup.launch.py 안에
    # 이미 구현돼 있고, auto_enable 인자가 그대로 전달되므로 여기서 따로
    # 다룰 게 없다.

    return LaunchDescription(
        declared_arguments
        + [
            control,
            camera,
            perception,
        ]
    )
