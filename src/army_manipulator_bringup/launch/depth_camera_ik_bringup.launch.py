"""뎁스카메라 좌표 -> MoveIt IK 역산 -> 팔 경로 실행까지의 전체 파이프라인 bringup.

기동 순서:
  mock_bringup.launch.py (robot_state_publisher -> ros2_control_node ->
  joint_state_broadcaster -> arm_controller/gripper_controller -> move_group ->
  RViz -> maru_ik_node)
  + (sim_target:=false, 기본) realsense_bringup.launch.py (RealSense 드라이버
    + cam_link<->camera_link TF) + target_detector_node (color/depth -> 3D
    타겟 좌표 -> /maru/target/point)
  + (sim_target:=true) fake_target_publisher (실물 카메라 없이 파라미터로 받은
    고정 좌표를 지연 후 /maru/target/point로 publish - 카메라/검출 노드 없이도
    maru_ik_node 이후 다운스트림 전체를 시뮬레이션으로 검증할 때 사용)

즉 이 launch 자체는 새 로직이 없고, 이미 각각 검증된 조각들
(팔+MoveIt, 카메라 드라이버+타겟 검출 또는 그 자리의 가짜 타겟 발행)을
하나로 묶기만 한다:
  카메라 픽셀+깊이 -> target_detector_node가 3D 점으로 변환(또는 sim_target
  모드에서 fake_target_publisher가 고정 좌표로 대체)해 /maru/target/point
  (PointStamped)로 publish
  -> maru_ik_node가 그 점을 구독해 MoveIt GetPositionIK로 관절해를 구하고
  arm_controller/gripper_controller로 전송

TODO(detection): target_detector_node.detect_target_pixel()이 아직 항상 None을
반환하는 자리표시자라서, 실제 타겟 검출 알고리즘이 붙기 전까지는 실물 카메라
모드(sim_target:=false)에서는 팔이 움직이지 않는다. 검출 로직 구현 전까지는
sim_target:=true로 다운스트림을 검증할 것.

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
            default_value="/camera/color/image_raw",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/aligned_depth_to_color/image_raw",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
            description="target_detector_node로 그대로 전달(sim_target:=false일 때만 사용).",
        ),
        DeclareLaunchArgument(
            "sim_target",
            default_value="false",
            description=(
                "true면 realsense2_camera 드라이버/target_detector_node 대신 "
                "fake_target_publisher가 target_x/y/z 좌표를 /maru/target/point로 "
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
            default_value="5.0",
            description="sim_target:=true일 때, 파이프라인이 다 뜰 시간을 준 뒤 목표를 쏘기까지의 지연(초).",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
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
            arm_and_moveit,
            depth_camera,
            target_detector_node,
            fake_target_publisher_node,
        ]
    )
