"""Minimal real-arm autonomous grasp bringup, using ik_node.py instead of
maru_ik_node.py.

Required path only:
robot/ros2_control/controllers + move_group(/compute_ik) + ik_node.
The sequence is GRASP_WAIT -> detected target -> close-depth confirmation
-> current-triggered gripper close -> HOLD.

Separated from control_bringup.launch.py (which stays on maru_ik_node.py,
the full state-machine node with AUTO/manual_override gating, retry/approach
logic, and forward_command integration) so the two IK nodes can be launched
independently instead of one replacing the other in-place.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="false면 실제 Dynamixel TTL + RMD CAN 하드웨어 사용."),
        DeclareLaunchArgument("use_mesh", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("target_topic", default_value="/arm/target_point"),
        DeclareLaunchArgument("planning_frame", default_value="base_actuator"),
        DeclareLaunchArgument("arm_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("gripper_duration_sec", default_value="3.0"),
        DeclareLaunchArgument(
            "use_fixed_target_z", default_value="true",
            description="true면 검출 X/Y만 사용하고 평지/박스 치수로 TCP Z를 계산."),
        DeclareLaunchArgument(
            "base_height_m", default_value="0.350",
            description="world 지면에서 base_actuator 원점까지 높이[m]."),
        DeclareLaunchArgument(
            "box_height_m", default_value="0.095",
            description="supplybox 높이[m]."),
        DeclareLaunchArgument(
            "box_grasp_height_ratio", default_value="0.5",
            description="박스 바닥=0, 상단=1 기준 TCP 파지 높이 비율."),
        DeclareLaunchArgument(
            "detected_z_warning_threshold_m", default_value="0.05",
            description="검출 Z와 고정 Z 차이가 이 값보다 크면 TF 점검 경고."),
        DeclareLaunchArgument(
            "supplybox_tcp_offset_z", default_value="-0.0055",
            description=(
                "치수로 계산한 고정 Z에 더하는 실기 캘리브레이션 미세 보정[m]. "
                "경사 아래쪽 박스 파지를 위해 -3.5mm에서 2mm 더 낮춤. "
                "최종 TCP Z는 -0.3025 - 0.0055 = -0.3080m. "
                "수치 IK: 반경 160~315mm, Y=-30/0/+30mm에서 해 확인. "
                "이 높이의 MoveIt 충돌검사 및 실기 도달은 미확인.")),
        DeclareLaunchArgument("ik_request_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("ik_avoid_collisions", default_value="true"),
        DeclareLaunchArgument("settle_before_close_sec", default_value="2.0"),
        DeclareLaunchArgument("gripper_current_threshold_ma", default_value="125.0"),
    ]

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("army_manipulator_bringup"),
            "launch", "mock_bringup.launch.py",
        ])),
        launch_arguments={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "use_mesh": LaunchConfiguration("use_mesh"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_maru_ik": "false",
        }.items(),
    )

    ik_node = Node(
        package="army_manipulator_bringup",
        executable="ik_node.py",
        name="ik_node",
        output="screen",
        parameters=[{
            "target_topic": ParameterValue(
                LaunchConfiguration("target_topic"), value_type=str),
            "planning_frame": ParameterValue(
                LaunchConfiguration("planning_frame"), value_type=str),
            "arm_duration_sec": ParameterValue(
                LaunchConfiguration("arm_duration_sec"), value_type=float),
            "gripper_duration_sec": ParameterValue(
                LaunchConfiguration("gripper_duration_sec"), value_type=float),
            "use_fixed_target_z": ParameterValue(
                LaunchConfiguration("use_fixed_target_z"), value_type=bool),
            "base_height_m": ParameterValue(
                LaunchConfiguration("base_height_m"), value_type=float),
            "box_height_m": ParameterValue(
                LaunchConfiguration("box_height_m"), value_type=float),
            "box_grasp_height_ratio": ParameterValue(
                LaunchConfiguration("box_grasp_height_ratio"), value_type=float),
            "detected_z_warning_threshold_m": ParameterValue(
                LaunchConfiguration("detected_z_warning_threshold_m"), value_type=float),
            "supplybox_tcp_offset_z": ParameterValue(
                LaunchConfiguration("supplybox_tcp_offset_z"), value_type=float),
            "ik_request_timeout_sec": ParameterValue(
                LaunchConfiguration("ik_request_timeout_sec"), value_type=float),
            "ik_avoid_collisions": ParameterValue(
                LaunchConfiguration("ik_avoid_collisions"), value_type=bool),
            "settle_before_close_sec": ParameterValue(
                LaunchConfiguration("settle_before_close_sec"), value_type=float),
            "gripper_current_threshold_ma": ParameterValue(
                LaunchConfiguration("gripper_current_threshold_ma"), value_type=float),
        }],
    )

    # DolbotZ-Center는 이 토픽이 있으면 /joint_states의 단순 평면 FK 대신
    # robot_state_publisher가 계산한 실제 TF 좌표를 그린다. legacy
    # control_bringup.launch.py와 같은 UI 좌표 기준을 사용하게 반드시 같이
    # 실행한다. 이 노드가 없으면 웹 UI의 기본 shoulder 영점(0deg) 때문에
    # 실제 형상이 약 90도 돌아가 보인다.
    arm_pose_array_publisher = Node(
        package="army_manipulator_bringup",
        executable="arm_pose_array_publisher.py",
        name="arm_pose_array_publisher",
        output="screen",
        parameters=[{
            "base_frame": ParameterValue(
                LaunchConfiguration("planning_frame"), value_type=str),
            "publish_hz": 20.0,
        }],
    )

    return LaunchDescription(
        arguments + [base_bringup, ik_node, arm_pose_array_publisher]
    )
