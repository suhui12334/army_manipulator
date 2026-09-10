"""URDF 단독 확인용: robot_state_publisher + joint_state_publisher_gui + RViz.

ros2_control / MoveIt2 없이 링크 형상, 조인트 리밋, TF 트리만 빠르게 검증할 때 사용한다.
"""
import os
import signal
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# 이 launch 파일을 반복 재실행할 때(특히 이전 인스턴스를 Ctrl+C로 안 끄고 새로
# 띄우는 경우) robot_state_publisher/rviz2/joint_state_publisher_gui가 중복으로
# 남아 같은 /joint_states, /tf를 두고 경합하면서 "좌표축이 왔다갔다" 하는 증상이
# 생긴다. 이전 인스턴스의 프로세스 그룹을 PID 파일로 추적해뒀다가, 새로 실행될
# 때마다 살아있으면 통째로 정리하고 시작한다 (이 launch 파일 전용 락이라 다른
# 로봇/패키지의 무관한 프로세스는 건드리지 않는다).
_LOCK_FILE = "/tmp/.army_manipulator_display_launch.pgid"


def _cleanup_previous_instance():
    if os.path.exists(_LOCK_FILE):
        try:
            with open(_LOCK_FILE, "r") as f:
                old_pgid = int(f.read().strip())
            os.killpg(old_pgid, signal.SIGKILL)
            time.sleep(1.0)
        except (OSError, ValueError):
            pass  # 이미 죽어있거나 pgid가 재사용된 경우는 무시
    try:
        with open(_LOCK_FILE, "w") as f:
            f.write(str(os.getpgid(0)))
    except OSError:
        pass


def generate_launch_description():
    _cleanup_previous_instance()

    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="ros2_control 하드웨어 인터페이스로 mock_components/GenericSystem 사용 여부",
        ),
        DeclareLaunchArgument(
            "use_mesh",
            default_value="true",
            description="true: meshes/{visual,collision}/*.stl 실형상 사용. false: primitive geometry.",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_description"), "urdf", "army_manipulator.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "use_mesh:=",
            use_mesh,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 모든 조인트의 기본 슬라이더 위치를 0으로 강제.
    # 단, base_joint/wrist_joint/gripper_joint는 joint_calibration.yaml의
    # zero_offset이 아직 0.0 placeholder라 리미트 범위 자체가 0을 포함하지 않아,
    # 실측 zero_offset이 채워지기 전까지는 슬라이더가 리미트 경계로 clamp된다.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "zeros": {
                "base_joint": 0.0,
                "shoulder_joint": 0.0,
                "elbow_joint": 0.0,
                "wrist_joint": 0.0,
                "gripper_joint": 0.0,
                "rack_left_joint": 0.0,
                "rack_right_joint": 0.0,
            }
        }],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("army_manipulator_description"), "rviz", "display.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
