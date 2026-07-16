"""URDF 단독 확인용: robot_state_publisher + joint_state_publisher_gui + RViz.

ros2_control / MoveIt2 없이 링크 형상, 조인트 리밋, TF 트리만 빠르게 검증할 때 사용한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="ros2_control 하드웨어 인터페이스로 mock_components/GenericSystem 사용 여부",
        ),
        DeclareLaunchArgument(
            "use_mesh",
            default_value="false",
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
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
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
