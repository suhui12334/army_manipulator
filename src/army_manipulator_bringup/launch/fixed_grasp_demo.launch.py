"""Standalone fixed-pose hardware demo; no perception, MoveIt, or IK nodes.

Starts only robot_state_publisher, ros2_control, the three controllers, and one
``autonomous_grasp_demo`` process.  This keeps the real autonomous bringup and
the encoder-pose replay in separate launch graphs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="false면 실제 Dynamixel TTL + RMD CAN 하드웨어를 사용."),
        DeclareLaunchArgument("use_mesh", default_value="true"),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("grasp_wait_preset", default_value="legacy"),
        DeclareLaunchArgument("sid_base_rad", default_value="-0.02761"),
        DeclareLaunchArgument("arm_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("gripper_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("between_steps_sec", default_value="0.3"),
        DeclareLaunchArgument(
            "gripper_current_threshold_ma", default_value="100.0"),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("army_manipulator_description"),
            "urdf", "army_manipulator.urdf.xacro",
        ]),
        " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
        " use_mesh:=", LaunchConfiguration("use_mesh"),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controller_config = PathJoinSubstitution([
        FindPackageShare("army_manipulator_bringup"),
        "config", "ros2_controllers.yaml",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controller_config],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_motion_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_spawner, gripper_spawner],
        )
    )

    demo = Node(
        package="army_manipulator_bringup",
        executable="autonomous_grasp_demo.py",
        name="autonomous_grasp_demo",
        output="screen",
        parameters=[{
            "grasp_wait_preset": ParameterValue(
                LaunchConfiguration("grasp_wait_preset"), value_type=str),
            "sid_base_rad": ParameterValue(
                LaunchConfiguration("sid_base_rad"), value_type=float),
            "arm_duration_sec": ParameterValue(
                LaunchConfiguration("arm_duration_sec"), value_type=float),
            "gripper_duration_sec": ParameterValue(
                LaunchConfiguration("gripper_duration_sec"), value_type=float),
            "between_steps_sec": ParameterValue(
                LaunchConfiguration("between_steps_sec"), value_type=float),
            "gripper_current_threshold_ma": ParameterValue(
                LaunchConfiguration("gripper_current_threshold_ma"), value_type=float),
        }],
    )
    delayed_demo = TimerAction(
        period=LaunchConfiguration("start_delay_sec"), actions=[demo])

    return LaunchDescription(arguments + [
        robot_state_publisher,
        controller_manager,
        joint_state_spawner,
        spawn_motion_controllers,
        delayed_demo,
    ])
