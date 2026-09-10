"""수치 DLS IK 기반 MARU 실제 자동파지 bringup.

카메라/인식은 별도 realsense_bringup.launch.py와 summer_supply.py가 담당한다.
이 launch는 기존 control_bringup의 controller, move_group, maru_ik_node를 모두
실행하되 IK backend만 로컬 수치해(numerical)로 고정한다. 수치해가 반환한
관절 목표는 MoveIt move_to_configuration을 거치므로 planning scene 충돌검사,
trajectory 생성, ros2_control 하드웨어 실행 경로는 유지된다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    argument_defaults = {
        "use_mock_hardware": "true",
        "use_mesh": "true",
        "launch_rviz": "true",
        "auto_enable": "true",
        "grasp_start_x_distance_m": "0.21",
        "direct_grasp_max_camera_depth_m": "1.0",
        "startup_grasp_wait_delay_sec": "2.0",
        "startup_grasp_wait_duration_sec": "3.0",
        "grasp_wait_preset": "legacy",
        "forward_command_min_interval_sec": "2.0",
        "ik_max_seed_attempts": "6",
        "numerical_ik_max_iterations": "250",
        "numerical_ik_position_tolerance_m": "0.003",
        "numerical_ik_pitch_tolerance_rad": "0.025",
        "numerical_ik_damping": "0.025",
    }
    declared = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in argument_defaults.items()
    ]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("army_manipulator_bringup"),
                "launch",
                "control_bringup.launch.py",
            ])
        ),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in argument_defaults},
            "ik_solver_backend": "numerical",
        }.items(),
    )
    return LaunchDescription(declared + [control])
