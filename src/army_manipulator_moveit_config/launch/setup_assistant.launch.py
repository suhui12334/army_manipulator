"""MoveIt Setup Assistant GUI 실행.

moveit_setup_assistant 는 실행 시 패키지 경로를 launch argument 로 받지 않는다
(업스트림 setup_assistant.launch.py 에는 `debug` 인자만 존재).
GUI가 뜨면 "Edit Existing MoveIt Configuration Package" 를 선택하고
army_manipulator_moveit_config 패키지 경로(이 패키지의 루트, package.xml 이 있는 곳)를
직접 지정할 것. 이 패키지에는 이미 .setup_assistant, config/army_manipulator.srdf,
config/kinematics.yaml 등이 준비되어 있어 그대로 이어서 편집할 수 있다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="moveit_setup_assistant 노드를 gdb 로 디버그 실행할지 여부",
        ),
    ]

    setup_assistant_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("moveit_setup_assistant"),
                    "launch",
                    "setup_assistant.launch.py",
                ]
            )
        ),
        launch_arguments={"debug": LaunchConfiguration("debug")}.items(),
    )

    return LaunchDescription(declared_arguments + [setup_assistant_launch])
