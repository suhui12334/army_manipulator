"""MoveIt MotionPlanning 플러그인이 포함된 RViz 실행.

robot_description 경로 지정 이유는 move_group.launch.py 주석 참고.
"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    urdf_xacro_path = os.path.join(
        get_package_share_directory("army_manipulator_description"),
        "urdf",
        "army_manipulator.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("army_manipulator", package_name="army_manipulator_moveit_config")
        .robot_description(file_path=urdf_xacro_path, mappings={"use_mock_hardware": "true"})
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
