"""MoveIt MotionPlanning 플러그인이 포함된 RViz 실행."""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "army_manipulator", package_name="army_manipulator_moveit_config"
    ).to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
