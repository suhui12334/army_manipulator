"""MoveIt MotionPlanning 플러그인이 포함된 RViz 실행.

robot_description 경로 지정 및 planning_pipelines 제한 이유는
move_group.launch.py 주석 참고.
use_mesh는 move_group.launch.py와 동일하게 ARMY_MANIPULATOR_USE_MESH
환경변수로 제어한다(두 launch가 같은 robot_description을 봐야 TF/충돌
형상이 일치하므로, 둘 다 켜거나 둘 다 꺼서 실행할 것).
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
    use_mesh = os.environ.get("ARMY_MANIPULATOR_USE_MESH", "false")

    moveit_config = (
        MoveItConfigsBuilder("army_manipulator", package_name="army_manipulator_moveit_config")
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={"use_mock_hardware": "true", "use_mesh": use_mesh},
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
