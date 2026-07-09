"""move_group 노드 실행.

MoveItConfigsBuilder 의 robot_description 기본 경로 추정은
<package_name>/config/<robot_name>.urdf.xacro 인데, 실제 URDF는
army_manipulator_description 패키지 쪽에 있어서 명시적으로 지정한다.
나머지(config/army_manipulator.srdf, config/kinematics.yaml,
config/joint_limits.yaml, config/moveit_controllers.yaml,
config/ompl_planning.yaml)는 army_manipulator_moveit_config/config 의
기본 파일명과 일치하므로 자동으로 로드된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


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
    return generate_move_group_launch(moveit_config)
