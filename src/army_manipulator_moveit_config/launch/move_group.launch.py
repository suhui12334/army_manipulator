"""move_group 노드 실행.

MoveItConfigsBuilder 의 robot_description 기본 경로 추정은
<package_name>/config/<robot_name>.urdf.xacro 인데, 실제 URDF는
army_manipulator_description 패키지 쪽에 있어서 명시적으로 지정한다.
나머지(config/army_manipulator.srdf, config/kinematics.yaml,
config/joint_limits.yaml, config/moveit_controllers.yaml,
config/ompl_planning.yaml)는 army_manipulator_moveit_config/config 의
기본 파일명과 일치하므로 자동으로 로드된다.

planning_pipelines를 ompl만으로 제한한 이유: config/pilz_cartesian_limits.yaml이
Setup Assistant 재실행 등으로 없어지면 MoveItConfigsBuilder가 pilz 파이프라인
설정을 못 찾아 launch 자체가 예외로 죽는다. ompl만 쓰면 이 의존성이 없다.

use_mesh: STL 실형상 사용 여부. 환경변수 ARMY_MANIPULATOR_USE_MESH=true 로
켤 수 있다(MoveItConfigsBuilder 경로는 DeclareLaunchArgument 조합을 그대로
받지 않으므로 launch 인자 대신 환경변수를 사용). collision mesh는 현재
visual과 동일한 원본 STL을 그대로 쓰고 있어 충돌 검사가 무거울 수 있음 —
추후 convex decomposition으로 단순화 권장(TODO).
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
    return generate_move_group_launch(moveit_config)
