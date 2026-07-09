"""move_group 노드 실행.

MoveItConfigsBuilder 가 army_manipulator_moveit_config/.setup_assistant 를 읽어
army_manipulator_description 패키지의 URDF(xacro)와 config/army_manipulator.srdf,
config/kinematics.yaml, config/joint_limits.yaml, config/moveit_controllers.yaml,
config/ompl_planning.yaml 을 자동으로 로드한다.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "army_manipulator", package_name="army_manipulator_moveit_config"
    ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)
