"""UI(로컬 RViz) + 구동부 연동 참고 bringup.

depth_camera_ik_bringup.launch.py에서 분리한 조각 중 "UI랑 로컬 RViz에 송신,
주행 명령/파지성공여부를 구동부 쪽에 송신"하는 부분을 담당한다. 다만 실제로
새로 띄워야 하는 프로세스는 RViz 하나뿐이다:

  - RViz: 이 파일은 army_manipulator_moveit_config/launch/moveit_rviz.launch.py를
    그대로 include한다. JECS 등 실기(SSH만 접속 가능)에서는 control_bringup의
    launch_rviz:=false로 그쪽 RViz를 끄고, 대신 이 launch를 "같은 저장소가 있는
    로컬 머신"에서 띄워서 네트워크로 move_group/TF에 붙는다(같은 ROS_DOMAIN_ID
    필요). ARMY_MANIPULATOR_USE_MESH 환경변수로 mesh/primitive를 control_bringup
    쪽과 맞춰줄 것(둘 다 켜거나 둘 다 꺼서 실행 - moveit_rviz.launch.py 상단
    주석 참고).

  - 구동부 송신: /arm/forward_command(가동범위 밖 타겟 -> "전진해도 됨"),
    /arm/picking_command(파지 시퀀스 완료 -> "주행 재개"), /arm/grasp_success
    (파지 성공 여부)는 control_bringup.launch.py의 maru_ik_node가 이미 직접
    publish한다. 즉 이 셋을 구동부 쪽으로 "보내기 위해" 여기서 새로 띄워야 하는
    릴레이/브리지 노드는 없다 - 구동부 쪽 주행 제어 노드가 같은 ROS 네트워크에서
    이 토픽들을 구독하기만 하면 된다(army_manipulator_bringup 밖의 별도 패키지).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
    )

    return LaunchDescription([rviz])
