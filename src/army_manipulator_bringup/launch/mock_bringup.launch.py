"""mock_hardware 기반 RViz + MoveIt2 통합 확인용 bringup.

기동 순서:
  robot_state_publisher -> ros2_control_node(mock_components/GenericSystem)
  -> joint_state_broadcaster -> arm_controller/gripper_controller
  -> move_group -> RViz(MotionPlanning plugin)

팀원 Hardware Interface 가 완성되면 use_mock_hardware:=false 로 실행하고,
army_manipulator_description/urdf/army_manipulator_ros2_control.xacro 의
usb_port/ifname/actuator_id 를 실제 배선에 맞게 수정할 것.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="mock_components/GenericSystem 사용 여부. false면 실제 하드웨어 플러그인 사용.",
        ),
        DeclareLaunchArgument(
            "use_mesh",
            default_value="false",
            description="true: meshes/{visual,collision}/*.stl 실형상 사용. false: primitive geometry.",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")

    # move_group.launch.py / moveit_rviz.launch.py는 MoveItConfigsBuilder 경로 특성상
    # DeclareLaunchArgument를 직접 받지 않고 ARMY_MANIPULATOR_USE_MESH 환경변수를 읽는다
    # (해당 launch 파일 상단 주석 참고). 여기서 동일한 use_mesh 값을 환경변수로 전파한다.
    set_use_mesh_env = SetEnvironmentVariable("ARMY_MANIPULATOR_USE_MESH", use_mesh)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_description"), "urdf", "army_manipulator.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "use_mesh:=",
            use_mesh,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    ros2_controllers_config = PathJoinSubstitution(
        [FindPackageShare("army_manipulator_bringup"), "config", "ros2_controllers.yaml"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ros2_controllers_config],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # joint_state_broadcaster 스폰이 끝난 뒤 나머지 컨트롤러를 스폰 (controller_manager 준비 대기)
    delay_controllers_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        )
    )

    # 카메라 타겟 -> IK -> arm_controller/gripper_controller 로 이어지는 노드.
    # /joint_command_mux 에는 아직 구독자가 없으므로(dxl_ee mux 미통합),
    # 기본적으로 direct_control=true 로 FollowJointTrajectory를 직접 보낸다.
    maru_ik_node = Node(
        package="army_manipulator_bringup",
        executable="maru_ik_node.py",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            set_use_mesh_env,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delay_controllers_after_joint_state_broadcaster,
            move_group_launch,
            moveit_rviz_launch,
            maru_ik_node,
        ]
    )
