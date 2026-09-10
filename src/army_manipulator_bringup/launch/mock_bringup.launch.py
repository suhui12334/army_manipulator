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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            default_value="true",
            description="true: meshes/{visual,collision}/*.stl 실형상 사용. false: primitive geometry.",
        ),
        # [추가] 실기에서 "매 사이클 실제로 명령되는 이론값"과 "실제 엔코더로
        # 읽히는 값"을 RViz에서 반투명 고스트 로봇으로 겹쳐보기 위한 옵션.
        # joint_trajectory_controller가 매 사이클 이미 발행하는
        # controller_state.reference를 그대로 재활용한다(새로 계산 안 함).
        DeclareLaunchArgument(
            "show_theoretical_ghost",
            default_value="false",
            description="true면 이론상 목표값(controller reference)을 별도 TF prefix"
                        "(theoretical_)로 발행해서 RViz에 고스트로 겹쳐볼 수 있게 한다.",
        ),
        # [추가] 실기(jecs 등)는 SSH로만 접속해서 GUI가 안 뜨는 경우가 많다 -
        # 여기서 RViz는 끄고, moveit_rviz.launch.py를 같은 저장소가 있는 로컬
        # 머신에서 따로 띄워서 네트워크로 move_group/TF에 붙게 하면 된다
        # (같은 ROS_DOMAIN_ID/네트워크 필요).
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="false면 이 launch에서 RViz를 안 띄운다 - 원격(실기)에서 SSH로만 "
                        "접속할 때, RViz는 로컬 머신에서 moveit_rviz.launch.py로 따로 "
                        "띄우고 여기서는 끄기 위한 옵션.",
        ),
        DeclareLaunchArgument(
            "launch_maru_ik", default_value="true",
            description="false면 legacy maru_ik_node를 실행하지 않음."),
        DeclareLaunchArgument("ik_max_seed_attempts", default_value="6"),
        DeclareLaunchArgument("ik_request_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("ik_solver_backend", default_value="analytic"),
        DeclareLaunchArgument("numerical_ik_max_iterations", default_value="250"),
        DeclareLaunchArgument("numerical_ik_position_tolerance_m", default_value="0.003"),
        DeclareLaunchArgument("numerical_ik_pitch_tolerance_rad", default_value="0.025"),
        DeclareLaunchArgument("numerical_ik_damping", default_value="0.025"),
        DeclareLaunchArgument("ik_solution_cache_size", default_value="16"),
        DeclareLaunchArgument("ik_cached_seed_count", default_value="3"),
        DeclareLaunchArgument("ik_avoid_collisions", default_value="true"),
        DeclareLaunchArgument("workspace_max_tcp_distance_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_start_x_distance_m", default_value="0.21"),
        DeclareLaunchArgument("direct_grasp_max_camera_depth_m", default_value="1.0"),
        DeclareLaunchArgument("forward_command_min_interval_sec", default_value="2.0"),
        DeclareLaunchArgument("startup_grasp_wait_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("startup_grasp_wait_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("startup_grasp_wait_external", default_value="false"),
        DeclareLaunchArgument(
            "startup_pose_complete_topic", default_value="/arm/startup_pose_complete"),
        DeclareLaunchArgument("grasp_wait_preset", default_value="legacy"),
        DeclareLaunchArgument("precision_failures_before_approach", default_value="2"),
        DeclareLaunchArgument("approach_retry_wait_sec", default_value="1.0"),
        DeclareLaunchArgument("picking_state_topic", default_value="/picking"),
        DeclareLaunchArgument(
            "calculation_failure_topic", default_value="/arm/calculation_failed"),
        DeclareLaunchArgument("target_point_base_topic", default_value="/arm/target_point_base"),
        DeclareLaunchArgument("target_distance_topic", default_value="/arm/target_distance_m"),
        DeclareLaunchArgument("manual_override_topic", default_value="/control/arm_manual_override"),
        DeclareLaunchArgument("manual_override_timeout_sec", default_value="1.5"),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")
    show_theoretical_ghost = LaunchConfiguration("show_theoretical_ghost")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_maru_ik = LaunchConfiguration("launch_maru_ik")
    ik_max_seed_attempts = LaunchConfiguration("ik_max_seed_attempts")
    ik_request_timeout_sec = LaunchConfiguration("ik_request_timeout_sec")
    ik_solver_backend = LaunchConfiguration("ik_solver_backend")
    numerical_ik_max_iterations = LaunchConfiguration("numerical_ik_max_iterations")
    numerical_ik_position_tolerance_m = LaunchConfiguration(
        "numerical_ik_position_tolerance_m")
    numerical_ik_pitch_tolerance_rad = LaunchConfiguration(
        "numerical_ik_pitch_tolerance_rad")
    numerical_ik_damping = LaunchConfiguration("numerical_ik_damping")
    ik_solution_cache_size = LaunchConfiguration("ik_solution_cache_size")
    ik_cached_seed_count = LaunchConfiguration("ik_cached_seed_count")
    ik_avoid_collisions = LaunchConfiguration("ik_avoid_collisions")
    workspace_max_tcp_distance_m = LaunchConfiguration("workspace_max_tcp_distance_m")
    grasp_start_x_distance_m = LaunchConfiguration("grasp_start_x_distance_m")
    direct_grasp_max_camera_depth_m = LaunchConfiguration(
        "direct_grasp_max_camera_depth_m")
    forward_command_min_interval_sec = LaunchConfiguration("forward_command_min_interval_sec")
    startup_grasp_wait_delay_sec = LaunchConfiguration("startup_grasp_wait_delay_sec")
    startup_grasp_wait_duration_sec = LaunchConfiguration(
        "startup_grasp_wait_duration_sec")
    startup_grasp_wait_external = LaunchConfiguration("startup_grasp_wait_external")
    startup_pose_complete_topic = LaunchConfiguration("startup_pose_complete_topic")
    grasp_wait_preset = LaunchConfiguration("grasp_wait_preset")
    precision_failures_before_approach = LaunchConfiguration("precision_failures_before_approach")
    approach_retry_wait_sec = LaunchConfiguration("approach_retry_wait_sec")
    picking_state_topic = LaunchConfiguration("picking_state_topic")
    calculation_failure_topic = LaunchConfiguration("calculation_failure_topic")
    target_point_base_topic = LaunchConfiguration("target_point_base_topic")
    target_distance_topic = LaunchConfiguration("target_distance_topic")
    manual_override_topic = LaunchConfiguration("manual_override_topic")
    manual_override_timeout_sec = LaunchConfiguration("manual_override_timeout_sec")

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
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

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
        ),
        condition=IfCondition(launch_rviz),
    )

    # 카메라 타겟 -> TF 변환 -> MoveIt move_group -> arm/gripper_controller로 이어지는 노드.
    # move_group이 만든 trajectory는 ros2_control 하드웨어 인터페이스를 통해
    # RMD(CAN)와 Dynamixel에 전달된다.
    maru_ik_node = Node(
        package="army_manipulator_bringup",
        executable="maru_ik_node.py",
        output="screen",
        parameters=[{
            "ik_max_seed_attempts": ParameterValue(ik_max_seed_attempts, value_type=int),
            "ik_request_timeout_sec": ParameterValue(ik_request_timeout_sec, value_type=float),
            "ik_solver_backend": ParameterValue(ik_solver_backend, value_type=str),
            "numerical_ik_max_iterations": ParameterValue(
                numerical_ik_max_iterations, value_type=int),
            "numerical_ik_position_tolerance_m": ParameterValue(
                numerical_ik_position_tolerance_m, value_type=float),
            "numerical_ik_pitch_tolerance_rad": ParameterValue(
                numerical_ik_pitch_tolerance_rad, value_type=float),
            "numerical_ik_damping": ParameterValue(
                numerical_ik_damping, value_type=float),
            "ik_solution_cache_size": ParameterValue(ik_solution_cache_size, value_type=int),
            "ik_cached_seed_count": ParameterValue(ik_cached_seed_count, value_type=int),
            "ik_avoid_collisions": ParameterValue(ik_avoid_collisions, value_type=bool),
            "workspace_max_tcp_distance_m": ParameterValue(
                workspace_max_tcp_distance_m, value_type=float),
            "grasp_start_x_distance_m": ParameterValue(
                grasp_start_x_distance_m, value_type=float),
            "direct_grasp_max_camera_depth_m": ParameterValue(
                direct_grasp_max_camera_depth_m, value_type=float),
            "forward_command_min_interval_sec": ParameterValue(
                forward_command_min_interval_sec, value_type=float),
            "startup_grasp_wait_delay_sec": ParameterValue(
                startup_grasp_wait_delay_sec, value_type=float),
            "startup_grasp_wait_duration_sec": ParameterValue(
                startup_grasp_wait_duration_sec, value_type=float),
            "startup_grasp_wait_external": ParameterValue(
                startup_grasp_wait_external, value_type=bool),
            "startup_pose_complete_topic": ParameterValue(
                startup_pose_complete_topic, value_type=str),
            "grasp_wait_preset": ParameterValue(grasp_wait_preset, value_type=str),
            "precision_failures_before_approach": ParameterValue(
                precision_failures_before_approach, value_type=int),
            "approach_retry_wait_sec": ParameterValue(approach_retry_wait_sec, value_type=float),
            "picking_state_topic": ParameterValue(picking_state_topic, value_type=str),
            "calculation_failure_topic": ParameterValue(
                calculation_failure_topic, value_type=str),
            "target_point_base_topic": ParameterValue(
                target_point_base_topic, value_type=str),
            "target_distance_topic": ParameterValue(target_distance_topic, value_type=str),
            "manual_override_topic": ParameterValue(manual_override_topic, value_type=str),
            "manual_override_timeout_sec": ParameterValue(
                manual_override_timeout_sec, value_type=float),
        }],
        condition=IfCondition(launch_maru_ik),
    )

    # [수정] 고스트 로봇 오버레이 - controller_state.reference를 중계 -> 별도
    # robot_state_publisher(frame_prefix=theoretical_)로 TF 생성 -> world와
    # theoretical_world를 identity로 이어서 같은 공간에 겹쳐 보이게 한다.
    # theoretical_state_relay.py 자체는 army_manipulator_bringup 패키지
    # 의존성이 없는 완전 독립 스크립트로 바뀌어서(path_visualizer.py와 동일
    # 패턴) 여기서 Node로 자동 실행하지 않는다 - show_theoretical_ghost:=true로
    # 이 launch를 띄운 뒤, 필요할 때 별도 터미널에서
    # `python3 scripts/theoretical_state_relay.py`로 직접 실행할 것.
    theoretical_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"frame_prefix": "theoretical_"}],
        remappings=[("joint_states", "/theoretical_joint_states")],
        condition=IfCondition(show_theoretical_ghost),
    )
    theoretical_world_link_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "theoretical_world"],
        condition=IfCondition(show_theoretical_ghost),
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
            theoretical_robot_state_publisher_node,
            theoretical_world_link_node,
        ]
    )
