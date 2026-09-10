"""팔 제어: TF 변환 -> MoveIt/IK -> 실제 명령(엔코더/CAN 송신) bringup.

depth_camera_ik_bringup.launch.py에서 분리한 조각 중 "TF 후 MoveIt/IK로
실제 명령값을 만들어 관절에 보내고, 그 값을 관측용으로 병렬 출력"하는
부분만 담당한다. /arm/target_point(PointStamped)를 구독하는 쪽부터다 -
그 점을 만드는 카메라 인식/좌표추출은 perception_bringup.launch.py 소관.

기동 순서 (mock_bringup.launch.py 내부):
  robot_state_publisher -> ros2_control_node -> joint_state_broadcaster
  -> arm_controller/gripper_controller -> move_group -> (launch_rviz:=true면 RViz)
  -> maru_ik_node (target_point 구독, TF 변환, GetPositionIK로 관절해,
     arm_controller/gripper_controller로 실행 - 실제 명령 경로는 이 하나뿐)

관측용 출력:
  planned_encoder_trajectory가 MoveIt 계획 경로를 raw encoder-radian 값으로
  변환해 /arm/planned_encoder_trajectory에 "읽기 전용"으로 발행한다. 이 노드는
  CAN 프레임을 직접 보내지 않는다 - 실제 명령 경로는 위 maru_ik_node ->
  arm_controller -> ros2_control 하나만 유지해야 하고(planned_encoder_trajectory.py
  파일 상단 주석 참고), 여기서는 그 값을 터미널에 병렬로 찍어서 확인하는
  용도로만 쓴다.

  [추가, 2026-09-01] arm_pose_array_publisher도 같은 이유로 여기서 같이
  띄운다 - TF(robot_state_publisher)를 조회해 각 관절의 실측 X-Z 좌표를
  /arm/joint_pose_array(PoseArray)로 발행해서 DolbotZ-Center 웹 UI가 쓴다.
  ui_bringup.launch.py(RViz)가 아니라 여기 둔 이유: JECS 등 SSH-only 환경은
  GUI가 없어 ui_bringup.launch.py 자체를 안 띄우는 경우가 많은데(RViz는
  로컬 머신에서 따로), 이 노드는 화면이 필요 없고 TF만 있으면 되므로 RViz
  여부와 무관하게 항상 떠 있어야 웹 UI가 계속 실측 좌표를 받는다. 이 노드가
  전혀 안 떠 있으면 app.js가 shoulder/elbow/wrist 각도만으로 자체 근사
  FK(중립 톱니바퀴 설정, 실측 미보정)로 폴백해서 X/Z가 실제와 어긋나 보인다.

real hardware:
  use_mock_hardware:=false 로 전환 시 army_manipulator_ros2_control.xacro의
  usb_port/ifname/actuator_id를 실제 배선에 맞게 먼저 수정할 것
  (mock_bringup.launch.py 상단 주석 참고).

JECS 등 실기에서 SSH로만 접속할 때는 launch_rviz:=false로 여기 RViz는 끄고,
RViz는 같은 저장소가 있는 로컬 머신에서 ui_bringup.launch.py(=
moveit_rviz.launch.py)로 따로 띄워서 네트워크로 붙는다(같은 ROS_DOMAIN_ID 필요).

AUTO 게이트:
  maru_ik_node는 시작부터 AUTO 활성 상태라 /arm/target_point를 받으면 바로
  계산/실행한다. auto_enable(기본 true)이 true면 이 launch도
  /control/auto_enabled=true를 1Hz로 반복 발행한다. 따라서 외부에서 AUTO를
  껐다가 다시 켜도 다음 주기에 자동 운전 상태가 복구된다.
  주의: 이 게이트는 launch 시작 직후 팔 동작을 즉시 허용한다. 미션 시작 승인/
  비상정지/실행 취소를 담당하는 안전 관리자 노드가 아직 없으니, 실기에서 켜기
  전에 반드시 사람이 옆에서 지켜보고 있을 것. auto_enable:=false로 끄면 예전처럼
  수동으로 /control/auto_enabled를 publish해야 움직인다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="mock_bringup.launch.py로 그대로 전달. false면 실제 하드웨어 플러그인 사용.",
        ),
        DeclareLaunchArgument(
            "use_mesh",
            default_value="true",
            description="mock_bringup.launch.py로 그대로 전달.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description=(
                "mock_bringup.launch.py로 그대로 전달. false면 여기서 RViz를 안 띄운다 - "
                "JECS 등 실기에서는 끄고 ui_bringup.launch.py를 로컬에서 따로 띄울 것."
            ),
        ),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="true",
            description=(
                "true면 launch 시작과 동시에 /control/auto_enabled=true를 반복 발행해서 "
                "AUTO(카메라 타겟 인식 시 자동 이동)를 자동으로 켠다. 실기에서 사람 확인 없이 "
                "테스트하려면 false로 끄고 수동으로 publish할 것."
            ),
        ),
        DeclareLaunchArgument(
            "ik_max_seed_attempts", default_value="6",
            description=(
                "target/pitch 하나당 시도할 우선순위 IK seed 최대 개수. "
                "current seed 후 원거리용 거의 편 seed까지 포함하려면 6이 필요."
            ),
        ),
        DeclareLaunchArgument(
            "ik_request_timeout_sec", default_value="0.5",
            description="각 compute_ik seed 요청의 최대 시간(초).",
        ),
        DeclareLaunchArgument(
            "ik_solver_backend", default_value="analytic",
            description=(
                "analytic(기본, 닫힌해 - 반복/seed 없음, <1ms) | moveit(/compute_ik KDL) "
                "| numerical(로컬 DLS 수치해). maru_ik_node.py ik_solver_backend 주석 참고."),
        ),
        DeclareLaunchArgument("numerical_ik_max_iterations", default_value="250"),
        DeclareLaunchArgument("numerical_ik_position_tolerance_m", default_value="0.003"),
        DeclareLaunchArgument("numerical_ik_pitch_tolerance_rad", default_value="0.025"),
        DeclareLaunchArgument("numerical_ik_damping", default_value="0.025"),
        DeclareLaunchArgument(
            "ik_solution_cache_size", default_value="16",
            description="재사용할 검증된 IK 해 캐시 크기. 0이면 비활성.",
        ),
        DeclareLaunchArgument(
            "ik_cached_seed_count", default_value="3",
            description="현재 pose와 가까운 캐시 해 중 seed로 우선 시도할 개수.",
        ),
        DeclareLaunchArgument(
            "ik_avoid_collisions", default_value="true",
            description="true면 compute_ik 단계부터 충돌 해를 거절.",
        ),
        DeclareLaunchArgument(
            "workspace_max_tcp_distance_m", default_value="0.0",
            description=(
                "양수면 해당 반경 밖 목표를 IK 전에 거절. 0.0은 실기 오판 방지를 위해 "
                "사전검사를 끄고 실제 IK로 판정."),
        ),
        DeclareLaunchArgument(
            "grasp_start_x_distance_m", default_value="0.21",
            description=(
                "보정된 base_actuator 기준 정면 X축 거리가 이 값보다 멀면 좌표를 "
                "latch하지 않고 forward_command만 발행. 0 이하면 비활성."),
        ),
        DeclareLaunchArgument(
            "direct_grasp_max_camera_depth_m", default_value="1.0",
            description=(
                "YOLO bbox 중심 camera optical Z가 이 값 이하면 base X 접근 게이트를 "
                "우회하고 즉시 팔 IK 시도. 0 이하면 비활성."),
        ),
        DeclareLaunchArgument(
            "forward_command_min_interval_sec", default_value="2.0",
            description="거리 접근용 forward_command 최소 발행 간격(초).",
        ),
        DeclareLaunchArgument(
            "startup_grasp_wait_delay_sec", default_value="2.0",
            description="유효 joint_states/AUTO 준비 후 시작 GRASP_WAIT 대기시간(초).",
        ),
        DeclareLaunchArgument(
            "startup_grasp_wait_duration_sec", default_value="3.0",
            description="시작 GRASP_WAIT 직접 controller 이동시간(초).",
        ),
        DeclareLaunchArgument(
            "initial_pose_launch_delay_sec", default_value="0.0",
            description=(
                "유효 joint_states와 arm_controller 준비 완료 후 시작 자세 명령까지 "
                "기다릴 시간(초). 기본 0.0으로 준비 즉시 명령."),
        ),
        DeclareLaunchArgument(
            "startup_pose_complete_topic", default_value="/arm/startup_pose_complete",
            description="초기 자세 직접 이동 성공 후 maru_ik_node gate 해제 토픽.",
        ),
        DeclareLaunchArgument(
            "grasp_wait_preset", default_value="legacy",
            description=(
                "시작/복귀 대기 자세: legacy 또는 grasp_wait1~grasp_wait5. "
                "base_joint는 선택값과 무관하게 현재 위치를 유지."),
        ),
        DeclareLaunchArgument(
            "precision_failures_before_approach", default_value="2",
            description="IK 또는 빈 파지가 연속 이 횟수 실패하면 forward_command 접근 재시도.",
        ),
        DeclareLaunchArgument(
            "approach_retry_wait_sec", default_value="1.0",
            description="forward_command nudge 뒤 grasp_wait 복귀 전 대기 시간(초).",
        ),
        DeclareLaunchArgument(
            "picking_state_topic", default_value="/picking",
            description="첫 위치 인식 성공(target latch)부터 파지/복귀 완료까지 true인 Bool 상태 토픽.",
        ),
        DeclareLaunchArgument(
            "calculation_failure_topic", default_value="/arm/calculation_failed",
            description="IK/workspace/planning 계산 실패 때 발행하는 Empty 이벤트 토픽.",
        ),
        DeclareLaunchArgument(
            "target_point_base_topic", default_value="/arm/target_point_base",
            description="실제 IK와 동일하게 보정된 base-frame TCP 목표 토픽.",
        ),
        DeclareLaunchArgument(
            "target_distance_topic", default_value="/arm/target_distance_m",
            description="보정된 TCP 목표의 planning-frame 원점 기준 거리 토픽.",
        ),
        DeclareLaunchArgument(
            "manual_override_topic", default_value="/control/arm_manual_override",
            description="직접 controller 명령 동안 MoveIt 자동명령을 차단하는 Bool 토픽.",
        ),
        DeclareLaunchArgument(
            "manual_override_timeout_sec", default_value="1.5",
            description="수동 명령 heartbeat 단절 후 AUTO 명령권을 복구하는 시간(초).",
        ),
    ]

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mesh = LaunchConfiguration("use_mesh")
    launch_rviz = LaunchConfiguration("launch_rviz")
    auto_enable = LaunchConfiguration("auto_enable")
    ik_argument_names = (
        "ik_max_seed_attempts", "ik_request_timeout_sec", "ik_solver_backend",
        "numerical_ik_max_iterations", "numerical_ik_position_tolerance_m",
        "numerical_ik_pitch_tolerance_rad", "numerical_ik_damping",
        "ik_solution_cache_size",
        "ik_cached_seed_count", "ik_avoid_collisions", "workspace_max_tcp_distance_m",
        "grasp_start_x_distance_m", "direct_grasp_max_camera_depth_m",
        "forward_command_min_interval_sec",
        "startup_grasp_wait_delay_sec",
        "startup_grasp_wait_duration_sec",
        "startup_pose_complete_topic",
        "grasp_wait_preset",
        "precision_failures_before_approach", "approach_retry_wait_sec",
        "picking_state_topic", "calculation_failure_topic",
        "target_point_base_topic", "target_distance_topic",
        "manual_override_topic",
        "manual_override_timeout_sec",
    )

    arm_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("army_manipulator_bringup"), "launch", "mock_bringup.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "use_mesh": use_mesh,
            "launch_rviz": launch_rviz,
            **{name: LaunchConfiguration(name) for name in ik_argument_names},
            "startup_grasp_wait_external": "true",
        }.items(),
    )

    initial_pose_node = Node(
        package="army_manipulator_bringup",
        executable="move_to_named_pose.py",
        name="startup_named_pose",
        output="screen",
        parameters=[{
            "pose": ParameterValue(
                LaunchConfiguration("grasp_wait_preset"), value_type=str),
            "hold_current_base": True,
            "duration_sec": ParameterValue(
                LaunchConfiguration("startup_grasp_wait_duration_sec"), value_type=float),
            "completion_topic": ParameterValue(
                LaunchConfiguration("startup_pose_complete_topic"), value_type=str),
            "joint_state_wait_timeout_sec": 0.0,
            "controller_wait_timeout_sec": 0.0,
            "ready_delay_sec": ParameterValue(
                LaunchConfiguration("initial_pose_launch_delay_sec"), value_type=float),
            "retry_until_success": True,
            "retry_interval_sec": 1.0,
        }],
    )

    planned_encoder_trajectory_node = Node(
        package="army_manipulator_bringup",
        executable="planned_encoder_trajectory.py",
        output="screen",
    )

    arm_pose_array_publisher_node = Node(
        package="army_manipulator_bringup",
        executable="arm_pose_array_publisher.py",
        output="screen",
    )

    auto_enable_publisher = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "-r", "1",
            "--qos-reliability", "reliable",
            "--qos-durability", "transient_local",
            "/control/auto_enabled", "std_msgs/msg/Bool", "{data: true}",
        ],
        output="screen",
        condition=IfCondition(auto_enable),
    )

    return LaunchDescription(
        declared_arguments
        + [
            arm_and_moveit,
            initial_pose_node,
            planned_encoder_trajectory_node,
            arm_pose_array_publisher_node,
            auto_enable_publisher,
        ]
    )
