#!/usr/bin/env python3
"""IK 노드.

이 노드는 카메라 기준으로 들어온 타겟 좌표를 바탕으로,
base pan(yaw)와 접근 pitch를 계산한 뒤 MoveIt2의 GetPositionIK 서비스로
4축(베이스/숄더/엘보우/리스트) 목표 자세를 구해
joint_command_mux로 publish 한다.

주요 가정:
- 카메라/타깃 좌표는 `geometry_msgs/PointStamped` 또는 `PoseStamped` 형태로 들어온다.
- 검출 로직 자체는 별도 노드에서 수행된다고 보고, 여기서는 좌표 입력만 받는다.
- 실제 D435i 검출 파이프라인이 붙으면 `target_topic` 또는 `camera` 관련 파라미터를 맞춰주면 된다.

TODO(mux): dxl_ee의 joint_command_mux 노드가 아직 이 워크스페이스에 통합되지
않아서, `/joint_command_mux`(mux_topic)로 publish해도 구독자가 없어 팔이
움직이지 않는다. 그래서 `direct_control` 파라미터(기본 true)가 켜져 있는 동안은
IK 해를 arm_controller/gripper_controller의 FollowJointTrajectory 액션으로도
직접 보낸다. 실제 mux 노드가 붙어서 teleop/IK 우선순위를 알아서 중재하게 되면
`direct_control:=false`로 끄고 이 direct-control 경로는 지워도 된다.

TODO(zero-offset): 아래에서 쓰는 JointCalibration은
army_manipulator_description/config/joint_calibration.yaml의 zero_offset(현재
전부 0.0 placeholder)에 의존한다. 실측 후 그 yaml만 갱신하면 이 노드의 clamp/
raw 변환 로직에 자동 반영된다.

각도 좌표계 정리:
- IK(MoveIt GetPositionIK)와 joint_states, arm_controller/gripper_controller
  (ros2_control)는 모두 URDF 기준, 즉 actual_joint_angle 좌표계로 동작한다
  (URDF <limit>도 joint_calibration.yaml에서 actual_limit로 계산됨).
- `/joint_command_mux`(dxl_ee 쪽)는 raw 인코더 좌표계를 기대하므로, publish_mux()
  에서 actual -> raw 역변환(actual_angle + zero_offset)을 거친다.
"""

import math
from typing import List, Optional, Sequence, Tuple

from joint_calibration import JointCalibration

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PointStamped, PoseStamped
    from std_msgs.msg import Float64MultiArray
    from builtin_interfaces.msg import Duration as BuiltinDuration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
    from moveit_msgs.srv import GetPositionIK
    import tf2_ros
    from tf2_geometry_msgs import do_transform_pose
except ModuleNotFoundError:  # pragma: no cover - 테스트 환경에서 ROS 미설치 시에도 import 가능
    rclpy = None
    ActionClient = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    Duration = None
    Time = None
    JointState = None
    PointStamped = None
    PoseStamped = None
    Float64MultiArray = None
    BuiltinDuration = None
    JointTrajectory = None
    JointTrajectoryPoint = None
    FollowJointTrajectory = None
    PositionIKRequest = None
    RobotState = None
    MoveItErrorCodes = None
    GetPositionIK = None
    tf2_ros = None
    do_transform_pose = None


DEFAULT_JOINT_NAMES = [
    "base_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
]


# TODO(home-pose): home_pitch/max_pitch/z_min/z_max 기본값은 실제 CATIA 홈 자세와
# 조인트 리밋이 확정되기 전까지의 플레이스홀더다. 사용자가 CATIA 확인 후 알려주는
# 값으로 아래 기본값과 declare_parameter 기본값을 함께 교체할 것.
def compute_approach_pitch(
    target_z: float,
    home_pitch: float = -0.25,
    max_pitch: float = -1.20,
    z_min: float = 0.20,
    z_max: float = 0.55,
) -> float:
    """타겟 높이에 따라 접근 pitch를 선형 보간한다.

    기본 자세(home_pitch)에서 타겟이 높아질수록 더 아래를 바라보게 하는 방식으로,
    IK 해가 리미트 안에서 나올 수 있도록 상한값까지 clamp 한다.
    """
    if target_z <= z_min:
        return home_pitch
    if target_z >= z_max:
        return max_pitch
    ratio = (target_z - z_min) / (z_max - z_min)
    return home_pitch + ratio * (max_pitch - home_pitch)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Sequence[float]:
    """표준 항공 관례 RPY(Rz(yaw)*Ry(pitch)*Rx(roll), pitch는 Y축)를 quaternion으로
    변환한다. 반환 순서는 (w, x, y, z)다. 이 로봇에는 쓰지 않는다 -
    quaternion_from_yaw_pitch() 참고."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def quaternion_from_yaw_pitch(yaw: float, pitch: float) -> Sequence[float]:
    """이 로봇의 실제 회전축 구성에 맞는 Rz(yaw) * Rx(pitch) 합성 quaternion을
    반환한다(w, x, y, z 순서). base_joint는 Z축 요, shoulder/elbow/wrist는 전부
    X축 피치라서(quaternion_from_rpy가 가정하는 Y축 피치가 아님), 표준
    Rz*Ry*Rx 공식 대신 이 조합을 써야 IK가 실제 도달 가능한 자세를 요청한다
    (FK round-trip으로 검증: base=0.5, shoulder=elbow=wrist=-0.4 지점에서
    yaw=0.5, pitch=-1.2로 IK 요청 시 성공(error_code=1) 확인)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    return [
        cy * cp,
        cy * sp,
        sy * sp,
        sy * cp,
    ]


class MaruIkNode(Node):
    """D435i 기준 좌표를 받아 MoveIt2 IK로 팔 자세를 해석한다."""

    # base_joint=0(yaw 없음)일 때 wrist_link의 X좌표(m). shoulder/elbow/wrist가
    # 전부 X축 회전이라 이 값은 그 세 조인트 각도와 무관하게 항상 고정이다
    # (FK 실측: bend (0,0,0)과 (-0.4,-0.4,-0.4) 둘 다 x=0.0313으로 동일).
    # compute_and_publish()의 yaw 역산에 쓴다.
    WRIST_X_AT_ZERO_YAW = 0.0313

    def __init__(self):
        super().__init__("maru_ik_node")

        self.declare_parameter("target_topic", "/maru/target/point")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("mux_topic", "/joint_command_mux")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link_name", "wrist_link")
        # TODO(home-pose): 아래 4개 기본값은 CATIA 실측/정식 캘리브레이션 전까지의
        # 임시값이다. FK 샘플링으로 재보정함(이전 -0.25/-1.20/0.20/0.55는 실제
        # 지오메트리와 최대 2배 가까이 어긋나 있었음):
        #   z=0.504(=home, shoulder/elbow/wrist 전부 0) -> pitch=0
        #   z=0.223(=shoulder/elbow/wrist 각 -0.8, 총 -2.4rad 굽힘) -> pitch=-2.4
        # 위 두 실측점을 잇는 선형 보간이라 z_min/z_max 사이에서도 여전히 근사값
        # 이지만(3링크 굽힘 관계 자체가 비선형이라 완벽히 맞진 않는다), 기존
        # 플레이스홀더보다 실제 도달 가능 범위에 훨씬 가깝다. 여유 마진을 두고
        # z_min=0.23(<0.223 한계), z_max=0.50(<0.504 한계), max_pitch=-2.2(<-2.4 한계)로 설정.
        self.declare_parameter("home_pitch", 0.0)
        self.declare_parameter("max_pitch", -2.2)
        self.declare_parameter("z_min", 0.23)
        self.declare_parameter("z_max", 0.50)
        self.declare_parameter("publish_rate_hz", 10.0)

        # mux 부재 gap 대응: joint_command_mux에 구독자가 없는 동안 IK 해를
        # arm_controller/gripper_controller에 직접 FollowJointTrajectory로 보낸다.
        # TODO(mux): dxl_ee의 joint_command_mux 노드가 통합되면 direct_control:=false
        # 로 끄고 이 경로를 제거할 것.
        self.declare_parameter("direct_control", True)
        self.declare_parameter("arm_controller_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("trajectory_time_from_start", 0.5)

        self.target_topic = self.get_parameter("target_topic").value
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.ik_service = self.get_parameter("ik_service").value
        self.mux_topic = self.get_parameter("mux_topic").value
        self.group_name = self.get_parameter("group_name").value
        self.ik_link_name = self.get_parameter("ik_link_name").value
        self.home_pitch = float(self.get_parameter("home_pitch").value)
        self.max_pitch = float(self.get_parameter("max_pitch").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.direct_control = bool(self.get_parameter("direct_control").value)
        self.arm_controller_action = self.get_parameter("arm_controller_action").value
        self.trajectory_time_from_start = float(self.get_parameter("trajectory_time_from_start").value)

        # joint_calibration.yaml(raw_min/raw_max/zero_offset) 기준 actual_limit clamp
        # 및 raw <-> actual 변환. base yaw clamp도 이걸로 대체(기존 하드코딩된
        # 대칭 ±180° base_yaw_limit 파라미터 제거).
        self.calibration = JointCalibration()

        self.joint_names = list(DEFAULT_JOINT_NAMES)
        self.last_joint_positions: List[float] = [0.0] * len(self.joint_names)
        self.last_target: Optional[Tuple[float, float, float]] = None
        self.tf_buffer = None
        self.tf_listener = None

        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10)
        if ReliabilityPolicy is not None and DurabilityPolicy is not None:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE

        self.joint_states_sub = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_callback, qos)
        self.target_sub = self.create_subscription(PointStamped, self.target_topic, self.target_callback, qos)
        self.mux_pub = self.create_publisher(Float64MultiArray, self.mux_topic, 10)
        self.ik_client = self.create_client(GetPositionIK, self.ik_service)

        self.arm_action_client = None
        if self.direct_control and ActionClient is not None:
            self.arm_action_client = ActionClient(self, FollowJointTrajectory, self.arm_controller_action)

        self.get_logger().info(
            f"MARU IK node ready. target_topic={self.target_topic}, ik_service={self.ik_service}, "
            f"mux_topic={self.mux_topic}, direct_control={self.direct_control}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        """joint_states에서 현재 자세를 기억한다."""
        try:
            names = list(msg.name)
            positions = list(msg.position)
            mapping = {name: pos for name, pos in zip(names, positions)}
            self.last_joint_positions = [
                float(mapping.get(name, self.last_joint_positions[idx])) for idx, name in enumerate(self.joint_names)
            ]
        except Exception as exc:  # pragma: no cover - 방어 코드
            self.get_logger().warn(f"Failed to parse joint_states: {exc}")

    def target_callback(self, msg: PointStamped) -> None:
        """타깃 좌표를 수신하면 IK를 계산해 mux로 publish 한다."""
        if self.tf_buffer is not None and msg.header.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    msg.header.frame_id,
                    self.get_clock().now(),
                    timeout=Duration(seconds=0.5),
                )
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = msg.point.x
                pose.pose.position.y = msg.point.y
                pose.pose.position.z = msg.point.z
                pose = do_transform_pose(pose, transform)
                target_xyz = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            except Exception as exc:  # pragma: no cover - 환경 의존
                self.get_logger().warn(f"TF transform failed: {exc}; using raw target data")
                target_xyz = (msg.point.x, msg.point.y, msg.point.z)
        else:
            target_xyz = (msg.point.x, msg.point.y, msg.point.z)

        self.last_target = target_xyz
        self.compute_and_publish(target_xyz)

    def compute_and_publish(self, target_xyz: Tuple[float, float, float]) -> None:
        """타깃으로부터 yaw를 계산하고, pitch 후보들을 IK 성공할 때까지 시도한다."""
        x, y, z = target_xyz
        # base_joint는 planning_frame 기준 Z축 회전이다. shoulder/elbow/wrist가
        # 전부 X축 회전이라 base_joint=0일 때 wrist_link의 X좌표는 그 세 조인트
        # 각도와 무관하게 항상 WRIST_X_AT_ZERO_YAW로 고정된다(FK 실측: bend
        # (0,0,0)과 (-0.4,-0.4,-0.4) 둘 다 x=0.0313으로 동일 - X축 회전은
        # 정의상 X성분을 바꾸지 못하므로 이는 근사가 아니라 정확한 불변량이다).
        # 따라서 목표(x,y)에 대해 y0=sqrt(x^2+y^2-x0^2)(항상 양수인 "전진" 성분)
        # 로 base_joint=0 기준 미회전 벡터(x0,y0)를 복원한 뒤,
        # yaw = atan2(y,x) - atan2(y0,x0) 로 정확히 구한다(회전은 각도 뺄셈과
        # 동치). base_joint=0.5rad로 직접 FK 실측해서 검산 완료.
        rho_sq = x * x + y * y
        y0 = math.sqrt(max(0.0, rho_sq - self.WRIST_X_AT_ZERO_YAW ** 2))
        yaw = math.atan2(y, x) - math.atan2(y0, self.WRIST_X_AT_ZERO_YAW)
        # base_joint의 actual_limit(= raw_limit - zero_offset) 기준으로 clamp
        yaw = self.calibration.clamp_to_actual_limit("base_joint", yaw)
        heuristic_pitch = compute_approach_pitch(
            z, home_pitch=self.home_pitch, max_pitch=self.max_pitch, z_min=self.z_min, z_max=self.z_max
        )

        # [설계] shoulder/elbow/wrist 3개가 접근각(pitch) 하나를 만드는 데 협력하는
        # 중복(redundant) 자유도라, target 높이(z) 하나만으로는 그 지점에 실제
        # 도달 가능한 pitch를 정확히 역산할 수 없다(비선형+비유일). compute_approach_pitch
        # 는 "괜찮은 시작 추정치"만 제공하고, 실제로는 IK가 성공할 때까지 후보
        # pitch들을 순서대로 시도한다 - 폐형식 공식 하나에 기대지 않고 IK 자체가
        # 도달 가능 여부를 판정하게 하는 편이 이 4자유도 팔에는 더 안정적이다.
        candidates = list(self._pitch_candidates(heuristic_pitch))
        self._try_next_pitch(x, y, z, yaw, candidates, 0)

    def _try_next_pitch(
        self, x: float, y: float, z: float, yaw: float, candidates: List[float], idx: int
    ) -> None:
        """candidates[idx]로 IK를 비동기 시도하고, 실패하면 다음 후보로 넘어간다.

        [수정] 예전엔 target_callback 콜백 안에서 rclpy.spin_until_future_complete()로
        서비스 응답을 블로킹 대기했다. 기본 SingleThreadedExecutor는 콜백 실행
        중에는 그 콜백을 낸 실행기 자신이 서비스 응답을 처리할 스핀을 돌 수 없어
        매번 타임아웃으로 데드락됐다(raw service call로 직접 부르면 항상 성공하는데
        노드를 거치면 매번 timeout이었던 진짜 원인). MultiThreadedExecutor로
        바꿔도 ActionClient(FollowJointTrajectory)와 조합에서 rclpy/rcl_action의
        wait-set 처리 버그(RCLError: wait set index ... out of bounds)로 노드가
        죽었다. 그래서 블로킹 스핀 자체를 없애고 add_done_callback()으로 완전히
        비동기 체인으로 재작성했다 - 콜백은 즉시 반환하고, 응답이 오면 실행기가
        평소처럼 그 콜백을 실행해 다음 후보를 이어간다."""
        if idx >= len(candidates):
            self.get_logger().warn("IK failed for all pitch candidates; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        if self.ik_client is None or not self.ik_client.service_is_ready():
            self.get_logger().warn("IK service not ready; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        pitch = candidates[idx]
        pose = self._build_pose(x, y, z, pitch, yaw)

        ik_request = PositionIKRequest()
        ik_request.group_name = self.group_name
        ik_request.ik_link_name = self.ik_link_name
        ik_request.pose_stamped = pose
        ik_request.robot_state = RobotState()
        ik_request.robot_state.joint_state.name = self.joint_names
        ik_request.robot_state.joint_state.position = self.last_joint_positions
        ik_request.avoid_collisions = True

        req = GetPositionIK.Request()
        req.ik_request = ik_request

        future = self.ik_client.call_async(req)
        future.add_done_callback(
            lambda f, x=x, y=y, z=z, yaw=yaw, candidates=candidates, idx=idx: self._on_ik_response(
                f, x, y, z, yaw, candidates, idx
            )
        )

    def _on_ik_response(
        self,
        future,
        x: float,
        y: float,
        z: float,
        yaw: float,
        candidates: List[float],
        idx: int,
    ) -> None:
        positions = self._extract_positions(future)
        if positions is not None:
            self.publish_mux(positions)
            # mux 부재 gap 대응: 성공한 IK 해만 direct-control 경로로도 내려보낸다.
            self.send_direct_trajectory(positions)
            return
        self._try_next_pitch(x, y, z, yaw, candidates, idx + 1)

    def _pitch_candidates(self, heuristic_pitch: float):
        """heuristic 추정치를 먼저 시도하고, 실패하면 [home_pitch, max_pitch] 구간을
        촘촘히 스캔한다."""
        yield heuristic_pitch
        steps = 9
        lo, hi = min(self.home_pitch, self.max_pitch), max(self.home_pitch, self.max_pitch)
        for i in range(steps):
            candidate = lo + (hi - lo) * i / (steps - 1)
            if abs(candidate - heuristic_pitch) > 1e-6:
                yield candidate

    def _build_pose(self, x: float, y: float, z: float, pitch: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # quaternion_from_yaw_pitch()는 (w, x, y, z) 순서로 반환한다. 표준
        # RPY(Y축 피치) 대신 이 로봇의 실제 축 구성(Z축 요 * X축 피치)에 맞는
        # 합성을 쓴다 - 안 그러면 pitch 값이 정확해도 요청 orientation 자체가
        # 물리적으로 존재하지 않는 회전이라 IK가 항상 NO_IK_SOLUTION을 낸다.
        pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = quaternion_from_yaw_pitch(
            yaw, pitch
        )
        return pose

    def _extract_positions(self, future) -> Optional[List[float]]:
        """완료된(add_done_callback으로 넘어온) IK future에서 관절해를 뽑는다.

        성공하면 actual_limit로 clamp된 관절해를, 실패면 None을 반환한다."""
        exc = future.exception()
        if exc is not None:
            self.get_logger().warn(f"IK service call raised: {exc}")
            return None

        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        solution = response.solution.joint_state
        if solution is None or len(solution.position) == 0:
            return None

        result_positions = [float(pos) for pos in solution.position[: len(self.joint_names)]]
        # IK/MoveIt 결과를 최종적으로 한 번 더 actual_limit(joint_calibration.yaml
        # 기준)으로 clamp해서 하드웨어에 리미트를 벗어난 명령이 나가지 않게 한다.
        return [
            self.calibration.clamp_to_actual_limit(name, pos)
            for name, pos in zip(self.joint_names, result_positions)
        ]

    def publish_mux(self, positions: Sequence[float]) -> None:
        """actual(URDF/IK) 각도를 raw 인코더 각도로 역변환해 mux로 publish 한다.

        역변환: raw_encoder_target = actual_angle + zero_offset
        (mux 반대편의 dxl_ee 하드웨어 브릿지는 raw 인코더 좌표계를 기대하므로,
        여기서만 actual -> raw 변환을 거친다. arm_controller/gripper_controller
        쪽 direct-control 경로는 ros2_control/URDF가 이미 actual 좌표계이므로
        변환하지 않는다.)
        """
        raw_positions = [
            self.calibration.actual_to_raw(name, pos) for name, pos in zip(self.joint_names, positions)
        ]
        msg = Float64MultiArray()
        msg.data = [float(pos) for pos in raw_positions]
        self.mux_pub.publish(msg)
        self.get_logger().debug(f"Published to {self.mux_topic}: {msg.data}")

    def send_direct_trajectory(self, positions: Sequence[float]) -> None:
        """TODO(mux): joint_command_mux가 아직 없어서 arm_controller에 직접 보내는
        임시 경로. 실제 mux 노드가 teleop/IK 명령을 중재하게 되면 direct_control
        파라미터를 false로 두고 이 메서드 호출부(solve_ik의 마지막 줄)를 지울 것.
        """
        if not self.direct_control or self.arm_action_client is None:
            return
        if not self.arm_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(f"{self.arm_controller_action} action server not available; skipping direct control")
            return

        point = JointTrajectoryPoint()
        point.positions = [float(pos) for pos in positions]
        seconds = int(self.trajectory_time_from_start)
        nanosec = int((self.trajectory_time_from_start - seconds) * 1e9)
        point.time_from_start = BuiltinDuration(sec=seconds, nanosec=nanosec)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = [point]

        self.arm_action_client.send_goal_async(goal)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is not available in this environment")

    rclpy.init(args=args)
    node = MaruIkNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
