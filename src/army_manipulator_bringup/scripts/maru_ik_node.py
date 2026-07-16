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
    "base_rotate_joint",
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
    """RPY를 quaternion으로 변환한다. roll은 자유도 없음으로 0으로 고정해도 무방하다."""
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


class MaruIkNode(Node):
    """D435i 기준 좌표를 받아 MoveIt2 IK로 팔 자세를 해석한다."""

    def __init__(self):
        super().__init__("maru_ik_node")

        self.declare_parameter("target_topic", "/maru/target/point")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("mux_topic", "/joint_command_mux")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link_name", "wrist_link")
        # TODO(home-pose): 아래 4개 기본값은 CATIA 홈 자세/조인트 리밋 확정 전
        # 플레이스홀더. 확정되면 launch 파라미터나 이 기본값을 실측값으로 교체할 것.
        self.declare_parameter("home_pitch", -0.25)
        self.declare_parameter("max_pitch", -1.20)
        self.declare_parameter("z_min", 0.20)
        self.declare_parameter("z_max", 0.55)
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
        """타깃으로부터 yaw/pitch를 계산하고 IK를 구한 뒤 mux로 보낸다."""
        x, y, z = target_xyz
        yaw = math.atan2(y, x)
        # base_rotate_joint의 actual_limit(= raw_limit - zero_offset) 기준으로 clamp
        yaw = self.calibration.clamp_to_actual_limit("base_rotate_joint", yaw)
        pitch = compute_approach_pitch(z, home_pitch=self.home_pitch, max_pitch=self.max_pitch, z_min=self.z_min, z_max=self.z_max)

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_rpy(
            0.0, pitch, yaw
        )

        self.solve_ik(pose)

    def solve_ik(self, pose: PoseStamped) -> None:
        """MoveIt2의 GetPositionIK 서비스로 IK를 해결한 뒤 command mux에 publish 한다."""
        if self.ik_client is None or not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("IK service not available; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        req = PositionIKRequest()
        req.group_name = self.group_name
        req.ik_link_name = self.ik_link_name
        req.pose_stamped = pose
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = self.last_joint_positions
        req.avoid_collisions = True

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().warn("IK request timed out; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"IK failed: {response.error_code.val}; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        solution = response.solution.joint_state
        if solution is None or len(solution.position) == 0:
            self.get_logger().warn("IK returned no solution; keeping last joint values")
            self.publish_mux(self.last_joint_positions)
            return

        result_positions = [float(pos) for pos in solution.position[: len(self.joint_names)]]
        # IK/MoveIt 결과를 최종적으로 한 번 더 actual_limit(joint_calibration.yaml
        # 기준)으로 clamp해서 하드웨어에 리미트를 벗어난 명령이 나가지 않게 한다.
        clamped_positions = [
            self.calibration.clamp_to_actual_limit(name, pos)
            for name, pos in zip(self.joint_names, result_positions)
        ]
        self.publish_mux(clamped_positions)
        # mux 부재 gap 대응: 성공한 IK 해만 direct-control 경로로도 내려보낸다.
        self.send_direct_trajectory(clamped_positions)

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
