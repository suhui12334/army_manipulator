#!/usr/bin/env python3
"""Minimal supply-box grasp controller.

Runtime sequence only:

  startup GRASP_WAIT (+ gripper open) -> first detected target -> one IK/controller move
  -> 2-second settle -> full gripper close (GRIPPER_CLOSED, no current cutoff)
  -> HOLD -> HOME -> picking_command -> BED (while driving)

There is no pregrasp/descend/lift/retry/drive state machine in this node.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState as RobotStateMsg
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

from joint_calibration import JointCalibration


ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
GRIPPER_JOINT_NAMES = ["gripper_joint"]
# Real RMD speed limits configured in army_manipulator_ros2_control.xacro.
# The driver receives these values in deg/s; use the same limits when deciding
# how long a FollowJointTrajectory must last so its reference does not outrun
# the physical actuator.
RMD_MAX_VELOCITY_RAD_S = {
    "shoulder_joint": math.radians(15.0),
    "elbow_joint": math.radians(5.0),
    "wrist_joint": math.radians(30.0),
}
# Actual controller targets for the startup GRASP_WAIT pose: 0/90/90 deg.
# base_joint is intentionally omitted so startup keeps its live encoder value.
GRASP_WAIT_ARM_JOINTS = {
    "shoulder_joint": 0.0,
    "elbow_joint": math.pi / 2,
    "wrist_joint": math.pi / 2,
}
HOLD_ARM_JOINTS = {
    "shoulder_joint": -0.76044,
    "elbow_joint": 1.63830,
    "wrist_joint": 1.53290,
}
# [추가, 2026-09-05] 파지 후 수납 시퀀스: hold -> home -> bed.
# move_to_named_pose.py의 NAMED_ARM_JOINTS(home/bed, 2026-08-31 실측)와 동일값.
# bed는 구동부에 기대 눕는 주행용 자세(capture_arm_pose.py 실측). base_joint는
# 현재값 유지.
HOME_ARM_JOINTS = {
    "shoulder_joint": -1.4818,
    "elbow_joint": 1.6383,
    "wrist_joint": 1.5329,
}
BED_ARM_JOINTS = {
    "shoulder_joint": -1.49610,
    "elbow_joint": 0.35867,
    "wrist_joint": 1.55648,
}
# rosbag ``ik_tf_debug``의 첫 검출점을 수정된 camera optical TF로 다시
# 투영한 TCP=(-0.20627, -0.00206, -0.22540)에 대해, mock MoveIt/KDL에
# random seed 181개를 넣어 실제로 검증한 아래쪽 파지 해다. 기존 current/SID
# seed만으로는 이 해의 basin에 들어가지 못하므로 지면 목표 전용 seed로 쓴다.
# base_joint는 각 타겟의 bearing으로 아래 _solve_ik에서 교체한다.
GROUND_GRASP_IK_SEED = [0.0, 0.9908576865, 1.6512680107, 0.7078743028]
# seed 추출 당시 Z=-0.3060m에서 충돌검사를 켠 MoveIt
# /compute_ik로 실제 추출한 거리별 파지 해다. 각 항목은
# (이름, 대표 반경[m], 검증된 수평 반경 구간[m], 관절) 순서이며, 반경 구간은
# 해당 seed와 그 pitch(관절합)만으로 Y=-30/0/+30mm에서 1회 시도에 IK가 풀린
# 범위다. 기존 current/ground/SID seed만으로는 반경 215mm 미만에서 4회
# 시도가 필요했고, near seed를 먼저 넣으면 160~315mm 전 구간이 1회에 풀린다.
# base_joint는 각 타겟의 bearing으로 _solve_ik에서 교체한다.
# 현재 launch 목표 Z=-0.3080m에서도 반경 160~315mm, Y=-30/0/+30mm의
# 수치 IK 해를 확인했다. 새 높이의 MoveIt 충돌검사/실기 도달은 미확인.
DISTANCE_GRASP_IK_SEEDS = (
    ("near", 0.160, (0.160, 0.290),
     [0.0, 1.3343839224, 1.6489892174, 0.3666268601]),
    ("middle", 0.235, (0.215, 0.315),
     [0.0, 1.3365278559, 1.5292312059, 0.2758335918]),
    ("far", 0.300, (0.215, 0.315),
     [0.0, 1.4922223027, 1.0694275975, 0.5799427533]),
)
# [추가, 2026-09-05] startup grasp_wait 자세는 그리퍼를 연 상태(open)로
# 맞춘다. maru_ik_node.py / move_to_named_pose.py의 GRIPPER_OPEN과 동일값.
GRIPPER_OPEN = 0.07363
GRIPPER_CLOSED = 1.8294
IK_COLLISION_ERROR_CODES = {-10, -12, -22}
# The physical finger part is 80 mm. The kinematic dimension needed by IK is
# instead the measured pinion-axis -> fingertip TCP distance (90 mm), which
# includes the finger mounting offset. Do not substitute 80 mm into the URDF
# transform or add it to the target Z again.
PHYSICAL_FINGER_LENGTH_M = 0.080
WRIST_TO_PINION_DISTANCE_M = 0.150
PINION_TO_TCP_DISTANCE_M = 0.090
# pinion +Z maps to wrist-frame -X through the fixed Ry(-90deg) mounting.
WRIST_TO_TCP_LOCAL_OFFSET = (
    -(WRIST_TO_PINION_DISTANCE_M + PINION_TO_TCP_DISTANCE_M),
    0.0,
    0.0313,
)
# Summer supplybox geometry, all in metres.  The physical finger itself is
# 80 mm long, while the measured pinion-axis -> fingertip TCP distance used by
# the URDF is 90 mm; wrist -> TCP is therefore L3(150 mm) + 90 mm = 240 mm.
DEFAULT_BASE_HEIGHT_M = 0.350
DEFAULT_SUPPLY_BOX_HEIGHT_M = 0.095
DEFAULT_BOX_GRASP_HEIGHT_RATIO = 0.5
DEFAULT_TCP_CALIBRATION_OFFSET_Z_M = 0.0


def rotate_vector_by_quaternion(q, vector):
    ux, uy, uz, w = q
    vx, vy, vz = vector
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    return (
        vx + w * tx + (uy * tz - uz * ty),
        vy + w * ty + (uz * tx - ux * tz),
        vz + w * tz + (ux * ty - uy * tx),
    )


def compute_base_angle(target_x: float, target_y: float) -> float:
    # The TCP lies on the local -X side of the base axis in the current URDF,
    # so its horizontal bearing is base_joint + pi.  This reproduces the
    # measured SID pair: TCP=(-0.33212183, 0.00917221) -> base=-0.02761 rad.
    raw = math.atan2(target_y, target_x) - math.pi
    return (raw + math.pi) % (2.0 * math.pi) - math.pi


def select_distance_seed(radius: float):
    """Pick the verified distance seed for a horizontal TCP radius.

    Seeds whose validated radius band contains ``radius`` are preferred, the
    closest representative radius among them wins. Outside every band the
    nearest representative is still returned as a best-effort first guess.
    """
    inside = [
        entry for entry in DISTANCE_GRASP_IK_SEEDS
        if entry[2][0] - 1e-9 <= radius <= entry[2][1] + 1e-9
    ]
    candidates = inside or list(DISTANCE_GRASP_IK_SEEDS)
    return min(candidates, key=lambda entry: abs(entry[1] - radius))


def quaternion_from_base_pitch(base_angle: float, pitch: float):
    cb, sb = math.cos(base_angle / 2.0), math.sin(base_angle / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    p = 0.5 * (cp + sp)
    q = 0.5 * (cp - sp)
    return (cb * p - sb * q, sb * p + cb * q, sb * p - cb * q, sb * q + cb * p)


def required_arm_duration(current, target, requested_sec: float) -> float:
    """Return a trajectory duration compatible with the configured RMD speeds."""
    required = float(requested_sec)
    for name, start, goal in zip(ARM_JOINT_NAMES, current, target):
        velocity = RMD_MAX_VELOCITY_RAD_S.get(name)
        if velocity is not None and start is not None and math.isfinite(float(start)):
            required = max(required, abs(float(goal) - float(start)) / velocity + 0.5)
    return required


def fixed_grasp_z_from_geometry(
    base_height_m: float,
    box_height_m: float,
    grasp_height_ratio: float,
) -> float:
    """Return box-side grasp height in the base_actuator frame.

    The world ground plane is z=0 and base_actuator is ``base_height_m`` above
    it.  ratio=0.5 therefore targets the centre of a 95 mm-high box:
    0.095 * 0.5 - 0.350 = -0.3025 m.
    """
    values = (base_height_m, box_height_m, grasp_height_ratio)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Fixed grasp geometry values must be finite.")
    if base_height_m < 0.0 or box_height_m <= 0.0:
        raise ValueError("base_height_m must be non-negative and box_height_m positive.")
    if not 0.0 <= grasp_height_ratio <= 1.0:
        raise ValueError("box_grasp_height_ratio must be in [0, 1].")
    return float(box_height_m) * float(grasp_height_ratio) - float(base_height_m)


def supplybox_origin_to_tcp_target(
    supplybox_origin,
    tcp_offset_z: float,
    fixed_target_z: float | None = None,
):
    """Create a TCP goal using detected X/Y and detected or fixed Z.

    ``supplybox_origin`` is already expressed in ``base_actuator``.  This is
    deliberately separate from camera extrinsics and the robot wrist->TCP TF.
    Summer operation supplies ``fixed_target_z`` so depth/TF Z noise cannot
    create an unreachable downward goal. ``tcp_offset_z`` remains a small,
    explicit calibration adjustment and is never used for the 47.5 mm box
    half-height calculation itself.
    """
    target_z = (
        float(supplybox_origin[2])
        if fixed_target_z is None
        else float(fixed_target_z)
    )
    return (
        float(supplybox_origin[0]),
        float(supplybox_origin[1]),
        target_z + float(tcp_offset_z),
    )


class IKNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_node")
        group = ReentrantCallbackGroup()
        parameters = (
            ("target_topic", "/arm/target_point"),
            ("planning_frame", "base_actuator"),
            ("arm_duration_sec", 3.0),
            ("gripper_duration_sec", 3.0),
            ("joint_position_tolerance_rad", 0.03),
            # Flat-ground summer mission: use detection for X/Y, but derive Z
            # from physical geometry. 95 mm * 0.5 - 350 mm = -302.5 mm.
            ("use_fixed_target_z", True),
            ("base_height_m", DEFAULT_BASE_HEIGHT_M),
            ("box_height_m", DEFAULT_SUPPLY_BOX_HEIGHT_M),
            ("box_grasp_height_ratio", DEFAULT_BOX_GRASP_HEIGHT_RATIO),
            ("detected_z_warning_threshold_m", 0.05),
            ("supplybox_tcp_offset_z", DEFAULT_TCP_CALIBRATION_OFFSET_Z_M),
            # 3.35는 수정된 optical TF로 복원한 실제 bag 목표에서 유일하게
            # 확인된 유효 pitch라 먼저 시도한다. 3.16236은 기존 SID 실측값.
            ("approach_pitches", [3.35, 3.16236, 2.95, 3.05, 2.85, 3.15, 2.75, 3.25, 2.65]),
            ("ik_request_timeout_sec", 0.5),
            ("ik_avoid_collisions", True),
            ("settle_before_close_sec", 2.0),
            ("gripper_current_threshold_ma", 125.0),
            ("gripper_current_lsb_ma", 2.69),
            ("picking_topic", "/picking"),
            ("grasp_success_topic", "/arm/grasp_success"),
            ("calculation_failure_topic", "/arm/calculation_failed"),
            ("picking_command_topic", "/arm/picking_command"),
            ("target_point_base_topic", "/arm/target_point_base"),
            ("target_distance_topic", "/arm/target_distance_m"),
            ("manual_override_topic", "/control/arm_manual_override"),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.arm_duration = max(0.1, float(self.get_parameter("arm_duration_sec").value))
        self.gripper_duration = max(
            0.1, float(self.get_parameter("gripper_duration_sec").value))
        self.joint_tolerance = max(
            0.001, float(self.get_parameter("joint_position_tolerance_rad").value))
        self.use_fixed_target_z = bool(
            self.get_parameter("use_fixed_target_z").value)
        self.fixed_target_z = fixed_grasp_z_from_geometry(
            float(self.get_parameter("base_height_m").value),
            float(self.get_parameter("box_height_m").value),
            float(self.get_parameter("box_grasp_height_ratio").value),
        )
        self.detected_z_warning_threshold = max(
            0.0, float(self.get_parameter("detected_z_warning_threshold_m").value))
        self.supplybox_tcp_offset_z = float(
            self.get_parameter("supplybox_tcp_offset_z").value)
        self.approach_pitches = [
            float(v) for v in self.get_parameter("approach_pitches").value]
        self.ik_timeout = max(
            0.01, float(self.get_parameter("ik_request_timeout_sec").value))
        self.ik_avoid_collisions = bool(
            self.get_parameter("ik_avoid_collisions").value)
        self.settle_before_close_sec = max(
            0.0, float(self.get_parameter("settle_before_close_sec").value))
        self.current_threshold = float(
            self.get_parameter("gripper_current_threshold_ma").value)
        self.current_lsb = float(self.get_parameter("gripper_current_lsb_ma").value)

        self._calibration = JointCalibration()
        self._arm_positions: dict[str, float] = {}
        self._gripper_current_ma = 0.0
        self._state_lock = threading.RLock()
        self._startup_complete = False
        # [추가, 2026-09-05] 파지 성공(hold 도달 + picking_command 발행) 후 True.
        # 미션당 박스 하나이므로 이후 들어오는 좌표는 전부 무시한다(박스를 문
        # 채로 다시 내려가는 것 방지). 노드를 재시작해야 다시 받는다.
        self._grasp_done = False
        self._busy = False
        self._manual_override = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=group)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory", callback_group=group)
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory", callback_group=group)

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            PointStamped, self.target_topic, self._on_target, 10,
            callback_group=group)
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, str(self.get_parameter("manual_override_topic").value),
            self._on_manual_override, latched_qos, callback_group=group)
        self.picking_pub = self.create_publisher(
            Bool, str(self.get_parameter("picking_topic").value), latched_qos)
        self.grasp_success_pub = self.create_publisher(
            Bool, str(self.get_parameter("grasp_success_topic").value), 10)
        self.failure_pub = self.create_publisher(
            Empty, str(self.get_parameter("calculation_failure_topic").value), 10)
        self.picking_command_pub = self.create_publisher(
            Empty, str(self.get_parameter("picking_command_topic").value), 10)
        self.target_base_pub = self.create_publisher(
            PointStamped, str(self.get_parameter("target_point_base_topic").value), 10)
        self.target_distance_pub = self.create_publisher(
            Float64, str(self.get_parameter("target_distance_topic").value), 10)
        self.picking_pub.publish(Bool(data=False))

        threading.Thread(target=self._startup_worker, daemon=True).start()
        target_z_description = (
            f"fixed {self.fixed_target_z + self.supplybox_tcp_offset_z:.4f}m"
            if self.use_fixed_target_z else "detected"
        )
        self.get_logger().info(
            "Minimal flow ready: startup grasp_wait -> one target IK move -> "
            f"settle {self.settle_before_close_sec:.1f}s -> "
            f"full gripper close to {GRIPPER_CLOSED:.4f}rad "
            f"-> hold -> home -> picking_command -> bed. target Z={target_z_description}.")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._state_lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position) and math.isfinite(msg.position[index]):
                    if name in ARM_JOINT_NAMES:
                        self._arm_positions[name] = float(msg.position[index])
                if name == "gripper_joint" and index < len(msg.effort):
                    if math.isfinite(msg.effort[index]):
                        self._gripper_current_ma = abs(float(msg.effort[index])) * self.current_lsb

    def _on_manual_override(self, msg: Bool) -> None:
        with self._state_lock:
            self._manual_override = bool(msg.data)

    def _on_target(self, msg: PointStamped) -> None:
        with self._state_lock:
            if self._grasp_done:
                self.get_logger().info(
                    "Grasp already completed; ignoring new target.",
                    throttle_duration_sec=10.0)
                return
            if not self._startup_complete or self._busy or self._manual_override:
                return
            self._busy = True
        try:
            supplybox_origin = self._transform_supplybox_origin(msg)
            target = supplybox_origin_to_tcp_target(
                supplybox_origin,
                self.supplybox_tcp_offset_z,
                self.fixed_target_z if self.use_fixed_target_z else None,
            )
        except Exception as exc:
            self.get_logger().error(f"Target TF failed: {exc}")
            self.failure_pub.publish(Empty())
            with self._state_lock:
                self._busy = False
            return

        if (
            self.use_fixed_target_z
            and self.detected_z_warning_threshold > 0.0
            and abs(supplybox_origin[2] - target[2])
            > self.detected_z_warning_threshold
        ):
            self.get_logger().warn(
                "Detected Z differs from the geometry-fixed TCP Z by "
                f"{abs(supplybox_origin[2] - target[2]):.3f}m "
                f"(detected={supplybox_origin[2]:.3f}, fixed={target[2]:.3f}); "
                "using fixed Z. Check camera TF/calibration if this persists.")

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.planning_frame
        point.point.x, point.point.y, point.point.z = target
        self.target_base_pub.publish(point)
        self.target_distance_pub.publish(
            Float64(data=math.dist((0.0, 0.0, 0.0), target)))
        self.picking_pub.publish(Bool(data=True))
        self.get_logger().info(
            "Target accepted once: "
            f"supplybox origin=({supplybox_origin[0]:.3f}, "
            f"{supplybox_origin[1]:.3f}, {supplybox_origin[2]:.3f}), "
            f"TCP target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}).")
        threading.Thread(target=self._grasp_worker, args=(target,), daemon=True).start()

    def _transform_supplybox_origin(self, msg: PointStamped) -> tuple[float, float, float]:
        """Transform the detected top-surface origin without grasp offsets."""
        if msg.header.frame_id and msg.header.frame_id != self.planning_frame:
            transform_time = (
                Time.from_msg(msg.header.stamp)
                if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
                else Time()
            )
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, msg.header.frame_id, transform_time,
                timeout=Duration(seconds=0.5))
            transformed = do_transform_point(msg, transform).point
            xyz = (transformed.x, transformed.y, transformed.z)
        else:
            xyz = (msg.point.x, msg.point.y, msg.point.z)
        return tuple(float(value) for value in xyz)

    def _startup_worker(self) -> None:
        preset = GRASP_WAIT_ARM_JOINTS
        gripper_opened = False
        while rclpy.ok():
            with self._state_lock:
                manual_override = self._manual_override
            if manual_override:
                time.sleep(0.1)
                continue
            # 그리퍼 open은 팔 도달 여부와 무관하게 startup 맨 처음에 먼저 연다.
            # 팔이 grasp_wait에 못 가더라도(손목 과전류 latch 등) 그리퍼는 열려 있어야 한다.
            if not gripper_opened:
                if self._send_trajectory(
                        self.gripper_client, GRIPPER_JOINT_NAMES, [GRIPPER_OPEN],
                        self.gripper_duration, "startup gripper open"):
                    gripper_opened = True
                    self.get_logger().info("Startup gripper open done.")
                else:
                    self.get_logger().warn("Startup gripper open failed; retrying in 1s.")
                    time.sleep(1.0)
                    continue
            if not self.arm_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(
                    "Waiting for active arm_controller before startup grasp_wait...",
                    throttle_duration_sec=5.0)
                continue
            with self._state_lock:
                ready = all(name in self._arm_positions for name in ARM_JOINT_NAMES)
                base = self._arm_positions.get("base_joint", 0.0)
            if not ready:
                time.sleep(0.1)
                continue
            target = [base if name == "base_joint" else preset[name]
                      for name in ARM_JOINT_NAMES]
            self.get_logger().info(f"Startup command immediately -> grasp_wait: {target}")
            if not self._send_arm(target, "startup grasp_wait"):
                self.get_logger().warn("Startup grasp_wait failed; retrying in 1s.")
                time.sleep(1.0)
                continue
            with self._state_lock:
                self._startup_complete = True
            self.get_logger().info(
                "Startup grasp_wait reached with gripper open; accepting supplybox targets.")
            return

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _send_trajectory(self, client, names, positions, duration, label) -> bool:
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{label}: controller action unavailable.")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        nanoseconds = int(round(duration * 1_000_000_000))
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        goal.trajectory.points = [point]
        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, 5.0):
            self.get_logger().error(f"{label}: goal response timeout.")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected.")
            return False
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, duration + 8.0):
            handle.cancel_goal_async()
            self.get_logger().error(f"{label}: execution timeout.")
            return False
        wrapped = result_future.result()
        ok = (
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not ok:
            self.get_logger().error(f"{label}: controller execution failed.")
        return ok

    def _arm_reached(self, target) -> bool:
        deadline = time.monotonic() + 2.0
        errors = [math.inf] * len(ARM_JOINT_NAMES)
        while rclpy.ok() and time.monotonic() < deadline:
            with self._state_lock:
                errors = [
                    abs(self._arm_positions.get(name, math.inf) - expected)
                    for name, expected in zip(ARM_JOINT_NAMES, target)
                ]
            if max(errors) <= self.joint_tolerance:
                return True
            time.sleep(0.05)
        self.get_logger().error(f"Measured joints did not reach target: errors={errors}")
        return False

    def _send_arm(self, target, label, *, require_reach: bool = True) -> bool:
        """arm_controller에 trajectory를 보내고 완료를 기다린다.

        require_reach=True(기본)면 측정 관절이 tolerance 안에 들어와야 성공이다.
        [갱신, 2026-09-05] require_reach=False면 컨트롤러가 trajectory를 완주한
        것만으로 성공 처리하고, 도달 오차는 경고 로그만 남긴다 - IK 이동 후
        도달 검사에 걸려 시퀀스가 중단되고 다음 좌표로 다시 움직이는 대신,
        바로 그리퍼를 닫게 하기 위한 용도.
        """
        with self._state_lock:
            current = [self._arm_positions.get(name) for name in ARM_JOINT_NAMES]
        duration = required_arm_duration(current, target, self.arm_duration)
        if duration > self.arm_duration + 1e-6:
            self.get_logger().info(
                f"{label}: extending trajectory {self.arm_duration:.1f}s -> "
                f"{duration:.1f}s for configured RMD velocity limits.")
        if not self._send_trajectory(
                self.arm_client, ARM_JOINT_NAMES, target, duration, label):
            return False
        reached = self._arm_reached(target)
        if not reached and not require_reach:
            self.get_logger().warn(
                f"{label}: controller finished but joints are outside tolerance; "
                "continuing anyway (require_reach=False).")
            return True
        return reached

    def _solve_ik(self, target_tcp, *, log_failure: bool = True):
        if not self.ik_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error("/compute_ik service unavailable.")
            return None
        with self._state_lock:
            current = [self._arm_positions.get(name) for name in ARM_JOINT_NAMES]
        if any(value is None for value in current):
            return None
        base_angle = compute_base_angle(target_tcp[0], target_tcp[1])
        radius = math.hypot(target_tcp[0], target_tcp[1])
        seed_name, _, _, distance_joints = select_distance_seed(radius)
        distance = [base_angle, *distance_joints[1:]]
        distance_pitch = sum(float(value) for value in distance_joints[1:])
        sid = [base_angle, 1.71566, 0.62151, 0.82519]
        ground = [base_angle, *GROUND_GRASP_IK_SEED[1:]]
        current_pitch = sum(float(value) for value in current[1:])
        # 거리 seed가 검증된 pitch를 먼저 시도해야 1회에 풀린다.
        pitches = [distance_pitch, current_pitch, *self.approach_pitches]
        pitches = list(dict.fromkeys(round(value, 8) for value in pitches))
        self.get_logger().info(
            f"IK radius={radius:.3f}m -> distance seed '{seed_name}' first.")
        for pitch in pitches:
            orientation = quaternion_from_base_pitch(base_angle, pitch)
            offset = rotate_vector_by_quaternion(
                orientation, WRIST_TO_TCP_LOCAL_OFFSET)
            wrist_target = tuple(target_tcp[i] - offset[i] for i in range(3))
            # distance는 목표 반경 구간에서 검증된 여름 파지 해, current는
            # 가까운 연속 목표, ground는 아래쪽 목표, SID는 기존 실측 파지
            # 자세를 각각 대표한다. 실패 시에만 뒤 seed로 넘어간다.
            for seed in (distance, current, ground, sid):
                req = GetPositionIK.Request()
                req.ik_request = PositionIKRequest()
                req.ik_request.group_name = "arm"
                req.ik_request.pose_stamped = PoseStamped()
                req.ik_request.pose_stamped.header.frame_id = self.planning_frame
                pose = req.ik_request.pose_stamped.pose
                pose.position.x, pose.position.y, pose.position.z = wrist_target
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
                req.ik_request.avoid_collisions = self.ik_avoid_collisions
                timeout_ns = int(round(self.ik_timeout * 1_000_000_000))
                req.ik_request.timeout.sec = timeout_ns // 1_000_000_000
                req.ik_request.timeout.nanosec = timeout_ns % 1_000_000_000
                state = RobotStateMsg()
                state.joint_state.name = list(ARM_JOINT_NAMES)
                state.joint_state.position = list(seed)
                state.is_diff = False
                req.ik_request.robot_state = state
                future = self.ik_client.call_async(req)
                if not self._wait_future(future, self.ik_timeout + 1.0):
                    continue
                response = future.result()
                if response is None or response.error_code.val != 1:
                    if (
                        response is not None
                        and response.error_code.val in IK_COLLISION_ERROR_CODES
                    ):
                        self.get_logger().warn(
                            "IK candidate rejected by collision checking.")
                    continue
                by_name = dict(zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position))
                solution = [float(by_name[name]) for name in ARM_JOINT_NAMES]
                valid = all(
                    self._calibration.actual_limits(name)[0] <= value
                    <= self._calibration.actual_limits(name)[1]
                    for name, value in zip(ARM_JOINT_NAMES, solution)
                )
                if valid:
                    self.get_logger().info(
                        f"IK solved with pitch={pitch:.2f}: {solution}")
                    return solution
        if log_failure:
            self.get_logger().error("No IK solution for the detected supplybox target.")
        return None

    def _close_gripper_full(self) -> bool:
        """[갱신, 2026-09-05] 전류 기준 조기 취소 없이 GRIPPER_CLOSED까지 끝까지
        닫는다. 실기에서 전류 threshold(125mA)에 너무 일찍 걸려 그리퍼가 조금만
        닫히고 파지 판정이 나는 문제가 있어서, 닫기 trajectory가 완주되면 그것을
        파지 성공으로 본다. 아래 _close_until_current()는 참고용으로 남겨둔다."""
        if not self._send_trajectory(
                self.gripper_client, GRIPPER_JOINT_NAMES, [GRIPPER_CLOSED],
                self.gripper_duration, "gripper close"):
            self.grasp_success_pub.publish(Bool(data=False))
            self.get_logger().warn("Gripper close trajectory failed; grasp failed.")
            return False
        with self._state_lock:
            final_current = self._gripper_current_ma
        self.get_logger().info(
            f"Gripper fully closed to {GRIPPER_CLOSED:.4f}rad "
            f"(current {final_current:.1f}mA).")
        self.grasp_success_pub.publish(Bool(data=True))
        return True

    def _close_until_current(self) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(GRIPPER_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_CLOSED]
        duration_ns = int(round(self.gripper_duration * 1_000_000_000))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        goal.trajectory.points = [point]
        with self._state_lock:
            self._gripper_current_ma = 0.0
        send_future = self.gripper_client.send_goal_async(goal)
        if not self._wait_future(send_future, 5.0):
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.gripper_duration + 5.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            with self._state_lock:
                current = self._gripper_current_ma
            if current >= self.current_threshold:
                cancel_future = handle.cancel_goal_async()
                cancel_done = self._wait_future(cancel_future, 2.0)
                result_done = self._wait_future(result_future, 2.0)
                if not cancel_done or not result_done:
                    self.get_logger().error(
                        "Gripper current reached, but controller cancellation did not finish; "
                        "HOLD is withheld to avoid overlapping trajectory goals.")
                    return False
                self.get_logger().info(
                    f"Grasp current reached: {current:.1f}mA >= "
                    f"{self.current_threshold:.1f}mA.")
                self.grasp_success_pub.publish(Bool(data=True))
                return True
            time.sleep(0.02)
        with self._state_lock:
            final_current = self._gripper_current_ma
        if final_current >= self.current_threshold:
            self.get_logger().info(
                f"Grasp current reached: {final_current:.1f}mA >= "
                f"{self.current_threshold:.1f}mA.")
            self.grasp_success_pub.publish(Bool(data=True))
            return True
        self.grasp_success_pub.publish(Bool(data=False))
        self.get_logger().warn(
            f"Gripper closed without reaching {self.current_threshold:.1f}mA; grasp failed.")
        return False

    def _grasp_worker(self, target) -> None:
        success = False
        try:
            solution = self._solve_ik(target)
            if solution is None:
                self.get_logger().warn(
                    "Exact target IK failed; holding current pose (partial reach disabled).")
                self.failure_pub.publish(Empty())
                return
            # [갱신, 2026-09-05] IK 이동은 컨트롤러 완주만 확인하고 바로 닫는다.
            # 도달 오차로 시퀀스를 끊고 새 좌표를 받아 다시 움직이지 않는다.
            if not self._send_arm(solution, "detected target", require_reach=False):
                self.failure_pub.publish(Empty())
                return
            self.get_logger().info(
                f"IK move finished; ignoring new targets and holding still for "
                f"{self.settle_before_close_sec:.1f}s before gripper close.")
            if self.settle_before_close_sec > 0.0:
                time.sleep(self.settle_before_close_sec)
            if not self._close_gripper_full():
                return
            with self._state_lock:
                hold = [
                    HOLD_ARM_JOINTS.get(name, self._arm_positions[name])
                    for name in ARM_JOINT_NAMES
                ]
            # [갱신, 2026-09-05] hold/home/bed 모두 컨트롤러 완주면 다음 단계로
            # 간다(도달 오차로 실패 처리하지 않음). hold까지 오면 박스는 이미
            # 들려 있으므로 그 뒤 단계가 실패해도 새 좌표는 받지 않는다.
            if not self._send_arm(hold, "hold", require_reach=False):
                self.failure_pub.publish(Empty())
                return
            with self._state_lock:
                self._grasp_done = True
            self.get_logger().info("HOLD complete -> HOME.")
            # [추가, 2026-09-05] 수납: hold -> home -> bed. home은 이미 접힌
            # 자세라 여기서 picking_command를 내 구동부를 출발시키고, 대기 없이
            # 곧바로 bed(주행용 눕는 자세)로 간다 - 즉 bed 이동은 주행 중에 한다.
            with self._state_lock:
                home = [HOME_ARM_JOINTS.get(name, self._arm_positions[name])
                        for name in ARM_JOINT_NAMES]
            if not self._send_arm(home, "home", require_reach=False):
                self.get_logger().error(
                    "home move failed after grasp; arm stays at hold. "
                    "picking_command withheld.")
                self.failure_pub.publish(Empty())
                return
            self.picking_command_pub.publish(Empty())
            self.get_logger().info(
                "HOME complete; picking_command published -> BED (while driving).")
            with self._state_lock:
                bed = [BED_ARM_JOINTS.get(name, self._arm_positions[name])
                       for name in ARM_JOINT_NAMES]
            if self._send_arm(bed, "bed", require_reach=False):
                self.get_logger().info("BED complete. Further targets are ignored.")
            else:
                # 파지/출발은 이미 끝났으므로 미션에는 영향 없음 - 경고만.
                self.get_logger().warn(
                    "bed move failed; arm stays between home and bed. "
                    "Mission continues (picking_command already sent).")
            success = True
        except Exception as exc:
            self.get_logger().error(
                f"Simple grasp sequence exception: {type(exc).__name__}: {exc}")
            self.failure_pub.publish(Empty())
        finally:
            self.picking_pub.publish(Bool(data=False))
            with self._state_lock:
                self._busy = False
            if not success:
                self.get_logger().warn("Simple grasp attempt ended; waiting for a new target.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
