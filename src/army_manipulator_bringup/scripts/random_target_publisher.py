#!/usr/bin/env python3
"""임의(random) 좌표를 /arm/target_point로 발행하는 테스트용 노드.

fake_target_publisher.py는 매번 같은 고정 좌표를 쏘는 반면, 이 노드는 지정된
범위 안에서 매번 다른 좌표를 뽑아 발행한다 - "카메라로 매번 다른 위치에서
물체를 검출했을 때" 다운스트림(maru_ik_node의 IK 역산 -> arm_controller 실행
-> 정지 -> 그리퍼 파지 -> 홈 복귀)이 여러 목표에서 반복적으로 잘 도는지
확인할 때 쓴다.

[수정] 범위 안에서 그냥 무작위로 뽑으면, 구동부 형상/관절 리미트 때문에 IK
자체가 안 풀리는 좌표가 섞여 나온다. 그래서 좌표를 뽑을 때마다 `/compute_ik`
서비스로 maru_ik_node가 실제로 요청할 pregrasp/descend pose를 그대로
검증(충돌 포함)해서, 진짜 성공할 좌표만 골라 publish한다(rejection sampling)
- 도달가능 영역을 미리 기하학적으로 계산하는 대신, 실제 솔버를 오라클로
쓰는 방식이라 팔 형상이 바뀌어도 코드를 안 고쳐도 된다.

[중요] compute_ik의 KDL 솔버는 지역 수치해법이라 seed(시작 관절각)에 따라
같은 목표도 풀리기도 안 풀리기도 한다 - 실측으로 확인: `is_diff=True`(현재
관절상태를 seed로 사용)로는 SRDF의 "home" 자세조차 실패하는데, 동일 목표를
zero-seed나 home-seed로 명시하면 성공한다. 그래서 후보 좌표 하나당 여러
seed(zero, home, grip_wait)를 다 시도해보고 "하나라도 성공하면 도달 가능"
으로 판정한다 - 이게 OMPL이 goal state를 샘플링할 때 하는 방식과 같다.
현재 상태 하나만 seed로 쓰면 실제로는 도달 가능한 좌표까지 대량으로
오탈락시킨다.

바닥 배치: base_link 원점이 바닥에서 base_height_m 위에 있을 때, 박스(90mm)
+ 핑거 길이(80mm) 조합으로 TCP는 지면 위 tcp_height_above_ground_m(기본
20mm)를 목표하면 된다고 실측 확인됨 - 그래서 Y(수직)는 랜덤이 아니라
(base_height_m - tcp_height_above_ground_m)로 고정하고, X(좌우)/Z(전방)만
지정된 범위 안에서 뽑는다.
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState as RobotStateMsg
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
# SRDF group_state들 - compute_ik의 KDL(지역 수치해법)이 seed에 따라 같은
# 목표도 성공/실패가 갈리므로, 여러 곳에서 출발시켜 "하나라도 성공하면
# 도달 가능"으로 판정한다.
IK_SEEDS = [
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 1.027, -1.2584, -1.1716],       # home
    [-0.3037, -0.7087, -1.3741, -0.998],  # grip_wait
    [0.0, 1.71548, 1.58825, 1.51844],  # AUTO storage home - maru_ik_node.py의 IK_SEEDS와
    # 반드시 맞춰야 한다: 실제 AUTO 시퀀스는 _move_to_home() 직후 상태에서
    # 시작하는데, 이 seed가 검증 목록에 없으면 "도달 가능"으로 판정해서
    # publish했는데 실제로는 maru_ik_node에서 실패하는 불일치가 생긴다
    # (2026-08-27 실기 테스트로 확인됨).
]


# [수정] maru_ik_node.py와 동일한 발견/공식 - army_manipulator_macro.xacro가
# shoulder/elbow/wrist를 axis=X에서 axis=Z로 재구성한 뒤로 base_joint의 실질
# 회전축이 base_link 기준 Y가 아니라 Z(전방)임을 compute_fk 실측으로 확인함.
# base_joint를 스윕하면 z는 고정된 채 x,y가 함께 회전하고(base=0일 때
# wrist_link의 y는 shoulder/elbow/wrist와 무관하게 상수 Y0≈0.0313m), wrist_link
# orientation은 shoulder+elbow+wrist "합"(pitch)에만 의존한다(base=0/pitch=0에서
# 정확히 Q0=(0.5,0.5,-0.5,0.5)). 자세한 유도는 maru_ik_node.py 참고 - 두 파일이
# 정확히 같은 공식을 써야 rejection sampling이 실제 요청과 일치한다.
BASE_ROTATION_Y_OFFSET_M = 0.0313


def compute_base_angle(target_x: float, target_y: float) -> float:
    radius = math.hypot(target_x, target_y)
    x0 = math.sqrt(max(radius * radius - BASE_ROTATION_Y_OFFSET_M ** 2, 0.0))
    return math.atan2(target_y, target_x) - math.atan2(BASE_ROTATION_Y_OFFSET_M, x0)


def quaternion_from_base_pitch(base_angle: float, pitch: float):
    cb, sb = math.cos(base_angle / 2.0), math.sin(base_angle / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    p = 0.5 * (cp + sp)
    q = 0.5 * (cp - sp)
    return (cb * p - sb * q, sb * p + cb * q, sb * p - cb * q, sb * q + cb * p)


class RandomTargetPublisher(Node):
    def __init__(self):
        super().__init__("random_target_publisher")

        self.declare_parameter("target_topic", "/arm/target_point")
        self.declare_parameter("frame_id", "")
        # [수정] maru_ik_node.py와 동일한 이유로 base_link -> base_actuator.
        # base_link는 base_joint의 자식(=회전 프레임 자신)이라 planning_frame으로
        # 쓰면 안 된다 - SRDF arm 체인의 실제 고정 루트는 base_actuator다.
        self.declare_parameter("planning_frame", "base_actuator")
        self.declare_parameter("group_name", "arm")

        # 바닥/박스/그리퍼 치수. [수정] 박스 상단면 높이(base_height-box_size)
        # 대신 실측 기준값을 그대로 쓴다: 박스 90mm + 핑거(finger) 길이 80mm
        # 조합으로 TCP는 지면 위 20mm를 목표하면 된다고 확인됨 - box_size_m/
        # finger_length_m은 그 계산의 근거로 남겨두고, 실제 Y 고정값은
        # tcp_height_above_ground_m을 직접 쓴다.
        self.declare_parameter("base_height_m", 0.350)
        self.declare_parameter("box_size_m", 0.090)
        self.declare_parameter("finger_length_m", 0.080)
        self.declare_parameter("tcp_height_above_ground_m", 0.020)

        # X(좌우)/Z(전방) 탐색 범위. Y는 위 두 값으로부터 고정 계산.
        # [수정] planning_frame을 base_actuator로 고친 뒤(회전하는 base_link를
        # 기준으로 쓰던 버그 수정, 위 planning_frame 주석 참고) 다시 실측한
        # 결과: z_max=0.32는 x=0에서만 유효했다. x가 x_min/x_max(±0.15) 끝으로
        # 갈수록 도달 가능한 z 상한이 줄어들어서(x=±0.15에서 seed 4개x pitch
        # 7개 전부 동원해도 z=0.26은 성공, z=0.28은 완전 실패로 확인) 기존
        # 0.32는 x 범위 양 끝에서 안전하지 않다. x_min~x_max 전체에서 검증된
        # z_max=0.26으로 낮췄다. z_min=0.15는 x=±0.15에서도 충분히 여유
        # 있게 도달 가능함을 확인해서 그대로 둔다. (rejection sampling인
        # pose_ok()가 개별 후보를 다시 검증하므로 이 범위를 벗어나는 후보가
        # 나가지는 않지만, 범위 자체를 넓게/타이트하게 잘못 잡으면 재시도
        # 낭비가 커진다.)
        self.declare_parameter("x_min", -0.15)
        self.declare_parameter("x_max", 0.15)
        self.declare_parameter("z_min", 0.15)
        self.declare_parameter("z_max", 0.26)

        # maru_ik_node.py와 동일한 기본값 - rejection sampling이 실제 요청과
        # 같은 pose를 검증하도록 맞춘다. maru_ik_node 쪽 파라미터를 바꾸면
        # 여기도 같이 맞춰야 한다.
        self.declare_parameter("grasp_offset_x", 0.0)
        self.declare_parameter("grasp_offset_y", 0.0)
        self.declare_parameter("grasp_offset_z", -0.0475)
        self.declare_parameter("pregrasp_offset_z", 0.10)
        # [수정] maru_ik_node.py와 동일 - 축 재구성 후 z에 따라 필요한 pitch가
        # -3.1~-2.5 범위에서 갈림(compute_ik 실측). 하나로 안 풀리면 다음
        # 후보로 넘어간다.
        self.declare_parameter("approach_pitch", -2.5)
        self.declare_parameter("approach_pitch_fallbacks", [-3.1, -2.8, -2.0, -1.57, -1.2, -0.9])

        self.declare_parameter("max_attempts", 200)
        self.declare_parameter("delay_sec", 3.0)
        self.declare_parameter("repeat", False)
        self.declare_parameter("rate_hz", 0.1)
        self.declare_parameter("seed", 0)  # 0이면 랜덤, 그 외 값이면 재현 가능한 고정 시드

        self.target_topic = self.get_parameter("target_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.group_name = str(self.get_parameter("group_name").value)

        self.base_height_m = float(self.get_parameter("base_height_m").value)
        self.tcp_height_above_ground_m = float(
            self.get_parameter("tcp_height_above_ground_m").value)
        # TCP가 지면 위 tcp_height_above_ground_m을 목표하도록 Y 고정.
        # "Y=수직(하단 +)" 관례라 base_link보다 아래일수록 Y가 커진다.
        self.y_fixed = self.base_height_m - self.tcp_height_above_ground_m

        self.x_range = (
            float(self.get_parameter("x_min").value), float(self.get_parameter("x_max").value))
        self.z_range = (
            float(self.get_parameter("z_min").value), float(self.get_parameter("z_max").value))

        self.grasp_offset = (
            float(self.get_parameter("grasp_offset_x").value),
            float(self.get_parameter("grasp_offset_y").value),
            float(self.get_parameter("grasp_offset_z").value),
        )
        self.pregrasp_offset_z = float(self.get_parameter("pregrasp_offset_z").value)
        self.approach_pitch = float(self.get_parameter("approach_pitch").value)
        self.approach_pitch_fallbacks = [
            float(v) for v in self.get_parameter("approach_pitch_fallbacks").value]
        self.max_attempts = int(self.get_parameter("max_attempts").value)

        self.delay_sec = float(self.get_parameter("delay_sec").value)
        self.repeat = bool(self.get_parameter("repeat").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        seed = int(self.get_parameter("seed").value)

        self._rng = random.Random(seed if seed != 0 else None)
        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.get_logger().info(
            f"random_target_publisher ready. x={self.x_range} y_fixed={self.y_fixed:.4f} "
            f"(base_height={self.base_height_m}, tcp_height_above_ground={self.tcp_height_above_ground_m}) "
            f"z={self.z_range} -> {self.target_topic} in {self.delay_sec}s (repeat={self.repeat})"
        )

    def run(self) -> None:
        """rclpy.spin()을 쓰지 않고 직접 순차 실행한다 - publish_target() 내부에서
        /compute_ik 검증할 때마다 rclpy.spin_until_future_complete()를 쓰는데,
        이걸 spin() 콜백 안에서 중첩 호출하면 같은 노드의 executor가 겹쳐서
        멎어버린다(실측으로 확인됨)."""
        import time
        time.sleep(self.delay_sec)
        self.publish_target()
        if self.repeat:
            period = 1.0 / self.rate_hz
            while rclpy.ok():
                time.sleep(period)
                self.publish_target()

    def _compute_ik_ok(self, x: float, y: float, z: float, quat) -> bool:
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/compute_ik service not available; cannot validate reachability.")
            return False
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.planning_frame
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.x = quat[0]
        req.ik_request.pose_stamped.pose.orientation.y = quat[1]
        req.ik_request.pose_stamped.pose.orientation.z = quat[2]
        req.ik_request.pose_stamped.pose.orientation.w = quat[3]
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1

        # [중요] KDL은 지역 수치해법이라 seed(시작 관절각)에 따라 같은 목표도
        # 성공/실패가 갈린다 - "현재 상태"(is_diff=True) 하나만 seed로 쓰면
        # 실제로는 도달 가능한 좌표까지 대량 오탈락시킨다(실측 확인:
        # 현재상태 seed로는 SRDF "home" 자세조차 실패, zero/home seed로
        # 명시하면 성공). 그래서 여러 seed를 순서대로 시도해서 하나라도
        # 성공하면 도달 가능으로 판정한다(OMPL의 goal 샘플링과 동일한 발상).
        for seed in IK_SEEDS:
            js = JointState()
            js.name = list(ARM_JOINT_NAMES)
            js.position = list(seed)
            rs = RobotStateMsg()
            rs.joint_state = js
            rs.is_diff = False
            req.ik_request.robot_state = rs

            future = self._ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            res = future.result()
            if res is not None and res.error_code.val == 1:
                return True
        return False

    def _is_reachable(self, x: float, y: float, z: float) -> bool:
        """maru_ik_node.py의 pregrasp/descend 요청을 그대로 흉내내서 둘 다
        검증한다 - 하나라도 실패하면 실제 그랩 시퀀스도 거기서 막힌다.
        maru_ik_node.py의 _move_arm과 동일하게, pregrasp/descend가 서로 다른
        z를 목표하고 pitch 유효 범위가 z에 따라 갈리므로 후보 pitch를 순서대로
        시도해서 하나라도 성공하면 그 지점은 도달 가능으로 판정한다."""
        gx, gy, gz = self.grasp_offset
        gx_pt, gy_pt, gz_pt = x + gx, y + gy, z + gz
        base_angle = compute_base_angle(gx_pt, gy_pt)
        pitches = (self.approach_pitch, *self.approach_pitch_fallbacks)

        def pose_ok(px, py, pz) -> bool:
            return any(
                self._compute_ik_ok(px, py, pz, quaternion_from_base_pitch(base_angle, pitch))
                for pitch in pitches
            )

        if not pose_ok(gx_pt, gy_pt, gz_pt + self.pregrasp_offset_z):
            return False
        return pose_ok(gx_pt, gy_pt, gz_pt)

    def publish_target(self) -> None:
        for attempt in range(1, self.max_attempts + 1):
            x = self._rng.uniform(*self.x_range)
            z = self._rng.uniform(*self.z_range)
            self.get_logger().info(f"  attempt {attempt}: trying x={x:.4f} z={z:.4f} ...")
            if self._is_reachable(x, self.y_fixed, z):
                msg = PointStamped()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = x
                msg.point.y = self.y_fixed
                msg.point.z = z
                self.target_pub.publish(msg)
                self.get_logger().info(
                    f"Published reachable random target ({x:.4f}, {self.y_fixed:.4f}, {z:.4f}) "
                    f"to {self.target_topic} (attempt {attempt}/{self.max_attempts})"
                )
                return
        self.get_logger().error(
            f"{self.max_attempts}번 시도했지만 도달 가능한 좌표를 못 찾음 - "
            f"x_range/z_range를 좁히거나 IK/충돌 설정을 다시 확인할 것."
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomTargetPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
