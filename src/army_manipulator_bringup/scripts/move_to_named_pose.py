#!/usr/bin/env python3
"""arm_controller/gripper_controller(FollowJointTrajectory)에 직접 골을 보내
이름 붙은 자세로 이동하고/또는 그리퍼를 연다·닫는다.

[갱신, 2026-09-02] 원래는 pymoveit2 MoveIt2.move_to_configuration()으로
move_group을 거쳤으나(계획/충돌검사 수행), 실기에서 다음 두 문제가 확인돼
raw `ros2 action send_goal /<controller>/follow_joint_trajectory` 방식으로
되돌렸다:
  - control_bringup.launch.py(maru_ik_node + move_group을 같이 띄움)와
    동시에 이 스크립트를 실행하면, 이 스크립트와 maru_ik_node가 같은
    move_group 액션 서버(/move_action)를 동시에 점유하려다 충돌한다
    ("점유 문제").
  - move_group 없이 이 스크립트만 단독 실행하면 /move_action 서버가 없어서
    move_to_configuration()이 "Better luck next time" 경고만 내고 아무
    동작도 하지 않는다.
raw FollowJointTrajectory는 move_group을 아예 거치지 않고 ros2_control의
arm_controller/gripper_controller(JointTrajectoryController)에 직접
붙는다. 다만 maru_ik_node도 최종적으로 같은 controller를 사용하므로 그냥
동시에 명령하면 PREEMPTED가 다시 발생한다. 실행 중
/control/arm_manual_override=true heartbeat를 발행해 maru_ik_node의 자동
명령을 정지시키고, 종료 시 false로 복구한다. heartbeat가 끊기면 IK 노드가
lease timeout으로 자동 복구한다. 컨트롤러 두 개만 살아있으면 move_group
서버 없이도 동작한다. 대신 MoveIt의 충돌검사(SRDF
disable_collisions 포함)는 더 이상 안 받는다 - 여기 정의된 이름 붙은
자세들은 이미 실기에서 검증된 값이라 감수한다(gripper_safe_close.py와
동일한 절충).

관절값(HOME/HOLD/GRASP_WAIT)은 maru_ik_node.py와 동일한 값을 여기 다시
선언해뒀다 - 그쪽이 갱신되면(실측 재캘리브레이션 등) 여기도 같이 갱신할 것.

base_joint는 STAND만 0.0으로 고정, 나머지(HOME/HOLD/GRASP_WAIT)는
maru_ik_node.py와 동일하게 "현재 위치 유지"가 원래 동작이라 --base
인자로 직접 지정하게 했다(기본 0.0).

그리퍼 열기/닫기만 단독으로(또는 팔 이동과 같이) 테스트할 수 있게 gripper
파라미터를 제공한다. pose와 gripper를 둘 다 주면 팔 이동을 먼저 끝내고 그
다음 그리퍼를 움직인다(팔 이동이 실패하면 그리퍼는 시도하지 않는다).

[갱신, 2026-09-02] gripper:=close는 maru_ik_node.py
_close_gripper_with_current_feedback()과 동일한 전류 피드백 파지 판정을
쓴다 - 닫는 도중 gripper_joint 전류가 gripper_current_threshold_ma를
넘으면(물체에 부하 걸림) 즉시 취소하고 파지 성공으로 판정한다. 여기에
더해(사용자 요청) 전류 스파이크 없이 GRIPPER_CLOSED 근처까지 완전히
닫힌 경우도 OR 조건으로 성공 처리한다 - "물체를 못 집었다"보다는 "닫기
동작 자체는 정상 완료됐다"를 신호하려는 수동 테스트 도구 성격이라, 자동
파지 상태머신(maru_ik_node.py, MISS 처리)과는 성공 판정 기준이 다르다.
두 조건 중 하나라도 맞으면 grasp_success_topic(기본 /arm/grasp_success,
maru_ik_node.py와 동일 토픽)에 Bool(true)을 발행한다. gripper:=open은
그리퍼 물림 위험이 없어 이 판정 없이 기존처럼 단순 이동만 한다.

사용:
    ros2 run army_manipulator_bringup move_to_named_pose.py --ros-args \
        -p pose:=home
    # pose: stand | home | hold | grasp_wait | bed | sid (생략 가능 - gripper만 써도 됨)
    # -p base:=0.0 로 base_joint 값 지정 가능(기본 0.0) - sid를 캡처했을 때의
    # 실측 base_joint는 -0.02761이었으니 그대로 재현하려면
    # -p base:=-0.02761 을 같이 지정할 것.
    # -p gripper:=open  (또는 close) - 그리퍼만 단독으로, 또는 pose와 같이 지정 가능
    #   예: -p pose:=sid -p gripper:=open   (sid로 이동한 뒤 그리퍼 열기)
    # -p duration_sec:=3.0 로 이동/개폐에 걸리는 시간 지정 가능(팔/그리퍼 각각 적용,
    #   기본 3.0초 - `ros2 action send_goal .../follow_joint_trajectory`로 수동
    #   검증할 때 쓰던 값과 동일).
    # -p gripper_current_threshold_ma:=100.0 로 전류 피드백 파지 판정 임계값
    #   지정 가능(기본 100.0mA, maru_ik_node.py와 동일 실측값).
    # -p grasp_success_topic:=/arm/grasp_success 로 발행 토픽 변경 가능.
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty
from trajectory_msgs.msg import JointTrajectoryPoint

from joint_calibration import JointCalibration
from grasp_wait_presets import GRASP_WAIT_PRESETS

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
GRIPPER_JOINT_NAMES = ["gripper_joint"]
RMD_MAX_VELOCITY_RAD_S = {
    "shoulder_joint": math.radians(15.0),
    "elbow_joint": math.radians(5.0),
    "wrist_joint": math.radians(30.0),
}


def required_arm_duration(current_positions, target, requested_sec: float) -> float:
    """Do not make the trajectory reference outrun the real RMD actuators."""
    required = float(requested_sec)
    for name, goal in zip(ARM_JOINT_NAMES, target):
        start = current_positions.get(name)
        velocity = RMD_MAX_VELOCITY_RAD_S.get(name)
        if velocity is not None and start is not None and math.isfinite(float(start)):
            required = max(required, abs(float(goal) - float(start)) / velocity + 0.5)
    return required

# maru_ik_node.py와 동일한 그리퍼 상수.
GRIPPER_OPEN = 0.07363
GRIPPER_CLOSED = 1.8294

# maru_ik_node.py의 전류 피드백 파지 판정과 동일한 기본값
# (gripper_current_logger.py/gripper_safe_close.py로 실기 검증됨).
GRIPPER_CURRENT_THRESHOLD_MA_DEFAULT = 100.0
GRIPPER_CURRENT_LSB_MA = 2.69
# GRIPPER_CLOSED 근처까지 도달했다고 볼 위치 오차 허용폭(rad).
GRIPPER_POSITION_TOLERANCE_RAD = 0.01

# maru_ik_node.py와 동일한 값(2026-08-31 재실측) - 갱신되면 같이 맞출 것.
NAMED_ARM_JOINTS = {
    "stand": {"shoulder_joint": 0.0, "elbow_joint": 0.0, "wrist_joint": 0.0},
    "home": {"shoulder_joint": -1.4818, "elbow_joint": 1.6383, "wrist_joint": 1.5329},
    # [갱신, 2026-09-01] maru_ik_node.py HOLD/GRASP_WAIT shoulder 값 교체와 동일.
    "hold": {"shoulder_joint": -0.76044, "elbow_joint": 1.6383, "wrist_joint": 1.5329},
    "grasp_wait": {"shoulder_joint": 0.0000, "elbow_joint": math.pi / 2, "wrist_joint": math.pi / 2},
    **{name: dict(joints) for name, joints in GRASP_WAIT_PRESETS.items()},
    "bed": {"shoulder_joint": -1.49610, "elbow_joint": 0.35867, "wrist_joint": 1.55648},
    # [추가, 2026-09-01] capture_arm_pose.py "f" 시퀀스(방금 인식된 supplybox를
    # grasp_wait에서부터 실제로 손으로 밀어서 파지 위치까지 재현) 최종 수렴
    # 자세 - compute_fk로 tcp 위치 검증 완료(verify_grasp_pose_fk.py).
    # 실측 base_joint=-0.02761(위 사용법 주석 참고, 기본은 0.0으로 이동).
    "sid": {"shoulder_joint": 1.71566, "elbow_joint": 0.62151, "wrist_joint": 0.82519},
}


def _send_and_wait(node, client, action_name, joint_names, positions, duration_sec, label):
    """FollowJointTrajectory 단일 waypoint 골을 action_name에 보내고
    결과를 기다린다. 성공하면 True, 실패(서버 없음/거부/미완료)하면 False."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
    goal.trajectory.points.append(point)

    node.get_logger().info(
        f"{label}: {dict(zip(joint_names, positions))} ({duration_sec:.1f}s)")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    if not send_future.done():
        node.get_logger().error(f"{label}: goal 전송 응답 timeout.")
        return False
    if send_future.exception() is not None:
        node.get_logger().error(
            f"{label}: goal 전송 실패: {send_future.exception()}")
        return False
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error(f"{label}: 컨트롤러가 goal을 거부했습니다.")
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(
        node, result_future, timeout_sec=max(10.0, duration_sec + 5.0))
    if not result_future.done():
        node.get_logger().error(f"{label}: 실행 결과 timeout; goal 취소 요청.")
        goal_handle.cancel_goal_async()
        return False
    if result_future.exception() is not None:
        node.get_logger().error(
            f"{label}: 실행 결과 수신 실패: {result_future.exception()}")
        return False
    wrapped_result = result_future.result()
    if wrapped_result is None or wrapped_result.result is None:
        node.get_logger().error(f"{label}: controller가 빈 결과를 반환했습니다.")
        return False
    status = wrapped_result.status
    error_code = wrapped_result.result.error_code
    if (
        status != GoalStatus.STATUS_SUCCEEDED
        or error_code != FollowJointTrajectory.Result.SUCCESSFUL
    ):
        node.get_logger().error(
            f"{label}: 실패(status={status}, error_code={error_code}, "
            f"error_string='{wrapped_result.result.error_string}')")
        return False
    node.get_logger().info(f"{label}: 완료.")
    return True


def _close_gripper_with_feedback(
    node, client, duration_sec,
    current_threshold_ma, gripper_state, grasp_pub, label,
):
    """gripper_controller에 GRIPPER_CLOSED 골을 보내고, 닫는 도중 전류가
    current_threshold_ma를 넘으면 즉시 취소한다(maru_ik_node.py
    _close_gripper_with_current_feedback()과 동일한 판정). 전류 스파이크
    없이 GRIPPER_CLOSED 근처까지 완전히 닫힌 경우도 OR 조건으로 성공
    처리한다(모듈 docstring [갱신, 2026-09-02] 절 참고). 두 조건 중
    하나라도 맞으면 grasp_pub에 Bool(true)를, 아니면 Bool(false)를
    발행한다. 반환값은 grasp 성공 여부가 아니라 "모션 자체가 정상
    실행됐는지"(goal 거부/timeout 등 실제 실패만 False)."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = GRIPPER_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = [GRIPPER_CLOSED]
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
    goal.trajectory.points.append(point)

    node.get_logger().info(
        f"{label}: gripper_joint -> {GRIPPER_CLOSED} ({duration_sec:.1f}s, "
        f"current_threshold={current_threshold_ma:.1f}mA)")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    if not send_future.done() or send_future.exception() is not None:
        node.get_logger().error(f"{label}: goal 전송 실패/timeout.")
        return False
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error(f"{label}: 컨트롤러가 goal을 거부했습니다.")
        return False

    result_future = goal_handle.get_result_async()
    deadline = time.monotonic() + max(10.0, duration_sec + 5.0)
    stopped_by_current = False
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        if gripper_state["current_ma"] >= current_threshold_ma:
            node.get_logger().info(
                f"{label}: 전류 {gripper_state['current_ma']:.1f}mA >= "
                f"threshold {current_threshold_ma:.1f}mA - 즉시 정지(파지 성공).")
            goal_handle.cancel_goal_async()
            stopped_by_current = True
            break
        if time.monotonic() >= deadline:
            node.get_logger().error(f"{label}: 실행 결과 timeout; goal 취소 요청.")
            goal_handle.cancel_goal_async()
            return False
    if not result_future.done():
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=5.0)

    if stopped_by_current:
        grasp_success = True
        motion_ok = True
    else:
        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.result is None:
            node.get_logger().error(f"{label}: controller가 빈 결과를 반환했습니다.")
            return False
        status = wrapped_result.status
        error_code = wrapped_result.result.error_code
        motion_ok = (
            status == GoalStatus.STATUS_SUCCEEDED
            and error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not motion_ok:
            node.get_logger().error(
                f"{label}: 실패(status={status}, error_code={error_code}, "
                f"error_string='{wrapped_result.result.error_string}')")
            return False
        position = gripper_state["position"]
        grasp_success = (
            position is not None
            and position >= GRIPPER_CLOSED - GRIPPER_POSITION_TOLERANCE_RAD
        )
        if grasp_success:
            node.get_logger().warn(
                f"{label}: 전류 스파이크 없이 GRIPPER_CLOSED 근처까지 도달 "
                f"(position={position}) - OR 조건으로 성공 처리(물체 유무는 "
                "불확실할 수 있음).")
        else:
            node.get_logger().warn(
                f"{label}: 전류 스파이크도 없고 GRIPPER_CLOSED에도 못 미침 "
                f"(position={position}) - 실패 처리.")

    grasp_pub.publish(Bool(data=grasp_success))
    node.get_logger().info(f"{label}: 완료 (grasp_success={grasp_success}).")
    return motion_ok


def main():
    rclpy.init()
    node = Node("move_to_named_pose")
    node.declare_parameter("pose", "")
    node.declare_parameter("base", 0.0)
    node.declare_parameter("hold_current_base", False)
    node.declare_parameter("gripper", "")
    node.declare_parameter("duration_sec", 3.0)
    # 일반 수동 실행은 기존처럼 유한 timeout/1회 시도를 유지한다. 자동 시작
    # launch만 retry_until_success=true와 0(무제한) timeout을 넘겨, 실기
    # controller 초기화가 늦어도 시작 자세 worker가 먼저 죽지 않게 한다.
    node.declare_parameter("joint_state_wait_timeout_sec", 10.0)
    node.declare_parameter("controller_wait_timeout_sec", 15.0)
    node.declare_parameter("ready_delay_sec", 0.0)
    node.declare_parameter("retry_until_success", False)
    node.declare_parameter("retry_interval_sec", 1.0)
    node.declare_parameter("verify_position_tolerance_rad", 0.03)
    node.declare_parameter("verify_position_timeout_sec", 2.0)
    node.declare_parameter("manual_override_topic", "/control/arm_manual_override")
    node.declare_parameter("manual_override_settle_sec", 0.5)
    node.declare_parameter("gripper_current_threshold_ma", GRIPPER_CURRENT_THRESHOLD_MA_DEFAULT)
    node.declare_parameter("grasp_success_topic", "/arm/grasp_success")
    node.declare_parameter("completion_topic", "")
    pose_name = str(node.get_parameter("pose").value).strip().lower()
    base_value = float(node.get_parameter("base").value)
    hold_current_base = bool(node.get_parameter("hold_current_base").value)
    gripper_action = str(node.get_parameter("gripper").value).strip().lower()
    duration_sec = float(node.get_parameter("duration_sec").value)
    joint_state_wait_timeout_sec = float(
        node.get_parameter("joint_state_wait_timeout_sec").value)
    controller_wait_timeout_sec = float(
        node.get_parameter("controller_wait_timeout_sec").value)
    ready_delay_sec = max(0.0, float(node.get_parameter("ready_delay_sec").value))
    retry_until_success = bool(node.get_parameter("retry_until_success").value)
    retry_interval_sec = max(
        0.1, float(node.get_parameter("retry_interval_sec").value))
    verify_position_tolerance_rad = max(
        0.001, float(node.get_parameter("verify_position_tolerance_rad").value))
    verify_position_timeout_sec = max(
        0.1, float(node.get_parameter("verify_position_timeout_sec").value))
    manual_override_topic = str(node.get_parameter("manual_override_topic").value)
    manual_override_settle_sec = max(
        0.0, float(node.get_parameter("manual_override_settle_sec").value))
    gripper_current_threshold_ma = float(
        node.get_parameter("gripper_current_threshold_ma").value)
    grasp_success_topic = str(node.get_parameter("grasp_success_topic").value)
    completion_topic = str(node.get_parameter("completion_topic").value).strip()

    if not pose_name and not gripper_action:
        node.get_logger().error(
            f"pose 또는 gripper 파라미터 중 하나는 필요합니다 - pose는 "
            f"{list(NAMED_ARM_JOINTS)} 중 하나, gripper는 open/close.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    if pose_name and pose_name not in NAMED_ARM_JOINTS:
        node.get_logger().error(
            f"알 수 없는 pose '{pose_name}' - {list(NAMED_ARM_JOINTS)} 중 하나를 지정하세요.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    if gripper_action and gripper_action not in ("open", "close"):
        node.get_logger().error(f"gripper는 open/close만 가능합니다 (받은 값: '{gripper_action}')")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        node.get_logger().error("duration_sec은 0보다 큰 유한값이어야 합니다.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # launch 자동 시작에서는 base_joint를 0으로 선회시키지 않고 현재 실측
    # 위치를 유지한다. controller action을 기다리기 전에 joint_states를
    # 받아 실제 값을 확보한다.
    arm_positions = {}
    if pose_name:

        def _on_arm_state(msg: JointState) -> None:
            for name, position in zip(msg.name, msg.position):
                if name in ARM_JOINT_NAMES and math.isfinite(position):
                    arm_positions[name] = float(position)

        node.create_subscription(
            JointState, "/joint_states", _on_arm_state, qos_profile_sensor_data)

    if pose_name and hold_current_base:
        wait_started = time.monotonic()
        next_wait_log = wait_started
        while "base_joint" not in arm_positions and rclpy.ok():
            elapsed = time.monotonic() - wait_started
            if joint_state_wait_timeout_sec > 0.0 and elapsed >= joint_state_wait_timeout_sec:
                break
            if time.monotonic() >= next_wait_log:
                node.get_logger().info(
                    "Waiting for a valid base_joint on /joint_states before startup motion...")
                next_wait_log = time.monotonic() + 5.0
            rclpy.spin_once(node, timeout_sec=0.1)
        if "base_joint" not in arm_positions:
            node.get_logger().error(
                "hold_current_base=true인데 제한시간 안에 base_joint 상태를 못 받았습니다.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        base_value = arm_positions["base_joint"]
        node.get_logger().info(f"Holding current base_joint={base_value:.5f}rad.")

    calibration = JointCalibration()
    requested_targets = {}
    if pose_name:
        requested_targets.update(NAMED_ARM_JOINTS[pose_name])
        requested_targets["base_joint"] = base_value
    if gripper_action:
        requested_targets["gripper_joint"] = (
            GRIPPER_OPEN if gripper_action == "open" else GRIPPER_CLOSED)
    invalid = []
    for name, value in requested_targets.items():
        if not math.isfinite(value) or not calibration.has_joint(name):
            invalid.append(name)
            continue
        low, high = calibration.actual_limits(name)
        if not low <= value <= high:
            invalid.append(f"{name}={value:.5f} (limit {low:.5f}..{high:.5f})")
    if invalid:
        node.get_logger().error("관절 목표가 유효하지 않습니다: " + ", ".join(invalid))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # gripper:=close 전류 피드백 판정용 - _close_gripper_with_feedback()이
    # 읽는 위치/전류값(모듈 docstring [갱신, 2026-09-02] 절 참고).
    gripper_state = {"position": None, "current_ma": 0.0}
    grasp_pub = None
    if gripper_action == "close":
        def _on_joint_states(msg: JointState) -> None:
            try:
                idx = msg.name.index("gripper_joint")
            except ValueError:
                return
            if idx < len(msg.position):
                gripper_state["position"] = msg.position[idx]
            if idx < len(msg.effort):
                gripper_state["current_ma"] = abs(msg.effort[idx]) * GRIPPER_CURRENT_LSB_MA

        node.create_subscription(
            JointState, "/joint_states", _on_joint_states, qos_profile_sensor_data)
        grasp_pub = node.create_publisher(Bool, grasp_success_topic, 10)

    clients = {}
    if pose_name:
        clients["arm"] = ActionClient(
            node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if gripper_action:
        clients["gripper"] = ActionClient(
            node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
    for label, client in clients.items():
        wait_started = time.monotonic()
        next_wait_log = wait_started
        server_ready = False
        while rclpy.ok():
            if client.wait_for_server(timeout_sec=0.5):
                server_ready = True
                break
            elapsed = time.monotonic() - wait_started
            if controller_wait_timeout_sec > 0.0 and elapsed >= controller_wait_timeout_sec:
                break
            if time.monotonic() >= next_wait_log:
                node.get_logger().info(
                    f"Waiting for {label}_controller action server to become active...")
                next_wait_log = time.monotonic() + 5.0
        if not server_ready:
            node.get_logger().error(
                f"{label} controller action 서버를 제한시간 안에 못 찾음. "
                "move_group이 아니라 ros2_control controller가 필요합니다.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    # TimerAction의 launch 시작 기준 지연과 달리, 여기부터는 실제 controller
    # action 서버와 joint feedback이 모두 준비된 시점을 기준으로 센다.
    if ready_delay_sec:
        node.get_logger().info(
            f"Hardware/controller ready; waiting {ready_delay_sec:.1f}s before commanding pose.")
        deadline = time.monotonic() + ready_delay_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))

    override_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    override_pub = node.create_publisher(Bool, manual_override_topic, override_qos)
    override_msg = Bool(data=True)
    override_pub.publish(override_msg)
    heartbeat = node.create_timer(0.2, lambda: override_pub.publish(override_msg))
    if manual_override_settle_sec:
        deadline = time.monotonic() + manual_override_settle_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))
    node.get_logger().info(
        f"Manual override acquired on {manual_override_topic}; direct controller command enabled.")

    ok = False
    try:
        def arm_target_reached(target) -> bool:
            deadline = time.monotonic() + verify_position_timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                missing_or_far = [
                    name for name, expected in zip(ARM_JOINT_NAMES, target)
                    if name not in arm_positions
                    or abs(arm_positions[name] - expected) > verify_position_tolerance_rad
                ]
                if not missing_or_far:
                    return True
                rclpy.spin_once(node, timeout_sec=0.05)
            errors = {
                name: (
                    None if name not in arm_positions
                    else arm_positions[name] - expected
                )
                for name, expected in zip(ARM_JOINT_NAMES, target)
                if name not in arm_positions
                or abs(arm_positions[name] - expected) > verify_position_tolerance_rad
            }
            node.get_logger().error(
                "Controller completed but measured arm joints did not reach the target "
                f"within {verify_position_tolerance_rad:.3f}rad: errors={errors}")
            return False

        attempt = 0
        while rclpy.ok():
            attempt += 1
            attempt_ok = True
            if retry_until_success:
                node.get_logger().info(f"Startup pose execution attempt {attempt}.")
            if pose_name:
                joints = NAMED_ARM_JOINTS[pose_name]
                target = [
                    base_value if name == "base_joint" else joints[name]
                    for name in ARM_JOINT_NAMES
                ]
                command_duration_sec = required_arm_duration(
                    arm_positions, target, duration_sec)
                if command_duration_sec > duration_sec + 1e-6:
                    node.get_logger().info(
                        f"RMD 속도 제한에 맞춰 trajectory 시간을 "
                        f"{duration_sec:.1f}s -> {command_duration_sec:.1f}s로 연장합니다.")
                attempt_ok = _send_and_wait(
                    node, clients["arm"], "/arm_controller/follow_joint_trajectory",
                    ARM_JOINT_NAMES, target, command_duration_sec,
                    f"arm -> '{pose_name}'")
                if attempt_ok:
                    attempt_ok = arm_target_reached(target)

            if attempt_ok and gripper_action == "open":
                attempt_ok = _send_and_wait(
                    node, clients["gripper"], "/gripper_controller/follow_joint_trajectory",
                    GRIPPER_JOINT_NAMES, [GRIPPER_OPEN], duration_sec, "gripper open")
            elif attempt_ok and gripper_action == "close":
                attempt_ok = _close_gripper_with_feedback(
                    node, clients["gripper"], duration_sec, gripper_current_threshold_ma,
                    gripper_state, grasp_pub, "gripper close")

            if attempt_ok:
                ok = True
                break
            if not retry_until_success:
                break
            node.get_logger().warn(
                f"Startup pose attempt {attempt} failed; retrying in "
                f"{retry_interval_sec:.1f}s.")
            retry_deadline = time.monotonic() + retry_interval_sec
            while rclpy.ok() and time.monotonic() < retry_deadline:
                rclpy.spin_once(
                    node, timeout_sec=min(0.1, retry_deadline - time.monotonic()))
    finally:
        heartbeat.cancel()
        override_pub.publish(Bool(data=False))
        # Reliable DDS writer가 false를 전송할 짧은 시간을 확보한다. 전달에
        # 실패해도 maru_ik_node의 lease timeout이 자동으로 해제한다.
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info("Manual override released.")

    if ok and completion_topic:
        complete_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        completion_pub = node.create_publisher(Empty, completion_topic, complete_qos)
        completion_pub.publish(Empty())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info(f"Startup completion published: {completion_topic}")

    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
