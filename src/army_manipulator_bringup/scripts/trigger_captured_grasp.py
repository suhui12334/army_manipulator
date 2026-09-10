#!/usr/bin/env python3
"""캡처된 마지막 파지 관절값을 compute_fk로 tcp 좌표로 역산해서
/arm/target_point에 1회 publish한다 - maru_ik_node의 on_target을 그대로
트리거해서 pregrasp->gripper open->descend->(confirm)->gripper close->hold
전체 시퀀스가 실제로 도는지 확인하기 위한 1회성 테스트 스크립트.

실행 전 확인:
  - move_group / maru_ik_node / rmd/dxl ros2_control 하드웨어 인터페이스가
    떠 있어야 함(compute_fk 서비스, /arm/target_point 구독자 모두 필요).
  - confirm_before_close 기본값(false)이면 그리퍼가 자동으로 닫힘 - 팔
    주변 확인 후 실행. 이상하면 Ctrl+C 또는
    `ros2 topic pub -1 /control/auto_enabled std_msgs/msg/Bool "{data: false}"`
    로 즉시 정지 가능.

사용:
    python3 trigger_captured_grasp.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

# maru_ik_node.py와 동일 상수/공식(WRIST_TO_TCP_LOCAL_OFFSET,
# _rotate_vector_by_quaternion) - 이 스크립트만 따로 돌리기 위해 그대로 복사.
WRIST_TO_TCP_LOCAL_OFFSET = (-0.240, 0.0, 0.0313)

# capture_arm_pose.py로 캡처한 실제 파지 궤적의 마지막(그리퍼가 닿는 지점) 자세.
# [갱신, 2026-09-01] "f" 시퀀스(방금 인식된 supplybox를 grasp_wait에서부터
# 실측)의 최종 수렴 자세로 교체 - verify_grasp_pose_fk.py의 CAPTURED_JOINTS와
# 반드시 같이 갱신할 것(둘이 다른 자세를 참조하면 서로 다른 값이 나온다).
CAPTURED_JOINTS = {
    "base_joint": -0.02761,
    "shoulder_joint": 1.71566,
    "elbow_joint": 0.62151,
    "wrist_joint": 0.82519,
}

# maru_ik_node.py 기본 파라미터(grasp_offset_x/y=0.0) - 바꿨다면 여기도 맞출 것.
GRASP_OFFSET_X = 0.0
GRASP_OFFSET_Y = 0.0


def rotate_vector_by_quaternion(q, v):
    ux, uy, uz, w = q
    vx, vy, vz = v
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    return (
        vx + w * tx + (uy * tz - uz * ty),
        vy + w * ty + (uz * tx - ux * tz),
        vz + w * tz + (ux * ty - uy * tx),
    )


def main():
    rclpy.init()
    node = Node("trigger_captured_grasp")

    client = node.create_client(GetPositionFK, "/compute_fk")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/compute_fk 서비스를 못 찾음 - move_group이 떠 있는지 확인하세요.")
        rclpy.shutdown()
        return

    req = GetPositionFK.Request()
    req.header.frame_id = "base_actuator"
    req.fk_link_names = ["wrist_link"]
    req.robot_state = RobotState()
    req.robot_state.joint_state.name = list(CAPTURED_JOINTS.keys())
    req.robot_state.joint_state.position = list(CAPTURED_JOINTS.values())

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    res = future.result()
    if res is None or not res.pose_stamped:
        node.get_logger().error(f"compute_fk 실패: {res}")
        rclpy.shutdown()
        return

    pose = res.pose_stamped[0].pose
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    offset_world = rotate_vector_by_quaternion(q, WRIST_TO_TCP_LOCAL_OFFSET)
    tcp_x = pose.position.x + offset_world[0]
    tcp_y = pose.position.y + offset_world[1]
    tcp_z = pose.position.z + offset_world[2]
    node.get_logger().info(
        f"wrist_link FK: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}), "
        f"tcp 역산: ({tcp_x:.4f}, {tcp_y:.4f}, {tcp_z:.4f})"
    )

    # _run_sequence: detected_xyz + grasp_offset = 최종 목표. 여기서는 역으로
    # detected_xyz = tcp - grasp_offset. use_fixed_ground_z(기본 true)면 z는
    # 어차피 ground_z 파라미터로 덮어써지므로 tcp_z를 그대로 넣어도 무방.
    detected_x = tcp_x - GRASP_OFFSET_X
    detected_y = tcp_y - GRASP_OFFSET_Y
    detected_z = tcp_z

    msg = PointStamped()
    msg.header.frame_id = ""  # 이미 base_actuator(=planning_frame) 좌표이므로 TF 변환 생략.
    msg.point.x = detected_x
    msg.point.y = detected_y
    msg.point.z = detected_z

    pub = node.create_publisher(PointStamped, "/arm/target_point", 10)
    # [수정, 2026-09-01] 고정 1초 sleep은 DDS discovery가 늦으면 구독자가 아직
    # 안 잡힌 상태로 publish해서 메시지가 그냥 사라지는 경우가 있었다(실기에서
    # "publish 로그는 찍혔는데 팔이 안 움직임" 확인) - get_subscription_count()로
    # maru_ik_node가 실제로 잡혔는지 확인하고 없으면 경고 후 그래도 시도한다.
    import time
    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().warn(
            "/arm/target_point에 구독자가 없음(maru_ik_node가 안 떠 있거나 아직 "
            "discovery 안 됨) - 그래도 publish는 하지만 아무 반응이 없을 수 있음."
        )
    else:
        node.get_logger().info(
            f"/arm/target_point 구독자 {pub.get_subscription_count()}개 확인됨.")
    pub.publish(msg)
    node.get_logger().info(
        f"/arm/target_point publish: ({detected_x:.4f}, {detected_y:.4f}, {detected_z:.4f}) "
        "- maru_ik_node의 전체 파지 시퀀스가 시작됩니다."
    )
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
