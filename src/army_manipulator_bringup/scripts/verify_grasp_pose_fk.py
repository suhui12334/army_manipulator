#!/usr/bin/env python3
"""여러 자세를 compute_fk로 한 번에 검증한다.

wrist_link 위치/자세를 구한 뒤 maru_ik_node.py와 동일한
WRIST_TO_TCP_LOCAL_OFFSET 보정을 적용해 실제 그리퍼(tcp_link)가 base_actuator
기준 어디에 있는지, 그리고 수평 리치(x0=hypot(x,y))가 얼마인지 계산해서
출력한다 - "이 관절값이면 실제로 박스가 있는 자리를 집는 게 맞는지",
"이 자세가 실패하는 원거리 타겟(x0)까지 실제로 닿는지" 확인하는 용도.

[갱신, 2026-09-02] 실제 grasp_wait -> sid -> hold 흐름을 FK로 함께 출력한다.
sid는 실제 파지 위치에서 캡처한 IK seed이므로, 이 값의 TCP 위치가 인식된
박스 위치와 맞는지를 먼저 확인한다.

사용:
    python3 verify_grasp_pose_fk.py
"""
import math

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

# maru_ik_node.py와 동일 상수/공식.
WRIST_TO_TCP_LOCAL_OFFSET = (-0.240, 0.0, 0.0313)

# 검증할 자세들. 필요하면 자유롭게 추가/삭제.
POSES = {
    # move_to_named_pose.py의 실제 파지 흐름. base는 sid를 캡처했을 때의
    # 실측값이며, grasp_wait -> sid -> hold 동안 유지된다.
    "grasp_wait(파지 전 대기)": {
        "base_joint": -0.02761, "shoulder_joint": -0.56531,
        "elbow_joint": 1.63830, "wrist_joint": 1.53290,
    },
    # capture_arm_pose.py "f" 시퀀스의 최종 수렴 자세(실제 파지 위치).
    "sid(파지 위치)": {
        "base_joint": -0.02761, "shoulder_joint": 1.71566,
        "elbow_joint": 0.62151, "wrist_joint": 0.82519,
    },
    "hold(파지 후 주행 대기)": {
        "base_joint": -0.02761, "shoulder_joint": -0.76044,
        "elbow_joint": 1.63830, "wrist_joint": 1.53290,
    },
    # [수정, 2026-09-02] STAND(전부 0도)는 "수평으로 최대한 편 자세"가 아니라
    # 엔코더 영점 자체가 "천장을 향해 수직으로 편 자세"라(joint_calibration.yaml
    # 주석 참고) FK 결과 z=0.714, x0=0으로 완전히 위를 가리켜서 수평 최대
    # 리치 비교에 못 쓴다는 게 실기로 확인됨 - 대신 elbow를 거의 편(0.1) 채로
    # pitch(=shoulder+elbow+wrist 합)를 실제 하강 자세들과 비슷한 3.0으로
    # 고정하고 shoulder를 바꿔가며(wrist는 관절 리미트 안에서 pitch를 맞추게
    # 역산) 실패한 타겟 높이(z≈-0.397)에 가까운 지점의 x0을 찾는다.
    "거의폄 shoulder=1.5 pitch=3.0": {
        "base_joint": 0.0, "shoulder_joint": 1.5,
        "elbow_joint": 0.1, "wrist_joint": 1.4,
    },
    "거의폄 shoulder=1.6 pitch=3.0": {
        "base_joint": 0.0, "shoulder_joint": 1.6,
        "elbow_joint": 0.1, "wrist_joint": 1.3,
    },
    "거의폄 shoulder=1.7 pitch=3.0": {
        "base_joint": 0.0, "shoulder_joint": 1.7,
        "elbow_joint": 0.1, "wrist_joint": 1.2,
    },
}


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
    node = Node("verify_grasp_pose_fk")

    client = node.create_client(GetPositionFK, "/compute_fk")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/compute_fk 서비스를 못 찾음 - move_group이 떠 있는지 확인하세요.")
        rclpy.shutdown()
        return

    for pose_name, joints in POSES.items():
        req = GetPositionFK.Request()
        req.header.frame_id = "base_actuator"
        req.fk_link_names = ["wrist_link", "tcp_link"]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(joints.keys())
        req.robot_state.joint_state.position = list(joints.values())

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        if res is None or not res.pose_stamped:
            print(f"[{pose_name}] compute_fk 실패: {res}")
            continue

        wrist_pose = res.pose_stamped[res.fk_link_names.index("wrist_link")].pose
        q = (wrist_pose.orientation.x, wrist_pose.orientation.y,
             wrist_pose.orientation.z, wrist_pose.orientation.w)
        offset_world = rotate_vector_by_quaternion(q, WRIST_TO_TCP_LOCAL_OFFSET)
        tcp_x = wrist_pose.position.x + offset_world[0]
        tcp_y = wrist_pose.position.y + offset_world[1]
        tcp_z = wrist_pose.position.z + offset_world[2]
        x0 = math.hypot(tcp_x, tcp_y)
        pitch = sum(joints[n] for n in ("shoulder_joint", "elbow_joint", "wrist_joint"))
        print(
            f"[{pose_name}] tcp(base_actuator): x={tcp_x:.4f}, y={tcp_y:.4f}, z={tcp_z:.4f}, "
            f"x0(수평거리)={x0:.4f}, pitch={pitch:.4f}"
        )

    print(
        "\n위 세 행은 실제 grasp_wait -> sid -> hold 흐름의 TCP 위치다. "
        "IK seed 추가 전에는 반드시 sid 행이 의도한 파지 위치와 일치하는지 "
        "확인할 것."
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
