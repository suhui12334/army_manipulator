#!/usr/bin/env python3
"""여러 관절 자세를 /check_state_validity로 검사해서 자기충돌 여부와,
충돌이면 정확히 어느 링크끼리 부딪히는지(예: 구동부 박스) 출력한다.

구동부 박스/바퀴/회로박스 등 URDF 콜리전 지오메트리를 손대기 전에, 지금
쓰고 있는 자세들이 실제로 그 부위와 충돌 판정을 받는지부터 근거를 만드는
용도 - 근거 없이 지오메트리부터 세분화하는 걸 피하기 위한 진단 스크립트.

사용:
    ros2 run army_manipulator_bringup check_self_collision.py
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

# 검사할 자세들 - maru_ik_node.py의 IK_SEEDS/HOME/HOLD/GRASP_WAIT,
# verify_grasp_pose_fk.py의 sid/far_reach/STAND와 동일한 값들.
POSES = {
    "NEAR[0] 근거리 elbow접힘": [-0.01687, 1.42349, 1.30027, 0.48066],
    "NEAR[1] 근거리 elbow펴짐": [-0.01687, 1.71007, 0.61104, 0.49585],
    "NEAR[2] sid": [-0.02761, 1.71566, 0.62151, 0.82519],
    "MEASURED_GROUND[0] 초기": [-0.02915, 0.36652, 1.62316, 1.02974],
    "MEASURED_GROUND[1] 중간": [-0.02915, 1.39626, 1.27409, 0.31416],
    "MEASURED_GROUND[2] 최종 수렴": [-0.09357, 1.65806, 0.99484, 0.08727],
    "SYNTHETIC_FAR[0] 거의폄(shoulder낮음)": [-0.02000, 0.30000, 0.10000, 1.00000],
    "SYNTHETIC_FAR[1] 거의폄(shoulder높음)": [-0.02000, 1.60000, 0.10000, 1.00000],
    "HOME": [0.0, -1.4818, 1.6383, 1.5329],
    "HOLD": [0.0, -0.76044, 1.6383, 1.5329],
    "GRASP_WAIT": [0.0, -0.56531, 1.6383, 1.5329],
    "sid(근거리 파지 최종)": [-0.02761, 1.71566, 0.62151, 0.82519],
    "STAND(완전히 폄)": [0.0, 0.0, 0.0, 0.0],
}


def main():
    rclpy.init()
    node = Node("check_self_collision")

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/check_state_validity 서비스를 못 찾음 - move_group이 떠 있는지 확인하세요.")
        rclpy.shutdown()
        return

    any_invalid = False
    for pose_name, joints in POSES.items():
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = list(joints)
        req.robot_state.is_diff = False

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            print(f"[{pose_name}] 서비스 호출 실패")
            continue

        if res.valid:
            print(f"[{pose_name}] OK (충돌 없음)")
        else:
            any_invalid = True
            pairs = ", ".join(
                f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts
            ) or "(contacts 정보 없음 - 관절 리미트 위반일 수도 있음)"
            print(f"[{pose_name}] 충돌/무효: {pairs}")

    if not any_invalid:
        print("\n전부 유효(충돌 없음) - 구동부 박스 등 콜리전 지오메트리는 "
              "지금 쓰는 자세들의 원인이 아닌 것으로 보입니다.")
    else:
        print("\n위에서 무효로 나온 자세의 접촉 링크 이름을 보고 실제로 "
              "구동부 박스(혹은 다른 부위)인지 확인하세요.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
