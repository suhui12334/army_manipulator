#!/usr/bin/env python3
"""수동으로 움직인 팔 자세의 엔코더 값을 캡처해서 maru_ik_node.py의
HOME_ARM_JOINTS/HOLD_ARM_JOINTS dict 형식(rad)으로 바로 출력한다.

/joint_states만 구독하고 아무 것도 publish하지 않는다(읽기 전용) - 그래서
rmd_joint_state_bridge(포지션 커맨드 없이 getter로만 읽는 노드)를 띄운 뒤
팔을 손으로 자유롭게 움직이면서 실행해도 안전하다.

사용:
    ros2 launch army_manipulator_description display.launch.py   # RViz(선택)
    ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py
    ros2 run army_manipulator_bringup capture_arm_pose.py

팔을 원하는 자세로 옮긴 뒤 Enter(라벨 입력 가능)를 누르면 그 순간 값을
캡처해서 출력한다. Ctrl+C로 종료.
"""
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]


class PoseCapture(Node):
    def __init__(self):
        super().__init__('arm_pose_capture')
        self._latest = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in ARM_JOINT_NAMES:
                self._latest[name] = pos

    def snapshot(self):
        return dict(self._latest)


def main():
    rclpy.init()
    node = PoseCapture()

    # Ctrl+C가 입력 대기 중에 들어오면 rclpy의 signal handler가 context를
    # 먼저 종료한다. spin()은 그 정상 종료를 ExternalShutdownException으로
    # 알리므로, 스레드 traceback을 출력하지 않도록 처리한다.
    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    print("팔을 손으로 원하는 자세로 옮긴 뒤 Enter를 누르면 캡처합니다.")
    print("(라벨 입력 후 Enter, 그냥 Enter면 UNNAMED. 종료는 Ctrl+C)")
    try:
        while True:
            label = input("\n라벨(예: HOME, HOLD): ").strip() or "UNNAMED"
            snap = node.snapshot()
            missing = [n for n in ARM_JOINT_NAMES if n not in snap]
            if missing:
                print(f"  아직 /joint_states에서 못 받은 조인트: {missing} "
                      "- rmd_joint_state_bridge가 떠 있는지 확인 후 다시 시도하세요.")
                continue
            print(f"\n{label}_ARM_JOINTS = {{")
            for name in ("shoulder_joint", "elbow_joint", "wrist_joint"):
                print(f'    "{name}": {snap[name]:.5f},')
            print("}")
            print(f"# base_joint = {snap['base_joint']:.5f}  "
                  "(base는 HOME/HOLD_ARM_JOINTS에 안 들어감 - maru_ik_node.py 참고,"
                  " 진입 동작 중엔 현재 위치를 그대로 유지함)")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
