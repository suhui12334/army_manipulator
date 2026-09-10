#!/usr/bin/env python3
"""매니퓰레이터를 옆에서 본 것처럼 2D로 그려주는 완전 독립 실행 스크립트
(path_visualizer.py와 동일 패턴 - army_manipulator_bringup 패키지 의존성 없음,
rclpy + tf2_ros + sensor_msgs + opencv-python + numpy만 필요).

DH 트리거노메트리를 직접 손으로 다시 계산하지 않는다 - shoulder/elbow/wrist가
지금 전부 axis=Z로 재구성되어 있어서(army_manipulator_macro.xacro 상단 주석)
"조인트각 그대로 삼각함수"로 그리면 이번 세션에서 겪은 것과 같은 부호/축
실수가 또 날 수 있다. 대신 robot_state_publisher가 실제 엔코더값으로 이미
정확히 계산해둔 TF(base_link -> shoulder_link/elbow_link/wrist_link/
pinion_gear)를 그대로 읽어서 좌표만 옮겨 그린다 - 항상 정확하다.

"베이스 회전 제외"는 base_link 자신의 좌표계로 그리는 것으로 자동 해결된다 -
base_link는 base_joint 회전에 따라 같이 돌아가는 프레임이라, 그 프레임
기준으로 그리면 base_joint 값이 뭐든 상관없이 항상 "팔 자체가 지금 어떻게
접혀 있는지"만 보인다.

이번 세션에서 compute_fk로 실측 확인한 사실(quaternion_from_base_pitch 관련
주석 참고): base_joint=0일 때 shoulder/elbow/wrist가 만드는 옆모습은 X-Z
평면에 있고 Y(수직, base_link 기준 아래쪽 +)는 거의 상수다 - 그래서 이
"옆에서 본 모습"은 X-Z 평면 투영이 자연스럽다(Z=전방을 화면 가로, X를
화면 세로로 매핑).
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
import tf2_ros

BASE_FRAME = "base_link"
# 그릴 체인 - base_link 원점(0,0)에서 시작해서 순서대로 이어 그린다.
CHAIN_FRAMES = ["shoulder_link", "elbow_link", "wrist_link", "pinion_gear"]
JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

CANVAS_SIZE = (700, 700)  # (width, height) px
# 전체 팔 뻗었을 때 최대 reach가 L1+L2+L3 ~= 0.55m + tcp_off(0.090m)까지
# 고려해서, 어느 자세든 캔버스 안에 들어오도록 여유있게 잡음.
SCALE_PX_PER_M = 450.0
ORIGIN_PX = (200, 350)  # base_link를 캔버스 어디에 그릴지 (x, y)


def world_to_canvas(x_m: float, z_m: float):
    """base_link 기준 (x, z)[m] -> 캔버스 픽셀 좌표.

    [주의] chassis_link -> base_link TF가 180도 yaw(REP-103 정합용,
    army_manipulator_macro.xacro의 world_to_base 조인트 참고)라서
    base_link의 로컬 +Z는 실제로는 섀시 "후방"을 가리킨다. z_m 부호를 반전해서
    로봇 실제 정면이 화면 오른쪽에 오도록 한 번 고쳤었는데, 실기로 확인해보니
    그래도 방향이 이상해서(2026-08-27) 화면 전체를 180도 더 돌려달라는 피드백을
    반영했다 - x_m 부호도 마저 반전(전체적으로 원래 식 대비 x,z 둘 다 반전).
    """
    px = int(ORIGIN_PX[0] + z_m * SCALE_PX_PER_M)
    py = int(ORIGIN_PX[1] + x_m * SCALE_PX_PER_M)
    return px, py


class SideViewVisualizer(Node):
    def __init__(self):
        super().__init__("side_view_visualizer")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_positions = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.frame = np.zeros((CANVAS_SIZE[1], CANVAS_SIZE[0], 3), dtype=np.uint8)

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                self.joint_positions[name] = pos

    def _lookup_chain(self):
        """base_link 기준 체인의 각 링크 (x, z)를 반환. TF 없으면 None."""
        points = [(0.0, 0.0)]  # base_link 원점
        for frame in CHAIN_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME, frame, Time(), timeout=Duration(seconds=0.1))
            except Exception:
                return None
            points.append((t.transform.translation.x, t.transform.translation.z))
        return points

    def render(self) -> None:
        self.frame[:] = (30, 30, 30)

        # 격자 (10cm 간격)
        for i in range(-10, 11):
            gx, gy = world_to_canvas(0, i * 0.1)
            cv2.line(self.frame, (gx, 0), (gx, CANVAS_SIZE[1]), (55, 55, 55), 1)
        for i in range(-10, 11):
            gx, gy = world_to_canvas(i * 0.1, 0)
            cv2.line(self.frame, (0, gy), (CANVAS_SIZE[0], gy), (55, 55, 55), 1)
        cv2.line(self.frame, (0, ORIGIN_PX[1]), (CANVAS_SIZE[0], ORIGIN_PX[1]), (90, 90, 90), 2)
        cv2.line(self.frame, (ORIGIN_PX[0], 0), (ORIGIN_PX[0], CANVAS_SIZE[1]), (90, 90, 90), 2)
        # 180도 반전 이후: z_m 음수(base_link -Z, 실제 섀시 정면)가 이제 화면
        # 왼쪽에 찍힌다 - 그래서 라벨/화살표도 왼쪽으로 옮김. X는 이제 커질수록
        # 화면 아래로 가므로(이전엔 위) 라벨도 아래로 옮김 - 물리적으로 위/아래
        # 어느 쪽이 맞는지는 아직 별도 확인 필요, 일단 코드 동작과 라벨을
        # 일치시킴.
        cv2.putText(self.frame, "<- 정면(front)", (20, ORIGIN_PX[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        cv2.putText(self.frame, "X+ (아래로 증가)", (ORIGIN_PX[0] + 8, CANVAS_SIZE[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)

        points = self._lookup_chain()
        if points is None:
            cv2.putText(self.frame, "TF 대기 중 (robot_state_publisher 떠 있는지 확인)",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            pixel_pts = [world_to_canvas(x, z) for x, z in points]
            for i in range(len(pixel_pts) - 1):
                cv2.line(self.frame, pixel_pts[i], pixel_pts[i + 1], (0, 200, 255), 4, cv2.LINE_AA)
            for i, p in enumerate(pixel_pts):
                cv2.circle(self.frame, p, 7, (0, 200, 255), -1, lineType=cv2.LINE_AA)
                cv2.circle(self.frame, p, 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
            cv2.circle(self.frame, pixel_pts[0], 10, (255, 255, 255), 2, lineType=cv2.LINE_AA)

        # 관절각 텍스트 (라디안 -> 도 변환)
        y = CANVAS_SIZE[1] - 90
        for name in JOINT_NAMES:
            val = self.joint_positions.get(name)
            label = name.replace("_joint", "")
            if val is None:
                text = f"{label:>9}: N/A"
            else:
                mark = " (그림에서 제외)" if name == "base_joint" else ""
                text = f"{label:>9}: {val:+7.4f} rad ({math.degrees(val):+7.2f} deg){mark}"
            cv2.putText(self.frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y += 20


def main(args=None):
    rclpy.init(args=args)
    node = SideViewVisualizer()
    window = "army_manipulator side view (base rotation excluded)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.render()
            cv2.imshow(window, node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
