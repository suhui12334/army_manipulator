#!/usr/bin/env python3
"""RViz에서 tcp_link 기준으로 드래그하면 실제 팔이 따라 움직이는 인터랙티브 마커.

MoveIt RViz의 기본 드래그 핸들은 "arm" 플래닝 그룹의 tip_link(wrist_link)에
고정되어 있다. army_manipulator.srdf 12-16행 주석에 따르면, 예전에 "arm"
그룹을 tcp_link까지(gripper_joint 포함) 확장한 버전을 Setup Assistant가
자동생성했었는데 moveit_controllers.yaml의 arm_controller(4DOF)/
gripper_controller 매핑과 안 맞아 실제 구동 테스트에서 확인 후 지금의
4DOF(wrist_link tip) 버전으로 되돌린 이력이 있다 - 즉 "arm" 그룹을 다시
tcp_link까지 늘리는 건 이미 한 번 실패로 확인된 접근이다.

그래서 이 노드는 "arm" 그룹/컨트롤러 매핑을 전혀 건드리지 않는다. 대신
tcp_link의 현재 pose에 별도 6DOF 인터랙티브 마커를 띄우고, 드래그가
끝나면(MOUSE_UP) 그 목표 pose를 wrist_link<->tcp_link의 현재(고정 가정)
TF 오프셋을 이용해 wrist_link 기준 목표로 역산해서, 기존에 검증된 "arm"
그룹(4DOF, wrist_link tip)에 그대로 move_to_pose()로 보낸다.

주의: 이 오프셋은 gripper_joint의 "현재" 값 기준으로 매 순간 TF lookup해서
쓴다 - 그리퍼가 열리고 닫히는 도중에 드래그하면 그 순간의 그리퍼 폭 기준으로
계산된다(그리퍼 폭이 바뀌면 tcp_link 위치도 미세하게 바뀌는 구조라면).
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from pymoveit2 import MoveIt2, MoveIt2State

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    result = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (result[0], result[1], result[2])


def invert_pose(pos, quat):
    q_inv = quat_conjugate(quat)
    p_inv = tuple(-c for c in rotate_vector(q_inv, pos))
    return p_inv, q_inv


def compose_pose(pos_a, quat_a, pos_b, quat_b):
    """프레임 A 안에 놓인 pose B(A 기준 국소좌표)를 상위 프레임 좌표로 합성한다."""
    q_result = quat_multiply(quat_a, quat_b)
    p_rotated = rotate_vector(quat_a, pos_b)
    p_result = tuple(a + b for a, b in zip(pos_a, p_rotated))
    return p_result, q_result


def make_6dof_marker(frame_id: str, pose: Pose) -> InteractiveMarker:
    marker = InteractiveMarker()
    marker.header.frame_id = frame_id
    marker.pose = pose
    marker.scale = 0.15
    marker.name = "tcp_jog"
    marker.description = "Drag to jog TCP (real hardware moves on release)"

    visual = Marker()
    visual.type = Marker.SPHERE
    visual.scale.x = visual.scale.y = visual.scale.z = 0.03
    visual.color.r = 0.2
    visual.color.g = 0.8
    visual.color.b = 0.2
    visual.color.a = 0.8
    visual_control = InteractiveMarkerControl()
    visual_control.always_visible = True
    visual_control.markers.append(visual)
    marker.controls.append(visual_control)

    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for ax, ay, az in axes:
        move = InteractiveMarkerControl()
        move.orientation.w = 1.0
        move.orientation.x, move.orientation.y, move.orientation.z = ax, ay, az
        move.name = f"move_{ax}{ay}{az}"
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move)

        rotate = InteractiveMarkerControl()
        rotate.orientation.w = 1.0
        rotate.orientation.x, rotate.orientation.y, rotate.orientation.z = ax, ay, az
        rotate.name = f"rotate_{ax}{ay}{az}"
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate)

    return marker


class TcpJogMarker(Node):
    def __init__(self):
        super().__init__("tcp_jog_marker")
        group = ReentrantCallbackGroup()

        # base_link는 base_joint의 자식이라 base_joint 회전과 함께 도는
        # 프레임이다 - SRDF arm 체인의 실제 고정 루트는 base_actuator
        # (maru_ik_node.py/random_target_publisher.py와 동일한 이유로 수정,
        # commit 686ee47 참고 - 이 파일은 그 뒤에 추가돼 누락돼 있었음).
        # base_link로 두면 인터랙티브 마커가 base_joint 회전을 따라 함께
        # 돌아 "베이스가 비틀린" 것처럼 보이고, move_to_pose() 타겟의 실제
        # 의미도 base_joint 값에 따라 달라진다.
        self.declare_parameter("planning_frame", "base_actuator")
        self.declare_parameter("cartesian", False)
        self.declare_parameter("motion_timeout_sec", 20.0)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.cartesian = bool(self.get_parameter("cartesian").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINT_NAMES, base_link_name=self.planning_frame,
            end_effector_name="wrist_link", group_name="arm", callback_group=group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )

        self._server = InteractiveMarkerServer(self, "tcp_jog_marker")
        self._busy = threading.Lock()

        threading.Thread(target=self._init_marker_when_ready, daemon=True).start()

    def _lookup(self, target_frame: str, source_frame: str):
        return self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time(), timeout=Duration(seconds=1.0))

    def _init_marker_when_ready(self) -> None:
        # rclpy.spin_once()를 여기서 부르면 안 된다 - 메인 스레드가 이미
        # executor.spin()으로 이 노드를 돌리고 있어서 중첩 spin은 멎어버린다
        # (이번 세션에서 random_target_publisher.py 만들 때 겪은 문제와 동일).
        # tf_buffer는 이미 spin 중인 executor의 콜백으로 알아서 채워지므로
        # 그냥 폴링만 하면 된다.
        transform = None
        for _ in range(100):
            try:
                transform = self._lookup(self.planning_frame, "tcp_link")
                break
            except Exception:
                time.sleep(0.1)
        if transform is None:
            self.get_logger().error(
                f"tcp_link -> {self.planning_frame} TF를 10초 안에 못 받음 - "
                "robot_state_publisher/TF가 떠 있는지 확인할 것.")
            return

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation

        marker = make_6dof_marker(self.planning_frame, pose)
        self._server.insert(marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()
        self.get_logger().info(
            f"tcp_jog_marker ready at ({pose.position.x:.3f}, {pose.position.y:.3f}, "
            f"{pose.position.z:.3f}) in {self.planning_frame}. RViz에서 InteractiveMarkers "
            "디스플레이 추가하고 드래그하면 실제 팔이 움직임(놓는 순간 실행)."
        )

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("이미 이동 중 - 이번 드래그는 무시함(끝나면 다시 시도).")
            return
        threading.Thread(target=self._execute_move, args=(feedback.pose,), daemon=True).start()

    def _execute_move(self, tcp_pose: Pose) -> None:
        try:
            try:
                wrist_to_tcp = self._lookup("wrist_link", "tcp_link")
            except Exception as exc:
                self.get_logger().error(f"wrist_link<-tcp_link TF lookup 실패: {exc}")
                return

            p_wrist_tcp = (
                wrist_to_tcp.transform.translation.x,
                wrist_to_tcp.transform.translation.y,
                wrist_to_tcp.transform.translation.z,
            )
            q_wrist_tcp = (
                wrist_to_tcp.transform.rotation.x,
                wrist_to_tcp.transform.rotation.y,
                wrist_to_tcp.transform.rotation.z,
                wrist_to_tcp.transform.rotation.w,
            )
            p_tcp_wrist, q_tcp_wrist = invert_pose(p_wrist_tcp, q_wrist_tcp)

            p_target = (tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z)
            q_target = (
                tcp_pose.orientation.x, tcp_pose.orientation.y,
                tcp_pose.orientation.z, tcp_pose.orientation.w,
            )
            p_wrist_target, q_wrist_target = compose_pose(p_target, q_target, p_tcp_wrist, q_tcp_wrist)

            self.get_logger().info(
                f"TCP target ({p_target[0]:.3f},{p_target[1]:.3f},{p_target[2]:.3f}) -> "
                f"wrist target ({p_wrist_target[0]:.3f},{p_wrist_target[1]:.3f},{p_wrist_target[2]:.3f})"
            )
            self.arm.move_to_pose(
                position=p_wrist_target, quat_xyzw=q_wrist_target, cartesian=self.cartesian,
                cartesian_fraction_threshold=0.98,
            )

            deadline = time.monotonic() + self.motion_timeout_sec
            while self.arm.query_state() != MoveIt2State.IDLE:
                if time.monotonic() >= deadline:
                    self.get_logger().error("이동 타임아웃 - 취소함.")
                    self.arm.cancel_execution()
                    return
                time.sleep(0.05)
            if not self.arm.motion_suceeded:
                self.get_logger().warn("MoveIt에서 이동 실패(도달 불가/충돌 등) - 마커는 원위치로 안 돌아감.")
        finally:
            self._busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = TcpJogMarker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
