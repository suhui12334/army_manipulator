#!/usr/bin/env python3
"""그리퍼 전류를 실시간으로 찍고 CSV로 남겨서 gripper_current_threshold_ma
실측에 쓰는 도구.

maru_ik_node.py의 _close_gripper_with_current_feedback()과 정확히 같은 변환
(raw Present Current * gripper_current_lsb_ma)을 써서, 거기서 보게 될 값과
여기서 보는 값이 항상 일치하게 맞춰뒀다.

쓰는 법: 빈 손으로 그리퍼를 몇 번 완전히 닫아보고(= "물체 없이 닫힘" 전류
곡선), 그 다음 실제 서플라이박스를 잡아보고(= "물체 잡음" 전류 곡선) 두
CSV를 비교해서, 전자의 최고점보다는 확실히 높고 후자의 상승 시작점보다는
낮은 지점으로 gripper_current_threshold_ma를 잡으면 된다.

터미널에는 값이 바뀔 때마다(또는 print_hz 주기로) 한 줄씩 찍히고, CSV에는
매 /joint_states 메시지가 다 기록된다(터미널 rate limit과 무관하게 전체
해상도 유지). Ctrl+C로 종료하면 이번 세션의 최고 전류값 요약이 뜬다.

실시간 그래프: /arm/gripper_current_ma(std_msgs/Float32)에도 매 샘플을 그대로
발행한다. JECS는 SSH 전용이라 GUI가 없으니, 같은 저장소가 있는 로컬 머신에서
(같은 ROS_DOMAIN_ID로) 아래처럼 띄우면 네트워크 너머로 실시간 그래프를 볼 수
있다:
  ros2 run rqt_plot rqt_plot /arm/gripper_current_ma/data
"""

import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class GripperCurrentLogger(Node):
    def __init__(self):
        super().__init__("gripper_current_logger")

        self.declare_parameter("csv_path", "/tmp/gripper_current_log.csv")
        self.declare_parameter("print_hz", 10.0)
        self.declare_parameter("gripper_current_lsb_ma", 2.69)
        self.declare_parameter("joint_name", "gripper_joint")

        self.csv_path = str(self.get_parameter("csv_path").value)
        self.lsb_ma = float(self.get_parameter("gripper_current_lsb_ma").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        print_hz = float(self.get_parameter("print_hz").value)
        self.print_interval = 1.0 / print_hz if print_hz > 0 else 0.0

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_sec", "position_rad", "effort_raw", "current_ma"])

        self.start_time = time.monotonic()
        self.last_print = 0.0
        self.peak_current_ma = 0.0
        self.sample_count = 0

        self.current_pub = self.create_publisher(Float32, "/arm/gripper_current_ma", 10)
        self.create_subscription(
            JointState, "/joint_states", self.on_state, qos_profile_sensor_data)

        self.get_logger().info(
            f"gripper_current_logger ready: joint='{self.joint_name}', "
            f"LSB={self.lsb_ma}mA/count, CSV -> {self.csv_path}, "
            "live plot -> /arm/gripper_current_ma (로컬에서 rqt_plot으로 구독). "
            "빈 손으로 몇 번, 물체 잡고 몇 번 닫아서 곡선을 비교하세요."
        )

    def on_state(self, msg: JointState) -> None:
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return
        if idx >= len(msg.effort) or idx >= len(msg.position):
            return

        position = msg.position[idx]
        effort_raw = msg.effort[idx]
        # maru_ik_node.py의 on_joint_states()와 동일한 변환.
        current_ma = abs(effort_raw) * self.lsb_ma

        t = time.monotonic() - self.start_time
        self.csv_writer.writerow([f"{t:.4f}", f"{position:.5f}", f"{effort_raw:.4f}", f"{current_ma:.2f}"])
        self.current_pub.publish(Float32(data=current_ma))
        self.sample_count += 1
        if current_ma > self.peak_current_ma:
            self.peak_current_ma = current_ma

        now = time.monotonic()
        if now - self.last_print >= self.print_interval:
            self.last_print = now
            bar_len = min(int(current_ma / 10.0), 60)
            bar = "#" * bar_len
            print(f"[{t:7.2f}s] pos={position:7.4f}rad  current={current_ma:7.1f}mA "
                  f"peak={self.peak_current_ma:7.1f}mA  {bar}")

    def destroy_node(self) -> None:
        self.csv_file.close()
        print(f"\n=== 종료: {self.sample_count}개 샘플, 이번 세션 최고 전류 "
              f"{self.peak_current_ma:.1f}mA, CSV: {self.csv_path} ===")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperCurrentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
