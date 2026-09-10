#!/usr/bin/env python3
"""매 제어 사이클마다 실제 엔코더 값을 터미널에 나란히 찍고 이상을 감지한다.

실기에서 "매 사이클 CAN으로 보내는 엔코더 값이 제대로 나오는지"를 눈으로
바로 확인하기 위한 도구. /joint_states(실제 엔코더 피드백)를 구독해서:
  - 사이클 정지(stall): 예상 주기보다 STALL_MULTIPLIER배 이상 늦게 메시지가 오면 경고
  - 순간 점프: 관절 하나가 물리적으로 불가능한 속도(MAX_PLAUSIBLE_RAD_PER_SEC)로
    튀면 경고 (배선/인코더 노이즈 의심 신호)
  - NaN/Inf: 유한하지 않은 값이 오면 경고
을 감지하면서, 매 print_interval마다 전체 관절 값을 한 줄로 출력한다.

theoretical_state_relay.py가 떠 있으면(/theoretical_joint_states 발행 중)
"이론상 이 순간 있어야 할 위치(reference)"도 같은 줄에 나란히 찍어서, RViz
고스트 오버레이 없이 터미널만 보고도 실제값과 이론값 차이를 바로 비교할 수
있다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINTS = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint"]
MAX_PLAUSIBLE_RAD_PER_SEC = 20.0  # 급격한 순간 점프 판정 임계값 - 실측 아니고 러프한 안전판.
STALL_MULTIPLIER = 3.0
CURRENT_LSB_MA = 2.69  # gripper_joint effort(raw) -> mA


class CycleMonitor(Node):
    def __init__(self):
        super().__init__("cycle_monitor")

        self.declare_parameter("print_hz", 5.0)
        self.declare_parameter("theoretical_topic", "/theoretical_joint_states")
        print_hz = float(self.get_parameter("print_hz").value)
        theoretical_topic = str(self.get_parameter("theoretical_topic").value)

        self.last_msg_time = None
        self.last_positions = {}
        self.theoretical_positions = {}
        self.count = 0
        self.anomaly_count = 0
        self.rate_ema = None
        self.print_interval = 1.0 / print_hz
        self.last_print = 0.0

        self.create_subscription(
            JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.create_subscription(
            JointState, theoretical_topic, self.on_theoretical, 10)

        self.get_logger().info(
            f"cycle_monitor ready: /joint_states 매 사이클 감시, "
            f"{theoretical_topic}가 있으면 이론값도 같이 표시."
        )

    def on_theoretical(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.theoretical_positions[name] = pos

    def on_state(self, msg: JointState) -> None:
        now = time.monotonic()
        self.count += 1
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        dt = None
        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
            inst_hz = 1.0 / dt if dt > 0 else float("inf")
            self.rate_ema = inst_hz if self.rate_ema is None else 0.9 * self.rate_ema + 0.1 * inst_hz
            expected_period = 1.0 / self.rate_ema if self.rate_ema else 0.01
            if dt > expected_period * STALL_MULTIPLIER:
                self.anomaly_count += 1
                print(f"[ANOMALY] cycle stall: {dt*1000:.1f}ms since last /joint_states "
                      f"(expected ~{expected_period*1000:.1f}ms)")
        self.last_msg_time = now

        row = {}
        for j in JOINTS:
            if j not in name_to_idx:
                self.anomaly_count += 1
                print(f"[ANOMALY] joint '{j}' missing from /joint_states")
                continue
            idx = name_to_idx[j]
            pos = msg.position[idx] if idx < len(msg.position) else float("nan")
            vel = msg.velocity[idx] if idx < len(msg.velocity) else float("nan")
            eff = msg.effort[idx] if idx < len(msg.effort) else 0.0

            if not math.isfinite(pos) or not math.isfinite(vel):
                self.anomaly_count += 1
                print(f"[ANOMALY] {j}: non-finite state (pos={pos}, vel={vel})")

            if j in self.last_positions and dt and dt > 0 and math.isfinite(pos):
                d_rad_per_sec = abs(pos - self.last_positions[j]) / dt
                if d_rad_per_sec > MAX_PLAUSIBLE_RAD_PER_SEC:
                    self.anomaly_count += 1
                    print(f"[ANOMALY] {j}: implausible jump {d_rad_per_sec:.1f} rad/s "
                          f"(pos {self.last_positions[j]:.4f} -> {pos:.4f} in {dt*1000:.1f}ms)")
            if math.isfinite(pos):
                self.last_positions[j] = pos

            row[j] = (pos, vel, eff)

        if now - self.last_print >= self.print_interval:
            self.last_print = now
            hz = f"{self.rate_ema:5.1f}Hz" if self.rate_ema else "  --  "
            cols = []
            for j in JOINTS:
                short = j.replace("_joint", "")
                if j not in row:
                    cols.append(f"{short:>9}=  N/A")
                    continue
                pos, vel, eff = row[j]
                theo = self.theoretical_positions.get(j)
                theo_str = f"(이론={theo:7.4f})" if theo is not None else ""
                if j == "gripper_joint":
                    cols.append(f"{short:>9}={pos:7.4f}rad{theo_str} ({eff*CURRENT_LSB_MA:6.1f}mA)")
                else:
                    cols.append(f"{short:>9}={pos:7.4f}rad{theo_str}")
            print(f"[{hz}] msgs={self.count:6d} anomalies={self.anomaly_count:3d} | " + "  ".join(cols))


def main(args=None):
    rclpy.init(args=args)
    node = CycleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.count == 0:
            print("[ANOMALY] /joint_states에서 메시지를 단 한 번도 못 받음 (토픽 없음/퍼블리셔 미기동)")
        print(f"\n=== 종료: 총 {node.count}개 메시지, 이상 {node.anomaly_count}건 ===")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
