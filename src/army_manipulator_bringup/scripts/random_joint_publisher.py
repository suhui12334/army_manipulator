#!/usr/bin/env python3
"""임의의 관절각(joint-space)으로 팔을 반복 이동시키는 테스트/데이터수집용 노드.

random_target_publisher.py는 Cartesian 좌표를 뽑아 IK로 풀어야 하니 "혹시
도달 불가능하지 않을까"를 매번 compute_ik로 검증해야 했다. 이 노드는 관절각
자체를 직접 뽑아 MoveIt2.move_to_configuration()으로 보내므로 IK가 필요
없고, 관절 리미트 안에서만 뽑으면 항상 도달 가능한 목표다(단, move_group이
충돌 검사는 그대로 함 - 자기충돌 나는 조합이면 플래닝이 실패할 수 있음).

관절 리미트는 army_manipulator_description/config/joint_calibration.yaml의
raw_min/raw_max(-zero_offset)를 그대로 읽어서 쓴다 - 여기 하드코딩하면
나중에 실측 캘리브레이션이 갱신될 때 둘이 어긋난다.

[의도] 나중에 IK seed 캐시(각 관절해 -> FK로 구한 pose를 저장해두고 새 목표가
오면 가장 가까운 pose의 관절해를 seed로 우선 시도)를 만들 때, 이 노드로 얻은
"항상 유효한 관절 샘플" 궤적이 학습/캐시용 데이터 소스가 될 수 있다.
"""

import math
import random
import threading
import time

from ament_index_python.packages import get_package_share_directory
import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State

ARM_JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]


def load_joint_limits(calibration_path: str):
    with open(calibration_path, "r") as f:
        calib = yaml.safe_load(f)
    limits = {}
    for name in ARM_JOINT_NAMES:
        joint = calib["joints"][name]
        zero_offset = joint["zero_offset"]
        limits[name] = (joint["raw_min"] - zero_offset, joint["raw_max"] - zero_offset)
    return limits


class RandomJointPublisher(Node):
    def __init__(self):
        super().__init__("random_joint_publisher")

        default_calib_path = (
            get_package_share_directory("army_manipulator_description")
            + "/config/joint_calibration.yaml"
        )
        self.declare_parameter("joint_calibration_path", default_calib_path)
        self.declare_parameter("group_name", "arm")
        # 이 노드는 move_to_configuration()(joint-space)만 써서 지금은
        # planning_frame이 실질적으로 안 쓰이지만, MoveIt2() 생성자에
        # base_link_name으로 그대로 전달되므로 다른 army_manipulator arm
        # 스크립트(maru_ik_node.py 등, commit 686ee47)와 동일하게 회전 프레임인
        # base_link 대신 고정 루트 base_actuator를 기본값으로 맞춰둔다.
        self.declare_parameter("planning_frame", "base_actuator")
        # [안전] 실측 리미트 정확히 끝까지 붙으면 실기에서 하드스톱/리미트
        # 스위치에 닿을 수 있다 - 양쪽 끝에서 이 비율만큼 안쪽으로 좁혀서 뽑는다.
        self.declare_parameter("limit_margin_ratio", 0.9)
        self.declare_parameter("settle_time_sec", 1.5)
        self.declare_parameter("motion_timeout_sec", 20.0)
        self.declare_parameter("delay_sec", 3.0)
        self.declare_parameter("repeat", True)
        self.declare_parameter("rate_hz", 0.2)
        self.declare_parameter("seed", 0)  # 0이면 랜덤, 그 외 값이면 재현 가능한 고정 시드

        calib_path = str(self.get_parameter("joint_calibration_path").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.margin_ratio = float(self.get_parameter("limit_margin_ratio").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.delay_sec = float(self.get_parameter("delay_sec").value)
        self.repeat = bool(self.get_parameter("repeat").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        seed = int(self.get_parameter("seed").value)

        raw_limits = load_joint_limits(calib_path)
        self.joint_ranges = {}
        for name, (lo, hi) in raw_limits.items():
            mid = (lo + hi) / 2.0
            half = (hi - lo) / 2.0 * self.margin_ratio
            self.joint_ranges[name] = (mid - half, mid + half)

        self._rng = random.Random(seed if seed != 0 else None)
        # [수정] callback_group을 안 주면 기본(MutuallyExclusive)이라 move_group
        # action의 내부 콜백들이 서로 걸려 move_to_configuration()이 영영 IDLE로
        # 안 돌아오는 걸 확인함 - maru_ik_node.py와 동일하게 Reentrant로 맞춘다.
        callback_group = ReentrantCallbackGroup()
        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINT_NAMES, base_link_name=self.planning_frame,
            end_effector_name="wrist_link", group_name=self.group_name,
            callback_group=callback_group,
            use_move_group_action=True, ignore_new_calls_while_executing=True,
        )

        ranges_str = ", ".join(
            f"{n}=[{lo:.3f},{hi:.3f}]" for n, (lo, hi) in self.joint_ranges.items())
        self.get_logger().info(
            f"random_joint_publisher ready ({ranges_str}); starting in {self.delay_sec:.1f}s "
            f"(repeat={self.repeat})."
        )
        threading.Thread(target=self._run, daemon=True).start()

    def _sample_joints(self):
        return [self._rng.uniform(*self.joint_ranges[name]) for name in ARM_JOINT_NAMES]

    def _move_once(self) -> bool:
        target = self._sample_joints()
        self.get_logger().info(
            "Moving to random joint config: "
            + ", ".join(f"{n}={v:.4f}" for n, v in zip(ARM_JOINT_NAMES, target))
        )
        self.arm.move_to_configuration(target)

        deadline = time.monotonic() + self.motion_timeout_sec
        last_state = None
        while self.arm.query_state() != MoveIt2State.IDLE:
            state = self.arm.query_state()
            if state != last_state:
                self.get_logger().info(f"  state: {state}")
                last_state = state
            if time.monotonic() >= deadline:
                self.get_logger().error("Motion timed out; cancelling.")
                self.arm.cancel_execution()
                return False
            time.sleep(0.05)

        if not self.arm.motion_suceeded:
            self.get_logger().warn("Motion failed in MoveIt (likely self-collision at this sample).")
            return False
        return True

    def _run(self) -> None:
        time.sleep(self.delay_sec)
        self._move_once()
        if not self.repeat:
            return
        period = 1.0 / self.rate_hz
        while rclpy.ok():
            time.sleep(max(period - self.settle_time_sec, 0.0))
            self._move_once()
            time.sleep(self.settle_time_sec)


def main(args=None):
    rclpy.init(args=args)
    node = RandomJointPublisher()
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
