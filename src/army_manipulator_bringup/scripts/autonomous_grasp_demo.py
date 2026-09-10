#!/usr/bin/env python3
"""Replay a fixed grasp sequence through ros2_control controllers.

This is a hardware checkout/demo node, not an IK node.  It sends the measured
joint targets in this exact order::

    selected grasp_wait preset -> sid -> gripper close -> hold

The manual-override heartbeat prevents ``maru_ik_node`` from commanding the
same controllers during the replay.  ``/picking`` remains true for the whole
sequence and ``/arm/picking_command`` is emitted after a successful HOLD.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from grasp_wait_presets import get_grasp_wait_preset
from joint_calibration import JointCalibration
from move_to_named_pose import (
    ARM_JOINT_NAMES,
    GRIPPER_CURRENT_LSB_MA,
    GRIPPER_CURRENT_THRESHOLD_MA_DEFAULT,
    GRIPPER_JOINT_NAMES,
    NAMED_ARM_JOINTS,
    _close_gripper_with_feedback,
    _send_and_wait,
)


def _arm_target(base: float, joints: dict[str, float]) -> list[float]:
    return [base if name == "base_joint" else joints[name] for name in ARM_JOINT_NAMES]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("autonomous_grasp_demo")
    node.declare_parameter("grasp_wait_preset", "legacy")
    node.declare_parameter("sid_base_rad", -0.02761)
    node.declare_parameter("arm_duration_sec", 3.0)
    node.declare_parameter("gripper_duration_sec", 3.0)
    node.declare_parameter("between_steps_sec", 0.3)
    node.declare_parameter("manual_override_topic", "/control/arm_manual_override")
    node.declare_parameter("manual_override_settle_sec", 0.5)
    node.declare_parameter("picking_state_topic", "/picking")
    node.declare_parameter("grasp_success_topic", "/arm/grasp_success")
    node.declare_parameter("completion_command_topic", "/arm/picking_command")
    node.declare_parameter("startup_completion_topic", "/arm/startup_pose_complete")
    node.declare_parameter(
        "gripper_current_threshold_ma", GRIPPER_CURRENT_THRESHOLD_MA_DEFAULT)

    preset_name = str(node.get_parameter("grasp_wait_preset").value).strip().lower()
    sid_base = float(node.get_parameter("sid_base_rad").value)
    arm_duration = float(node.get_parameter("arm_duration_sec").value)
    gripper_duration = float(node.get_parameter("gripper_duration_sec").value)
    between_steps = max(0.0, float(node.get_parameter("between_steps_sec").value))
    override_topic = str(node.get_parameter("manual_override_topic").value)
    override_settle = max(
        0.0, float(node.get_parameter("manual_override_settle_sec").value))
    picking_topic = str(node.get_parameter("picking_state_topic").value)
    grasp_success_topic = str(node.get_parameter("grasp_success_topic").value)
    completion_topic = str(node.get_parameter("completion_command_topic").value)
    startup_completion_topic = str(node.get_parameter("startup_completion_topic").value)
    current_threshold = float(
        node.get_parameter("gripper_current_threshold_ma").value)

    try:
        grasp_wait = get_grasp_wait_preset(preset_name)
    except ValueError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    if not all(math.isfinite(v) and v > 0.0 for v in (arm_duration, gripper_duration)):
        node.get_logger().error("arm/gripper duration은 0보다 큰 유한값이어야 합니다.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    state = {"base": None, "gripper_position": None, "current_ma": 0.0}

    def on_joint_state(msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name == "base_joint" and math.isfinite(position):
                state["base"] = float(position)
            elif name == "gripper_joint" and math.isfinite(position):
                state["gripper_position"] = float(position)
        try:
            idx = msg.name.index("gripper_joint")
        except ValueError:
            return
        if idx < len(msg.effort) and math.isfinite(msg.effort[idx]):
            state["current_ma"] = abs(float(msg.effort[idx])) * GRIPPER_CURRENT_LSB_MA

    node.create_subscription(
        JointState, "/joint_states", on_joint_state, qos_profile_sensor_data)
    deadline = time.monotonic() + 15.0
    while state["base"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["base"] is None:
        node.get_logger().error("15초 안에 /joint_states의 base_joint를 받지 못했습니다.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    calibration = JointCalibration()
    targets = {
        "grasp_wait": _arm_target(float(state["base"]), grasp_wait),
        "sid": _arm_target(sid_base, NAMED_ARM_JOINTS["sid"]),
        "hold": _arm_target(sid_base, NAMED_ARM_JOINTS["hold"]),
    }
    invalid = []
    for label, target in targets.items():
        for name, value in zip(ARM_JOINT_NAMES, target):
            low, high = calibration.actual_limits(name)
            if not low <= value <= high:
                invalid.append(f"{label}.{name}={value:.5f} ({low:.5f}..{high:.5f})")
    if invalid:
        node.get_logger().error("관절 목표가 calibration limit 밖입니다: " + ", ".join(invalid))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    arm_client = ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    gripper_client = ActionClient(
        node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
    for label, client in (("arm", arm_client), ("gripper", gripper_client)):
        if not client.wait_for_server(timeout_sec=20.0):
            node.get_logger().error(f"{label}_controller action 서버를 찾지 못했습니다.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    latched_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    override_pub = node.create_publisher(Bool, override_topic, latched_qos)
    picking_pub = node.create_publisher(Bool, picking_topic, latched_qos)
    grasp_pub = node.create_publisher(Bool, grasp_success_topic, 10)
    completion_pub = node.create_publisher(Empty, completion_topic, 10)
    startup_pub = node.create_publisher(Empty, startup_completion_topic, latched_qos)
    override_true = Bool(data=True)
    override_pub.publish(override_true)
    heartbeat = node.create_timer(0.2, lambda: override_pub.publish(override_true))
    picking_pub.publish(Bool(data=True))
    settle_deadline = time.monotonic() + override_settle
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, settle_deadline - time.monotonic()))

    node.get_logger().info(
        f"Fixed autonomous demo start: {preset_name} -> sid -> close -> hold")
    ok = True
    try:
        ok = _send_and_wait(
            node, arm_client, "/arm_controller/follow_joint_trajectory",
            ARM_JOINT_NAMES, targets["grasp_wait"], arm_duration,
            f"1/4 grasp_wait ({preset_name})")
        if ok and between_steps:
            time.sleep(between_steps)
        if ok:
            ok = _send_and_wait(
                node, arm_client, "/arm_controller/follow_joint_trajectory",
                ARM_JOINT_NAMES, targets["sid"], arm_duration, "2/4 sid")
        if ok and between_steps:
            time.sleep(between_steps)
        if ok:
            gripper_state = {
                "position": state["gripper_position"],
                "current_ma": state["current_ma"],
            }

            # Keep the dictionary read by the shared close loop synchronized
            # while spin_once processes the subscription callback.
            sync_timer = node.create_timer(
                0.01,
                lambda: gripper_state.update(
                    position=state["gripper_position"], current_ma=state["current_ma"]),
            )
            try:
                ok = _close_gripper_with_feedback(
                    node, gripper_client, gripper_duration, current_threshold,
                    gripper_state, grasp_pub, "3/4 gripper close")
            finally:
                sync_timer.cancel()
        if ok and between_steps:
            time.sleep(between_steps)
        if ok:
            ok = _send_and_wait(
                node, arm_client, "/arm_controller/follow_joint_trajectory",
                ARM_JOINT_NAMES, targets["hold"], arm_duration, "4/4 hold")
        if ok:
            completion_pub.publish(Empty())
            node.get_logger().info(
                f"Fixed autonomous demo complete; published {completion_topic}.")
    finally:
        heartbeat.cancel()
        picking_pub.publish(Bool(data=False))
        override_pub.publish(Bool(data=False))
        if ok:
            startup_pub.publish(Empty())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info("Demo command ownership released.")

    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
