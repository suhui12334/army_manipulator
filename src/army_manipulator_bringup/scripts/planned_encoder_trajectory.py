#!/usr/bin/env python3
"""MoveIt 계획 경로를 raw encoder-radian 경로로 변환해 관측용 토픽에 발행한다.

``/display_planned_path``는 move_group이 RViz에도 보내는 계획 결과다. 이 노드는
그 안의 actual(URDF) 관절각을 ``joint_calibration.yaml``의 zero_offset으로
raw encoder-radian으로 변환해 ``/arm/planned_encoder_trajectory``에 발행한다.

이 노드는 CAN 프레임을 직접 보내지 않는다. 실제 명령 경로는 반드시
MoveIt -> arm_controller -> ros2_control RMD/Dynamixel hardware interface -> CAN
하나만 유지해야 한다. 별도 송신기를 병렬로 두면 같은 모터에 충돌 명령이 간다.
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.msg import DisplayTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ModuleNotFoundError:  # pragma: no cover - permits pure-python import in CI
    rclpy = None
    Node = object

from joint_calibration import JointCalibration


class PlannedEncoderTrajectory(Node):
    """Publish MoveIt arm trajectories in the hardware raw-angle convention."""

    def __init__(self) -> None:
        super().__init__("planned_encoder_trajectory")
        self.declare_parameter("display_trajectory_topic", "/display_planned_path")
        self.declare_parameter("encoder_trajectory_topic", "/arm/planned_encoder_trajectory")
        self.declare_parameter(
            "joint_names", ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
        )
        self.display_topic = str(self.get_parameter("display_trajectory_topic").value)
        self.encoder_topic = str(self.get_parameter("encoder_trajectory_topic").value)
        self.joint_names = tuple(str(name) for name in self.get_parameter("joint_names").value)
        self.calibration = JointCalibration()
        self.publisher = self.create_publisher(JointTrajectory, self.encoder_topic, 10)
        self.create_subscription(DisplayTrajectory, self.display_topic, self._on_plan, 10)
        self.get_logger().info(
            f"Observing {self.display_topic}; raw encoder radians -> {self.encoder_topic}"
        )

    def _on_plan(self, msg: DisplayTrajectory) -> None:
        for robot_trajectory in msg.trajectory:
            source = robot_trajectory.joint_trajectory
            if not source.points or not source.joint_names:
                continue
            source_index = {name: index for index, name in enumerate(source.joint_names)}
            if not all(name in source_index for name in self.joint_names):
                continue

            encoded = JointTrajectory()
            encoded.header = source.header
            # JointTrajectory has no unit field; frame_id makes the unit/convention
            # explicit for ros2 topic echo, loggers, and downstream bridges.
            encoded.header.frame_id = "raw_encoder_radians"
            encoded.joint_names = list(self.joint_names)
            for source_point in source.points:
                point = JointTrajectoryPoint()
                point.positions = [
                    self.calibration.actual_to_raw(name, source_point.positions[source_index[name]])
                    for name in self.joint_names
                ]
                point.time_from_start = source_point.time_from_start
                if source_point.velocities:
                    point.velocities = [source_point.velocities[source_index[name]] for name in self.joint_names]
                if source_point.accelerations:
                    point.accelerations = [source_point.accelerations[source_index[name]] for name in self.joint_names]
                encoded.points.append(point)
            self.publisher.publish(encoded)
            self.get_logger().info(
                f"MoveIt plan: {len(encoded.points)} points -> raw encoder trajectory"
            )


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 dependencies are not installed")
    rclpy.init(args=args)
    node = PlannedEncoderTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
