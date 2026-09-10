cmake_minimum_required(VERSION 3.8)
project(army_manipulator_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/detection_status_monitor.py
    scripts/fake_target_publisher.py
    scripts/random_target_publisher.py
    scripts/random_joint_publisher.py
    scripts/tcp_jog_marker.py
    scripts/cycle_monitor.py
    scripts/gripper_current_logger.py
    scripts/gripper_safe_close.py
    scripts/maru_ik_node.py
    scripts/planned_encoder_trajectory.py
    scripts/tcp_trail_publisher.py
    scripts/joint_calibration.py
    scripts/grasp_wait_presets.py
    scripts/arm_pose_array_publisher.py
    scripts/capture_arm_pose.py
    scripts/move_to_named_pose.py
    scripts/autonomous_grasp_demo.py
    scripts/ik_node.py
    scripts/verify_grasp_pose_fk.py
    scripts/trigger_captured_grasp.py
    scripts/check_self_collision.py
    scripts/check_reachability.py
    scripts/sid_fallback_grasp.py
    scripts/numerical_grasp_planner.py
    scripts/analytic_grasp_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
