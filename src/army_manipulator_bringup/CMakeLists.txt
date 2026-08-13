cmake_minimum_required(VERSION 3.8)
project(army_manipulator_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/target_detector_node.py
    scripts/fake_target_publisher.py
    scripts/maru_ik_node.py
    scripts/planned_encoder_trajectory.py
    scripts/tcp_trail_publisher.py
    scripts/joint_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
