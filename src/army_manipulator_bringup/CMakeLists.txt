cmake_minimum_required(VERSION 3.8)
project(army_manipulator_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/maru_ik_node.py
    scripts/target_detector_node.py
    scripts/fake_target_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# joint_calibration.py는 실행 파일이 아니라 maru_ik_node.py가 import하는
# 모듈이므로, 같은 디렉터리(lib/${PROJECT_NAME})에 나란히 설치해 sys.path
# 해석이 되도록 한다.
install(
  FILES scripts/joint_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
