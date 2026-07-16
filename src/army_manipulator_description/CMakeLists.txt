cmake_minimum_required(VERSION 3.8)
project(army_manipulator_description LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
