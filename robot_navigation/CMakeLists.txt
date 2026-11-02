cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS src/controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller
)
install(
  PROGRAMS src/pose_estimate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_estimate
)
install(
  PROGRAMS src/waypoint_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_navigation
)
install(
  PROGRAMS src/waypoint_mapping.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_mapping
)

ament_package()
