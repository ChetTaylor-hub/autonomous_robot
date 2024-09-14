cmake_minimum_required(VERSION 3.0.2)
project(autonomous_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  ObjectDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main_controller.py
  scripts/slam_controller.py
  scripts/object_detection.py
  scripts/navigation_controller.py
  scripts/arm_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)