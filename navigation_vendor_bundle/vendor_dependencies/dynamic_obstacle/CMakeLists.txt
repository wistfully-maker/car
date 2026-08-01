cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  costmap_converter
  message_generation
)

add_message_files(
  FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  costmap_converter
  message_runtime
)

catkin_install_python(PROGRAMS
  scripts/dynamic_obstacle_tracker.py
  scripts/safety_monitor.py
  scripts/test_synthetic_scan.py
  scripts/test_safety_monitor.py
  scripts/fake_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
