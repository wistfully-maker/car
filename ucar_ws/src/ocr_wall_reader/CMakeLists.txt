cmake_minimum_required(VERSION 2.8.3)
project(ocr_wall_reader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS scripts/ocr_wall_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
