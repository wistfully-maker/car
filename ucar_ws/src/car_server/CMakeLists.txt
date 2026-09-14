cmake_minimum_required(VERSION 3.0.2)
project(car_server)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package(
  CATKIN_DEPENDS rospy
)
include_directories(${catkin_INCLUDE_DIRS})
install(PROGRAMS
  car_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

