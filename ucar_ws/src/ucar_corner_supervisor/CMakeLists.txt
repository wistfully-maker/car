cmake_minimum_required(VERSION 3.0.2)
project(ucar_corner_supervisor)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  std_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/corner_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
