cmake_minimum_required(VERSION 3.0.2)
project(line_follow_integration)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/line_camera_adapter_node.py
  scripts/line_navigation_adapter_node.py
  scripts/line_follow_supervisor_node.py
  scripts/yolo_server.py
  scripts/follow_left_v4.py
  scripts/follow_right_v4.py
  scripts/follow_mid_v4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
