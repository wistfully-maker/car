cmake_minimum_required(VERSION 3.0.2)
project(qr_item_search)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/qr_scanner_node.py
  scripts/item_search_controller_node.py
  scripts/qr_debug_stream_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
