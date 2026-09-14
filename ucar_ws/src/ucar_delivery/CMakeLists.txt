cmake_minimum_required(VERSION 3.0.2)
project(ucar_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/delivery_mission_node.py
  scripts/sign_adapter_node.py
  scripts/frame_detector_node.py
  scripts/parking_controller_node.py
  scripts/delivery_velocity_mux_node.py
  scripts/mock_goal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_nosetests(test)
