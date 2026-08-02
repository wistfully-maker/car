cmake_minimum_required(VERSION 3.0.2)
project(task_orchestrator)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosnode
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/task_orchestrator_node.py
  scripts/voice_task_adapter_node.py
  scripts/tts_bridge_node.py
  scripts/fast_nav_adapter_node.py
  scripts/system_readiness_gate_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

