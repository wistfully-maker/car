cmake_minimum_required(VERSION 3.0.2)
project(task_orchestrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/task_orchestrator_node.py
  scripts/voice_task_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

