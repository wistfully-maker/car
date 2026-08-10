cmake_minimum_required(VERSION 2.8.3)
project(stop)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
)

# ---- 自定义消息 (BoundingBox/BoundingBoxes, 兼容 rknn 包) ----
add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
)

# ---- 安装脚本 ----
install(PROGRAMS
  scripts/scan_and_park.py
  scripts/precision_park.py
  scripts/ocr_native_node.py
  scripts/mission_orchestrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 OCR 模块和模型
install(DIRECTORY
  scripts/ocr
  scripts/models
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  wav
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
