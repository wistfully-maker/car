cmake_minimum_required(VERSION 3.0.2)
project(ucar_slope_test)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/runtime_check.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
