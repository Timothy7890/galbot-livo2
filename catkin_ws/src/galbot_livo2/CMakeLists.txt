cmake_minimum_required(VERSION 3.0.2)
project(galbot_livo2)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/sdk_bridge.py scripts/gen_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
