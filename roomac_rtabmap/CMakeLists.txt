cmake_minimum_required(VERSION 3.1.3)
project(roomac_rtabmap)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)