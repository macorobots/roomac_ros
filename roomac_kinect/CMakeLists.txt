cmake_minimum_required(VERSION 3.1.3)
project(roomac_kinect)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)