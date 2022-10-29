cmake_minimum_required(VERSION 3.1.3)
project(roomac_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)