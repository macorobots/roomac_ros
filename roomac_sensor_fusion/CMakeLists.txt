cmake_minimum_required(VERSION 3.1.3)
project(roomac_sensor_fusion)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/republish_odom_imu_msgs_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)