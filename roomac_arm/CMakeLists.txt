cmake_minimum_required(VERSION 2.8.3)
project(roomac_arm)

find_package(catkin REQUIRED
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ArmController.cfg
)

catkin_package()