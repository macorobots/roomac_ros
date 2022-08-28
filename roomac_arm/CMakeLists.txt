cmake_minimum_required(VERSION 3.1.3)
project(roomac_arm)

find_package(catkin REQUIRED
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ArmController.cfg
)

catkin_package()