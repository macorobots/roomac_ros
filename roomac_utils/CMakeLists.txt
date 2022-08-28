# todo: diffferent version?
cmake_minimum_required(VERSION 3.0.2)
project(roomac_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()