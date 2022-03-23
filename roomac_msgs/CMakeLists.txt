cmake_minimum_required(VERSION 3.0.2)
project(roomac_msgs)

find_package(catkin REQUIRED genmsg actionlib_msgs)
add_action_files(DIRECTORY action FILES PickAndBring.action)
generate_messages(DEPENDENCIES actionlib_msgs)