cmake_minimum_required(VERSION 3.0.2)
project(roomac_msgs)

find_package(catkin REQUIRED
  genmsg
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_action_files(DIRECTORY action FILES PickAndBring.action PickObject.action)

add_service_files(
  FILES
  DetectGripperPosition.srv
  DetectObjectAndTable.srv
)

add_message_files(
  FILES
  ObjectAndTable.msg
  ObjectWithBoundingBox.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()