cmake_minimum_required(VERSION 3.1.3)
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
  DigitalServoCmd.msg
  AnalogServoCmd.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package()