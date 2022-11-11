cmake_minimum_required(VERSION 3.1.3)
project(roomac_arm)

find_package(catkin REQUIRED
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ArmController.cfg
)

catkin_package()

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/roomac_arm_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/arm_joint_state_controller_node.py
  scripts/arm_joint_trajectory_controller_node.py
  scripts/gripper_joint_trajectory_controller_node.py
  scripts/servo_joint_state_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)