cmake_minimum_required(VERSION 3.1.3)
project(roomac_bringup)

find_package(catkin REQUIRED
  dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/PickAndBringManagerSettings.cfg
)

catkin_package()

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/pick_and_bring_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pick_and_bring_manager_node.py
  scripts/pose_publisher_node.py
  scripts/ros_mobile_bridge_node.py
  scripts/wheel_joint_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Full integration test in gazebo is disabled by default - tests are mainly used in github action
# and because of camera, gazebo tests require GPU, which isn't provided in github actions and in result
# test doesn't work
# It can be enabled by passing additional cmake argument:
# catkin run_tests --cmake-args -DENABLE_ROOMAC_SIMULATION_TESTS=True
if(CATKIN_ENABLE_TESTING AND ENABLE_ROOMAC_SIMULATION_TESTS)
  find_package(rostest REQUIRED)
  add_rostest(test/pick_and_bring.test.launch)

  catkin_download_test_data(
    rtabmap_test.db
    https://drive.google.com/uc?export=download&id=1ou8t9e6P6E_EBbSAz9ypjZNlKJi0fJ2K
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME rtabmap_test.db
    MD5 435f09db30cd515fa41d68ba2cdac8a8)

  catkin_download_test_data(
    positions_test.yaml
    https://drive.google.com/uc?export=download&id=1RENkEMJ5aNE3fq68uCzo9c_lkohD3slQ
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME positions_test.yaml
    MD5 75c7950f5fc5da5c02c183577de97a18)
endif()
