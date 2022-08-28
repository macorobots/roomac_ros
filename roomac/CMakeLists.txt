cmake_minimum_required(VERSION 3.1.3)
project(roomac)

find_package(catkin REQUIRED
dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/AutonomousMissionSettings.cfg
)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/pick_and_bring.test.launch)

  catkin_download_test_data(
    rtabmap_test.db
    https://drive.google.com/uc?export=download&id=1ou8t9e6P6E_EBbSAz9ypjZNlKJi0fJ2K
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    FILENAME rtabmap_test.db
    MD5 435f09db30cd515fa41d68ba2cdac8a8)

  catkin_download_test_data(
    positions_test.yaml
    https://drive.google.com/uc?export=download&id=1RENkEMJ5aNE3fq68uCzo9c_lkohD3slQ
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    FILENAME positions_test.yaml
    MD5 75c7950f5fc5da5c02c183577de97a18)
endif()
