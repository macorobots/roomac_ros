cmake_minimum_required(VERSION 3.1.3)
project(roomac_ar_tag)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/robot_pose_detection.test.launch)

  catkin_download_test_data(
    robot_pose_detection.bag
    https://drive.google.com/uc?export=download&id=192hhncAiv3lk4J8pc66VfoaUqSF8LMxD
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME robot_pose_detection.bag
    MD5 286e9025527307639e7b3e3bcbe26492)

  catkin_download_test_data(
    robot_pose_detection_correct_values.yaml
    https://drive.google.com/uc?export=download&id=1lYCZDSI3hSLt_jXhcQU7ufLMgmK-dTWj
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME robot_pose_detection_correct_values.yaml
    MD5 2ed3f868253a6aae20e871ce7c3c9e14)
endif()