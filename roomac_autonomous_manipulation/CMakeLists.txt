cmake_minimum_required(VERSION 3.1.3)
project(roomac_autonomous_manipulation)

find_package(catkin REQUIRED
  roscpp
  tf2_ros
  dynamic_reconfigure
  roomac_msgs
)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PickCorrection.cfg
  cfg/PickManagerSettings.cfg
  cfg/ObjectDetection.cfg
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(pick_correction_node src/pick_correction/pick_correction_node.cpp 
  src/pick_correction/PickCorrection.cpp)

add_executable(object_detection_node src/object_detection/object_detection_node.cpp 
  src/object_detection/ObjectDetection.cpp)

add_dependencies(pick_correction_node ${PROJECT_NAME}_gencfg)

add_dependencies(object_detection_node ${PROJECT_NAME}_gencfg)

target_link_libraries(pick_correction_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(object_detection_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/object_detection.test.launch)

  catkin_download_test_data(
    object_detection.bag
    https://drive.google.com/uc?export=download&id=11ZYm22SqDvrZLV93Mk8w1_hYYJ704uD-
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME object_detection.bag
    MD5 df7dd49f633579e19f86939bb22710bd)

  catkin_download_test_data(
    object_detection_correct_values.yaml
    https://drive.google.com/uc?export=download&id=1Y1BpCiKwLI53tSGplEXh6aXC6h_d_jLQ
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME object_detection_correct_values.yaml
    MD5 e6fe69060dc5942ebf50254b18ee2616)
endif()