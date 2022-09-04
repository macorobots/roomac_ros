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