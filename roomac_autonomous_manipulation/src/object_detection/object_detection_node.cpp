#include <ros/ros.h>

#include <roomac_autonomous_manipulation/object_detection/ObjectDetection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  ObjectDetection object_detection;
  ros::spin();
}