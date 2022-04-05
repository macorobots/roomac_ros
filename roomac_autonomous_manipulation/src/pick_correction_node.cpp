#include <ros/ros.h>

#include <roomac_autonomous_manipulation/PickCorrection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "picking_correction");
  ros::NodeHandle nh;
  PickCorrection pick;
  ros::spin();
}