#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <string>
#include <tuple>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>

#include <roomac_autonomous_manipulation/ObjectDetectionConfig.h>
#include <roomac_msgs/DetectObjectAndTable.h>
#include <roomac_msgs/ObjectAndTable.h>
#include <roomac_msgs/ObjectWithBoundingBox.h>

class ObjectDetection
{
public:
  ObjectDetection();

private:
  // TODO add briefs

  bool HandleObjectDetection(roomac_msgs::DetectObjectAndTable::Request& req,
                             roomac_msgs::DetectObjectAndTable::Response& res);

  roomac_msgs::ObjectAndTable FindObjectAndTable();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr RangeFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point>
  TablePlaneDetection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterObjectsOnTable(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                              geometry_msgs::Point min_point_table,
                                                              geometry_msgs::Point max_point_table);

  std::tuple<geometry_msgs::Point, geometry_msgs::Point>
  CalculateBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  geometry_msgs::Point CalculateMassCenter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr DetectObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  void PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, ros::Publisher& pub);

  void ReconfigureCallback(roomac_autonomous_manipulation::ObjectDetectionConfig& config, uint32_t level);

  int num_of_readings_for_average_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  bool publish_debug_;

  float max_range_;
  float table_detection_distance_threshold_;

  std::string camera_frame_;

  ros::ServiceServer service_;

  ros::Publisher floor_clipped_pub_;
  ros::Publisher without_table_pub_;
  ros::Publisher only_objects_on_table_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher object_cloud_pub_;
  ros::Publisher object_pt_pub_;

  dynamic_reconfigure::Server<roomac_autonomous_manipulation::ObjectDetectionConfig> reconfigure_server_;
};

#endif