#ifndef PICK_CORRECTION_H
#define PICK_CORRECTION_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/Trigger.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <string>

class PickCorrection
{

public:
  PickCorrection();

private:
  bool HandlePickCorrection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneClip(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, const Eigen::Vector4f &plane, bool negative);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr DetectClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud);
  geometry_msgs::Transform ArtagTransform();

  void PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, ros::Publisher &pub);
  geometry_msgs::Point AnalyzeCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster);

  geometry_msgs::Point FindGripperPosition();
  Eigen::Vector4f CalculatePlane(geometry_msgs::Transform trans);

  int num_of_readings_for_average_;
  float z_offset_pointcloud_split_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  float elbow_gripper_offset_;
  bool publish_debug_;
  float offset_planning_gripper_point_;

  std::string camera_frame_;
  std::string artag_frame_;

  ros::ServiceServer service_;
  ros::Publisher cloud_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher cluster_analyzed_pub_;
  ros::Publisher gripper_pt_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif