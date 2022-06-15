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
  /**
   * @brief Handles service calls
   */
  bool HandleObjectDetection(roomac_msgs::DetectObjectAndTable::Request& req,
                             roomac_msgs::DetectObjectAndTable::Response& res);

  /**
   * @brief Gets most recent pointcloud and detects table and object for picking
   *
   * @return object and table (mass center, min and max point of bounding box)
   * @exception std::runtime_error when there was error during processing
   */
  roomac_msgs::ObjectAndTable FindObjectAndTable();

  /**
   * @brief Filters pointcloud based on z axis (using max_range_ parameter)
   *
   * @param src_cloud
   * @param max_range
   * @return filtered pointcloud
   * @exception std::runtime_error when pointcloud is empty
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr RangeFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                     float max_range);

  /**
   * @brief Detects table plane on given pointcloud
   *
   * @param src_cloud
   * @return tuple of pointcloud without table, table mass center,
   * table bounding box min point and max point
   * @exception std::runtime_error when pointcloud is empty or couldn't estimate model
   */
  std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point>
  DetectTablePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  /**
   * @brief Computes points that are above height of table + objects_on_table_min_height and within tables rectangle
   *
   * @param src_cloud
   * @param min_point_table
   * @param max_point_table
   * @return filtered pointcloud that include only objects points
   * @exception std::runtime_error when pointcloud is empty
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterObjectsOnTable(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                              geometry_msgs::Point min_point_table,
                                                              geometry_msgs::Point max_point_table);

  /**
   * @brief Calculates average of vector of points
   *
   * @param point_readings
   * @return average point
   */
  geometry_msgs::Point CalculateAverage(const std::vector<geometry_msgs::Point>& point_readings);

  /**
   * @brief Calculates average of vector of ObjectWithBoundingBox
   *
   * @param object_readings
   * @return average ObjectWithBoundingBox (average mass center, min_point, max_point)
   */
  roomac_msgs::ObjectWithBoundingBox
  CalculateAverage(const std::vector<roomac_msgs::ObjectWithBoundingBox>& object_readings);

  /**
   * @brief Calculates average of vector of ObjectAndTable
   *
   * @param object_and_table_readings
   * @return average ObjectWithBoundingBox
   */
  roomac_msgs::ObjectAndTable
  CalculateAverage(const std::vector<roomac_msgs::ObjectAndTable>& object_and_table_readings);

  /**
   * @brief Calculates bounding box around given pointcloud
   *
   * @param src_cloud
   * @return tuple with max_point and min_point of bounding box
   * @exception std::runtime_error when pointcloud is empty
   */
  std::tuple<geometry_msgs::Point, geometry_msgs::Point>
  CalculateBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  /**
   * @brief Calculates mass center of given pointcloud
   *
   * @param src_cloud
   * @return mass center point
   * @exception std::runtime_error when pointcloud is empty
   */
  geometry_msgs::Point CalculateMassCenter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  /**
   * @brief Filters pointcloud using voxel filter
   *
   * @param src_cloud
   * @param leaf_size
   * @return filtered cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr VoxelFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                     float leaf_size);

  /**
   * @brief Performs clustering and returns clusters in form of
   * indeces of points in original pointcloud
   *
   * @param src_cloud source cloud for clustering
   * @param cluster_tolerance euclidean clustering parameters
   * @param min_cluster_size euclidean clustering parameters
   * @param max_cluster_size euclidean clustering parameters
   * @return groups of indeces of points in original pointcloud
   * @exception std::runtime_error when pointcloud is empty
   */
  std::vector<pcl::PointIndices> DetectClusterIndices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                      float cluster_tolerance, int min_cluster_size,
                                                      int max_cluster_size);

  /**
   * @brief Detects objects pointcloud - computes clusters and selects one with minimum
   * y coordinate of mass center - closest to the edge of the table
   *
   * @param src_cloud source cloud for detection
   * @param cluster_tolerance euclidean clustering parameters
   * @param min_cluster_size euclidean clustering parameters
   * @param max_cluster_size euclidean clustering parameters
   * @return detected object pointcloud
   * @exception std::runtime_error when pointcloud is empty
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr DetectObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                             float cluster_tolerance, int min_cluster_size,
                                                             int max_cluster_size,
                                                             const std::string& pointcloud_frame_id);

  /**
   * @brief Detects largest cluster in pointcloud using euclidean clustering
   *
   * @param src_cloud source cloud for clustering
   * @param cluster_tolerance euclidean clustering parameters
   * @param min_cluster_size euclidean clustering parameters
   * @param max_cluster_size euclidean clustering parameters
   * @return largest cluster
   * @exception std::runtime_error when pointcloud is empty
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr DetectLargestCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                              float cluster_tolerance, int min_cluster_size,
                                                              int max_cluster_size);

  /**
   * @brief Publishes pointcloud cloud using given publisher pub
   */
  void PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, ros::Publisher& pub,
                         const std::string& pointcloud_frame_id);

  /**
   * @brief Sets parameters changed dynamically
   */
  void ReconfigureCallback(roomac_autonomous_manipulation::ObjectDetectionConfig& config, uint32_t level);

  int num_of_readings_for_average_;
  int max_num_of_failed_readings_;
  bool publish_debug_;

  float object_cluster_tolerance_;
  int object_min_cluster_size_;
  int object_max_cluster_size_;

  float table_cluster_tolerance_;
  int table_min_cluster_size_;
  int table_max_cluster_size_;

  float max_range_;
  float table_detection_distance_threshold_;
  float table_voxels_leaf_size_;

  float objects_on_table_min_height_;

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