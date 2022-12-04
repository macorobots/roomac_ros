/*********************************************************************
 *  Software License Agreement
 *
 *  Copyright (C) 2022, Maciej Stępień, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Maciej Stępień
 *********************************************************************/

#ifndef PICK_CORRECTION_H
#define PICK_CORRECTION_H

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>

#include <roomac_autonomous_manipulation/PickCorrectionConfig.h>
#include <roomac_msgs/DetectGripperPosition.h>

class PickCorrection
{
public:
  PickCorrection();

private:
  /**
   * @brief Handles service call, calculates gripper position as an avarage of few detections
   */
  bool HandlePickCorrection(roomac_msgs::DetectGripperPosition::Request& req,
                            roomac_msgs::DetectGripperPosition::Response& res);
  /**
   * @brief Gets most recent pointcloud and detects gripper position
   *
   * @return Gripper position
   * @exception std::runtime_error when pointcloud is empty or there is an error during gripper detection
   */
  geometry_msgs::Point FindGripperPosition();

  /**
   * @brief Gets most recent pointcloud and detects gripper position
   *
   * @param cluster preprocessed pointcloud containing only gripper points
   * @return Gripper position
   * @exception std::runtime_error when pointcloud is empty or there is an error during gripper detection
   */
  geometry_msgs::Point AnalyzeCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster);

  /**
   * @brief Performs euclidean clustering on pointcloud and returns largest
   *
   * @param src_cloud cloud to perform clustering on
   * @return Largest pointcloud (with most points)
   * @exception std::runtime_error when pointcloud is empty
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr DetectClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud);

  /**
   * @brief Cuts pointlcoud with a given plane and returns one part
   *
   * @param src_cloud cloud to cut
   * @param plane used for cutting
   * @param negative which part to return
   * @return one pointcloud after cutting
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneClip(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                                   const Eigen::Vector4f& plane, bool negative);

  /**
   * @brief Reads current transform from camera_frame_ to artag_frame_
   *
   * @return transform from camera_frame_ to artag_frame_
   * @exception std::runtime_error when error during looking up transform
   */
  geometry_msgs::Transform ArtagTransform();

  /**
   * @brief Calculates plane coefficients based on transform. Used to get artag plane, which is used to cut pointcloud
   *
   * @param trans transform
   * @return plane
   */
  Eigen::Vector4f CalculatePlane(geometry_msgs::Transform trans);

  void PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, ros::Publisher& pub);

  void ReconfigureCallback(roomac_autonomous_manipulation::PickCorrectionConfig& config, uint32_t level);

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

  dynamic_reconfigure::Server<roomac_autonomous_manipulation::PickCorrectionConfig> reconfigure_server_;
};

#endif