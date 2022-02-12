#include <roomac_autonomous_manipulation/PickCorrection.h>

#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

PickCorrection::PickCorrection() : tf_listener_(tf_buffer_)
{
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  service_ = nh.advertiseService("pick_correction", &PickCorrection::HandlePickCorrection, this);

  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("clipped", 10, true);
  clusters_pub_ = nh.advertise<sensor_msgs::PointCloud2>("clusters", 10);
  cluster_analyzed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cluster_analyzed", 10);
  gripper_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("gripper_pt", 10);

  num_of_readings_for_average_ = ph.param<int>("num_of_readings_for_average", 5);
  z_offset_pointcloud_split_ = ph.param<float>("z_offset_pointcloud_split", -0.05);

  cluster_tolerance_ = ph.param<float>("cluster_tolerance", 0.02);
  min_cluster_size_ = ph.param<int>("min_cluster_size", 10);
  max_cluster_size_ = ph.param<int>("max_cluster_size", 25000);

  elbow_gripper_offset_ = ph.param<float>("elbow_gripper_offset", 0.2);

  publish_debug_ = ph.param<bool>("publish_debug", true);

  camera_frame_ = ph.param<std::string>("camera_frame", "camera_up_rgb_optical_frame");
  artag_frame_ = ph.param<std::string>("artag_frame", "ar_marker_2");

  offset_planning_gripper_point_ = ph.param<float>("offset_planning_gripper_point", 0.05);
}

bool PickCorrection::HandlePickCorrection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  geometry_msgs::Point pt_average;
  for (int i = 0; i < num_of_readings_for_average_; ++i)
  {
    try
    {
      geometry_msgs::Point pt = FindGripperPosition();
      pt_average.x += pt.x;
      pt_average.y += pt.y;
      pt_average.z += pt.z;
    }
    catch (const std::runtime_error &e)
    {
      res.success = false;
      res.message = e.what();

      return true;
    }
  }

  pt_average.x /= num_of_readings_for_average_;
  pt_average.y /= num_of_readings_for_average_;
  pt_average.z /= num_of_readings_for_average_;

  pt_average.x += offset_planning_gripper_point_;

  if (publish_debug_)
  {
    geometry_msgs::PointStamped pt_average_stamped;
    pt_average_stamped.point = pt_average;

    pt_average_stamped.header.frame_id = camera_frame_;
    pt_average_stamped.header.stamp = ros::Time::now();

    gripper_pt_pub_.publish(pt_average_stamped);
  }

  res.success = true;
  return true;
}

geometry_msgs::Point PickCorrection::FindGripperPosition()
{
  ROS_INFO("Waiting for pointcloud message");

  auto cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera_up/depth_registered/points", ros::Duration(10.));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->empty())
  {
    throw std::runtime_error("Pointcloud is empty (from initial message)");
  }

  ROS_INFO("Pointcloud message received");

  try
  {

    ROS_INFO("Waiting for artag transform");

    geometry_msgs::Transform artag_pos = ArtagTransform();

    ROS_INFO("ARtag transform received");

    ROS_INFO("Clipping pointcloud");

    Eigen::Vector4f plane = CalculatePlane(artag_pos);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZRGB>);

    clipped = PlaneClip(cloud, plane, true);

    ROS_INFO("Publishing pointcloud");
    if (publish_debug_)
      PublishPointcloud(clipped, cloud_pub_);

    ROS_INFO("Detecting clusters");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cluster = DetectClusters(clipped);
    geometry_msgs::Point pt = AnalyzeCluster(cluster);

    return pt;
  }
  catch (const std::runtime_error &e)
  {
    throw e;
  }
}

geometry_msgs::Point PickCorrection::AnalyzeCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster)
{
  float min_x = std::numeric_limits<float>::max();
  int min_x_ind = -1;
  for (int i = 0; i < cluster->points.size(); ++i)
  {
    if (cluster->points[i].x < min_x)
    {
      min_x_ind = i;
      min_x = cluster->points[i].x;
    }
  }

  if (min_x_ind == -1)
    throw std::runtime_error("Couldn't find min point");

  Eigen::Vector4f forearm_plane(0.0f, 1.0f, 0.0f, cluster->points[min_x_ind].y);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr forearm_clipped(new pcl::PointCloud<pcl::PointXYZRGB>);

  forearm_clipped = PlaneClip(cluster, forearm_plane, false);

  Eigen::Vector4f plane_gripper(1.0f, 0.0f, 0.0f, cluster->points[min_x_ind].x + elbow_gripper_offset_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
  clipped_gripper = PlaneClip(forearm_clipped, plane_gripper, false);

  if (publish_debug_)
    PublishPointcloud(clipped_gripper, cluster_analyzed_pub_);

  float max_y = std::numeric_limits<float>::min();
  float min_y = std::numeric_limits<float>::max();

  int min_y_ind = -1;
  int max_y_ind = -1;

  for (int i = 0; i < clipped_gripper->points.size(); ++i)
  {
    if (clipped_gripper->points[i].y < min_y)
    {
      min_y_ind = i;
      min_y = clipped_gripper->points[i].y;
    }

    if (clipped_gripper->points[i].y > max_y)
    {
      max_y_ind = i;
      max_y = clipped_gripper->points[i].y;
    }
  }

  if (min_y_ind == -1 || max_y_ind == -1)
    throw std::runtime_error("Couldn't find min or max point");

  geometry_msgs::Point gripper_pt;
  gripper_pt.x = (clipped_gripper->points[max_y_ind].x + clipped_gripper->points[min_y_ind].x) / 2.;
  gripper_pt.y = (clipped_gripper->points[max_y_ind].y + clipped_gripper->points[min_y_ind].y) / 2.;
  gripper_pt.z = (clipped_gripper->points[max_y_ind].z + clipped_gripper->points[min_y_ind].z) / 2.;

  return gripper_pt;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PickCorrection::DetectClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for clustering");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*src_cloud, *points_filtered, indices);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(points_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(points_filtered);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  int max_size = 0;

  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &idx : it->indices)
    {
      pcl::PointXYZRGB pt = (*points_filtered)[idx];
      pt.r = (j*10)%255;
      pt.g = (j*100)%255;
      pt.b = (j*1000)%255;
      cloud_cluster->push_back(pt);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    j++;

    if (publish_debug_)
      PublishPointcloud(cloud_cluster, clusters_pub_);

    if (cloud_cluster->size() > max_size)
    {
      copyPointCloud(*cloud_cluster, *max_cloud_cluster);
      max_size = cloud_cluster->size();
    }
  }

  return max_cloud_cluster;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PickCorrection::PlaneClip(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, const Eigen::Vector4f &plane, bool negative)
{
  pcl::PlaneClipper3D<pcl::PointXYZRGB> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  return dst_cloud;
}

geometry_msgs::Transform PickCorrection::ArtagTransform()
{
  geometry_msgs::TransformStamped transformStamped;

  try
  {
    transformStamped = tf_buffer_.lookupTransform(camera_frame_, artag_frame_,
                                                  ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    throw std::runtime_error(e.what());
  }

  return transformStamped.transform;
}

Eigen::Vector4f PickCorrection::CalculatePlane(geometry_msgs::Transform trans)
{
  geometry_msgs::Quaternion quat_rot;
  quat_rot.x = 0.;
  quat_rot.y = 0.;
  quat_rot.z = 0.7071;
  quat_rot.w = 0.7071;

  geometry_msgs::Quaternion quat;

  quat.x = trans.rotation.x * quat_rot.x;
  quat.y = trans.rotation.y * quat_rot.y;
  quat.z = trans.rotation.z * quat_rot.z;
  quat.w = trans.rotation.w * quat_rot.w;

  float a, b, c, d;
  a = quat.x;
  b = quat.y;
  c = quat.z;
  trans.translation.z += z_offset_pointcloud_split_;
  d = -a * trans.translation.x - b * trans.translation.y - c * trans.translation.z;

  Eigen::Vector4f plane(a, b, c, d);

  return plane;
}

void PickCorrection::PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, ros::Publisher &pub)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = camera_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  pub.publish(cloud_msg);
}