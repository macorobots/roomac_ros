#include <roomac_autonomous_manipulation/object_detection/ObjectDetection.h>

#include <limits>

#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

ObjectDetection::ObjectDetection()
{
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  service_ = nh.advertiseService("detect_table_and_object", &ObjectDetection::HandleObjectDetection, this);

  floor_clipped_pub_ = nh.advertise<sensor_msgs::PointCloud2>("floor_clipped", 10, true);
  without_table_pub_ = nh.advertise<sensor_msgs::PointCloud2>("without_table", 10, true);
  only_objects_on_table_pub_ = nh.advertise<sensor_msgs::PointCloud2>("only_objects_on_table", 10, true);
  clusters_pub_ = nh.advertise<sensor_msgs::PointCloud2>("clusters", 10, true);
  object_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 10, true);
  object_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("object_pt", 10, true);

  num_of_readings_for_average_ = ph.param<int>("num_of_readings_for_average", 5);
  max_num_of_failed_readings_ = ph.param<int>("max_num_of_failed_readings", 5);

  object_cluster_tolerance_ = ph.param<float>("object_cluster_tolerance", 0.02);
  object_min_cluster_size_ = ph.param<int>("object_min_cluster_size", 10);
  object_max_cluster_size_ = ph.param<int>("object_max_cluster_size", 25000);

  table_cluster_tolerance_ = ph.param<float>("table_cluster_tolerance", 0.02);
  table_min_cluster_size_ = ph.param<int>("table_min_cluster_size", 10);
  table_max_cluster_size_ = ph.param<int>("table_max_cluster_size", 100000);

  max_range_ = ph.param<float>("max_range", 2.0);
  table_detection_distance_threshold_ = ph.param<float>("table_detection_distance_threshold", 0.01);
  table_voxels_leaf_size_ = ph.param<float>("table_voxels_leaf_size", 0.01);

  publish_debug_ = ph.param<bool>("publish_debug", true);

  camera_frame_ = ph.param<std::string>("camera_frame", "camera_up_rgb_optical_frame");

  dynamic_reconfigure::Server<roomac_autonomous_manipulation::ObjectDetectionConfig>::CallbackType f;
  f = boost::bind(&ObjectDetection::ReconfigureCallback, this, _1, _2);
  reconfigure_server_.setCallback(f);
}

bool ObjectDetection::HandleObjectDetection(roomac_msgs::DetectObjectAndTable::Request& req,
                                            roomac_msgs::DetectObjectAndTable::Response& res)
{
  std::vector<roomac_msgs::ObjectAndTable> object_and_table_readings;
  int failed_readings_count = 0;
  for (int i = 0; i < num_of_readings_for_average_; ++i)
  {
    try
    {
      roomac_msgs::ObjectAndTable object_and_table = FindObjectAndTable();
      object_and_table_readings.push_back(object_and_table);
    }
    catch (const std::runtime_error& e)
    {
      ++failed_readings_count;
      if (failed_readings_count > max_num_of_failed_readings_)
      {
        res.success = false;
        res.message = e.what();
        return true;
      }
    }
  }

  roomac_msgs::ObjectAndTable object_and_table_average = CalculateAverage(object_and_table_readings);

  object_and_table_average.header.frame_id = camera_frame_;
  object_and_table_average.header.stamp = ros::Time::now();

  if (publish_debug_)
  {
    geometry_msgs::PointStamped object_pt_stamped;
    object_pt_stamped.header = object_and_table_average.header;
    object_pt_stamped.point = object_and_table_average.object.mass_center;
    object_pt_pub_.publish(object_pt_stamped);
  }

  res.success = true;
  res.object_and_table = object_and_table_average;
  return true;
}

roomac_msgs::ObjectAndTable ObjectDetection::FindObjectAndTable()
{
  ROS_INFO("Waiting for pointcloud message");

  auto cloud_msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera_up/depth_registered/points", ros::Duration(10.));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->empty())
  {
    throw std::runtime_error("Pointcloud is empty (from initial message)");
  }

  ROS_INFO("Pointcloud message received");

  try
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZRGB>);
    clipped = RangeFilter(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr without_table(new pcl::PointCloud<pcl::PointXYZRGB>);
    geometry_msgs::Point table_mass_center, min_point_table, max_point_table;
    std::tie(without_table, table_mass_center, min_point_table, max_point_table) = TablePlaneDetection(clipped);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_on_table(new pcl::PointCloud<pcl::PointXYZRGB>);
    objects_on_table = FilterObjectsOnTable(without_table, min_point_table, max_point_table);

    ROS_INFO("Detecting clusters");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    object_cluster = DetectObjectCluster(objects_on_table, object_cluster_tolerance_, object_min_cluster_size_,
                                         object_max_cluster_size_);

    if (publish_debug_)
    {
      PublishPointcloud(clipped, floor_clipped_pub_);
      PublishPointcloud(without_table, without_table_pub_);
      PublishPointcloud(objects_on_table, only_objects_on_table_pub_);
      PublishPointcloud(object_cluster, object_cloud_pub_);
    }

    geometry_msgs::Point min_point_object, max_point_object;
    std::tie(min_point_object, max_point_object) = CalculateBoundingBox(object_cluster);
    geometry_msgs::Point object_mass_center = CalculateMassCenter(object_cluster);

    roomac_msgs::ObjectAndTable msg;

    msg.table.mass_center = table_mass_center;
    msg.table.min_point_bounding_box = min_point_table;
    msg.table.max_point_bounding_box = max_point_table;

    msg.object.mass_center = object_mass_center;
    msg.object.min_point_bounding_box = min_point_object;
    msg.object.max_point_bounding_box = max_point_object;

    ROS_INFO("Finished detecting objects");

    return msg;
  }
  catch (const std::runtime_error& e)
  {
    throw e;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObjectDetection::RangeFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for height filtering");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(src_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, max_range_);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*points_filtered);

  return points_filtered;
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point>
ObjectDetection::TablePlaneDetection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for clustering");
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(table_detection_distance_threshold_);

  seg.setInputCloud(src_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    throw std::runtime_error("Could not estimate a planar model for the given dataset");
  }

  ROS_INFO_STREAM("Table model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
                                               << coefficients->values[2] << " " << coefficients->values[3]
                                               << std::endl);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_plane(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*filtered);

  extract.setNegative(false);
  extract.filter(*table_plane);

  ROS_INFO_STREAM("Table plane cloud size: " << table_plane->size());

  // voxel filter to reduce number of points - otherwise, on default resolution clustering is really slow
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_plane_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  table_plane_filtered = VoxelFilter(table_plane, table_voxels_leaf_size_);

  // indices from plane filtering also include other points on this plane, especially one from robot,
  // which makes it inaccurate to detect bounding box, so first clusterization has to be done to find
  // largest cluster - table alone
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr only_table(new pcl::PointCloud<pcl::PointXYZRGB>);
  only_table = DetectLargestCluster(table_plane_filtered, table_cluster_tolerance_, table_min_cluster_size_,
                                    table_max_cluster_size_);

  ROS_INFO_STREAM("Only table cloud size: " << only_table->size());

  geometry_msgs::Point min_point_table, max_point_table;
  std::tie(min_point_table, max_point_table) = CalculateBoundingBox(only_table);
  geometry_msgs::Point table_mass_center = CalculateMassCenter(only_table);

  ROS_INFO_STREAM("Min point table: " << min_point_table << "Max point table: " << max_point_table
                                      << "Mass center table: " << table_mass_center);

  return std::make_tuple(filtered, table_mass_center, min_point_table, max_point_table);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObjectDetection::VoxelFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(src_cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*filtered_cloud);

  return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObjectDetection::FilterObjectsOnTable(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
                                      geometry_msgs::Point min_point_table, geometry_msgs::Point max_point_table)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for filtering objects on table");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Assumes that table is parallel to kinect, otherwise it is necessary to use plane clipper

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(src_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, min_point_table.z);
  pass.filter(*points_filtered);

  pass.setInputCloud(points_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(min_point_table.x, max_point_table.x);
  pass.filter(*points_filtered);

  pass.setInputCloud(points_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(min_point_table.y, max_point_table.y);
  pass.filter(*points_filtered);

  return points_filtered;
}

std::tuple<geometry_msgs::Point, geometry_msgs::Point>
ObjectDetection::CalculateBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for calculating bounding box");
  }

  pcl::PointXYZRGB min_point_AABB, max_point_AABB;
  pcl::getMinMax3D(*src_cloud, min_point_AABB, max_point_AABB);

  geometry_msgs::Point min_point_pt, max_point_pt;
  min_point_pt.x = min_point_AABB.x;
  min_point_pt.y = min_point_AABB.y;
  min_point_pt.z = min_point_AABB.z;

  max_point_pt.x = max_point_AABB.x;
  max_point_pt.y = max_point_AABB.y;
  max_point_pt.z = max_point_AABB.z;

  return std::make_tuple(min_point_pt, max_point_pt);
}

geometry_msgs::Point ObjectDetection::CalculateMassCenter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for calculating mass center");
  }

  Eigen::Vector3f mass_center;
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(src_cloud);
  feature_extractor.compute();

  feature_extractor.getMassCenter(mass_center);

  geometry_msgs::Point mass_center_pt;
  mass_center_pt.x = mass_center.x();
  mass_center_pt.y = mass_center.y();
  mass_center_pt.z = mass_center.z();

  return mass_center_pt;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObjectDetection::DetectLargestCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, float cluster_tolerance,
                                      int min_cluster_size, int max_cluster_size)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for clustering");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*src_cloud, *points_filtered, indices);

  std::vector<pcl::PointIndices> cluster_indices =
      DetectClusterIndeces(points_filtered, cluster_tolerance, min_cluster_size, max_cluster_size);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  int max_size = 0;

  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : it->indices)
    {
      pcl::PointXYZRGB pt = (*points_filtered)[idx];
      cloud_cluster->push_back(pt);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    j++;

    if (cloud_cluster->size() > max_size)
    {
      copyPointCloud(*cloud_cluster, *max_cloud_cluster);
      max_size = cloud_cluster->size();
    }
  }

  return max_cloud_cluster;
}

std::vector<pcl::PointIndices>
ObjectDetection::DetectClusterIndeces(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, float cluster_tolerance,
                                      int min_cluster_size, int max_cluster_size)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for clustering");
  }

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(src_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(src_cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObjectDetection::DetectObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, float cluster_tolerance,
                                     int min_cluster_size, int max_cluster_size)
{
  if (src_cloud->empty())
  {
    throw std::runtime_error("Empty pointcloud sent for clustering");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*src_cloud, *points_filtered, indices);

  std::vector<pcl::PointIndices> cluster_indices =
      DetectClusterIndeces(points_filtered, cluster_tolerance, min_cluster_size, max_cluster_size);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  float min_x = std::numeric_limits<float>::max();

  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : it->indices)
    {
      pcl::PointXYZRGB pt = (*points_filtered)[idx];
      pt.r = (j * 10) % 255;
      pt.g = (j * 100) % 255;
      pt.b = (j * 1000) % 255;
      cloud_cluster->push_back(pt);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    j++;

    if (publish_debug_)
    {
      PublishPointcloud(cloud_cluster, clusters_pub_);
    }

    geometry_msgs::Point center = CalculateMassCenter(cloud_cluster);

    if (center.x < min_x)
    {
      copyPointCloud(*cloud_cluster, *object_cloud_cluster);
      min_x = center.x;
    }
  }

  return object_cloud_cluster;
}

geometry_msgs::Point ObjectDetection::CalculateAverage(const std::vector<geometry_msgs::Point>& point_readings)
{
  geometry_msgs::Point average;
  for (const auto& x : point_readings)
  {
    average.x += x.x;
    average.y += x.y;
    average.z += x.z;
  }

  average.x /= point_readings.size();
  average.y /= point_readings.size();
  average.z /= point_readings.size();

  return average;
}

roomac_msgs::ObjectWithBoundingBox
ObjectDetection::CalculateAverage(const std::vector<roomac_msgs::ObjectWithBoundingBox>& object_readings)
{
  roomac_msgs::ObjectWithBoundingBox average;

  std::vector<geometry_msgs::Point> mass_center_readings, max_point_readings, min_point_readings;

  for (const auto& x : object_readings)
  {
    mass_center_readings.push_back(x.mass_center);
    max_point_readings.push_back(x.max_point_bounding_box);
    min_point_readings.push_back(x.min_point_bounding_box);
  }

  average.mass_center = CalculateAverage(mass_center_readings);
  average.max_point_bounding_box = CalculateAverage(max_point_readings);
  average.min_point_bounding_box = CalculateAverage(min_point_readings);

  return average;
}

roomac_msgs::ObjectAndTable
ObjectDetection::CalculateAverage(const std::vector<roomac_msgs::ObjectAndTable>& object_and_table_readings)
{
  roomac_msgs::ObjectAndTable average;

  std::vector<roomac_msgs::ObjectWithBoundingBox> table_readings;
  std::vector<roomac_msgs::ObjectWithBoundingBox> object_readings;

  for (const auto& x : object_and_table_readings)
  {
    table_readings.push_back(x.table);
    object_readings.push_back(x.object);
  }

  average.table = CalculateAverage(table_readings);
  average.object = CalculateAverage(object_readings);

  return average;
}

void ObjectDetection::PublishPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, ros::Publisher& pub)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = camera_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  pub.publish(cloud_msg);
}

void ObjectDetection::ReconfigureCallback(roomac_autonomous_manipulation::ObjectDetectionConfig& config, uint32_t level)
{
  num_of_readings_for_average_ = config.num_of_readings_for_average;
  object_cluster_tolerance_ = config.object_cluster_tolerance;
  object_min_cluster_size_ = config.object_min_cluster_size;
  object_max_cluster_size_ = config.object_max_cluster_size;
  table_cluster_tolerance_ = config.table_cluster_tolerance;
  table_min_cluster_size_ = config.table_min_cluster_size;
  table_max_cluster_size_ = config.table_max_cluster_size;
  publish_debug_ = config.publish_debug;
  max_range_ = config.max_range;
  table_detection_distance_threshold_ = config.table_detection_distance_threshold;
  table_voxels_leaf_size_ = config.table_voxels_leaf_size;
}