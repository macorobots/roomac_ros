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

  cluster_tolerance_ = ph.param<float>("cluster_tolerance", 0.02);
  min_cluster_size_ = ph.param<int>("min_cluster_size", 10);
  max_cluster_size_ = ph.param<int>("max_cluster_size", 25000);

  max_range_ = ph.param<float>("max_range", 2.0);
  table_detection_distance_threshold_ = ph.param<float>("table_detection_distance_threshold", 0.01);

  publish_debug_ = ph.param<bool>("publish_debug", true);

  camera_frame_ = ph.param<std::string>("camera_frame", "camera_up_rgb_optical_frame");

  dynamic_reconfigure::Server<roomac_autonomous_manipulation::ObjectDetectionConfig>::CallbackType f;
  f = boost::bind(&ObjectDetection::ReconfigureCallback, this, _1, _2);
  reconfigure_server_.setCallback(f);
}

bool ObjectDetection::HandleObjectDetection(roomac_msgs::DetectObjectAndTable::Request& req,
                                            roomac_msgs::DetectObjectAndTable::Response& res)
{
  // for (int i = 0; i < num_of_readings_for_average_; ++i)
  roomac_msgs::ObjectAndTable object_and_table;

  try
  {
    object_and_table = FindObjectAndTable();
  }
  catch (const std::runtime_error& e)
  {
    res.success = false;
    res.message = e.what();

    return true;
  }

  object_and_table.header.frame_id = camera_frame_;
  object_and_table.header.stamp = ros::Time::now();

  if (publish_debug_)
  {
    object_pt_pub_.publish(object_and_table.object.mass_center);
  }

  res.success = true;
  res.object_and_table = object_and_table;
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

    if (publish_debug_)
    {
      PublishPointcloud(clipped, floor_clipped_pub_);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr without_table(new pcl::PointCloud<pcl::PointXYZRGB>);
    geometry_msgs::Point table_mass_center, min_point_table, max_point_table;
    std::tie(without_table, table_mass_center, min_point_table, max_point_table) = TablePlaneDetection(clipped);

    if (publish_debug_)
    {
      PublishPointcloud(without_table, without_table_pub_);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_on_table(new pcl::PointCloud<pcl::PointXYZRGB>);
    objects_on_table = FilterObjectsOnTable(without_table, min_point_table, max_point_table);

    if (publish_debug_)
    {
      PublishPointcloud(objects_on_table, only_objects_on_table_pub_);
    }

    ROS_INFO("Detecting clusters");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    object_cluster = DetectObjectCluster(objects_on_table);

    if (publish_debug_)
    {
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*filtered);

  extract.setNegative(false);
  extract.filter(*table);

  ROS_INFO_STREAM("Table cloud size: " << table->size());

  geometry_msgs::Point min_point_table, max_point_table;
  std::tie(min_point_table, max_point_table) = CalculateBoundingBox(table);
  geometry_msgs::Point table_mass_center = CalculateMassCenter(table);

  ROS_INFO_STREAM("Min point table: " << min_point_table << "Max point table: " << max_point_table
                                      << "Mass center table: " << table_mass_center);

  return std::make_tuple(filtered, table_mass_center, min_point_table, max_point_table);
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

  pcl::PointXYZRGB min_point_AABB;
  pcl::PointXYZRGB max_point_AABB;
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(src_cloud);
  feature_extractor.compute();

  feature_extractor.getAABB(min_point_AABB, max_point_AABB);

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
ObjectDetection::DetectObjectCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud)
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

    if (center.x > min_x)
    {
      copyPointCloud(*cloud_cluster, *object_cloud_cluster);
      min_x = center.x;
    }
  }

  return object_cloud_cluster;
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
  cluster_tolerance_ = config.cluster_tolerance;
  min_cluster_size_ = config.min_cluster_size;
  max_cluster_size_ = config.max_cluster_size;
  publish_debug_ = config.publish_debug;
  max_range_ = config.max_range;
  table_detection_distance_threshold_ = config.table_detection_distance_threshold;
}