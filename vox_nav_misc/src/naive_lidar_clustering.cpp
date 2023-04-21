// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vox_nav_misc/naive_lidar_clustering.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace vox_nav_misc;

NaiveLIDARClustering::NaiveLIDARClustering()
    : Node("cloud_clustering_rclcpp_node")
{
  cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::SensorDataQoS(),
      std::bind(
          &NaiveLIDARClustering::cloudCallback,
          this, std::placeholders::_1));

  detection_objects_pub_ = this->create_publisher<vox_nav_msgs::msg::ObjectArray>(
      "detections", rclcpp::SystemDefaultsQoS());

  declare_parameter("clustering.x_bound",
                    clustering_params_.x_bound);
  declare_parameter("clustering.y_bound",
                    clustering_params_.y_bound);
  declare_parameter("clustering.z_bound",
                    clustering_params_.z_bound);
  declare_parameter("clustering.downsample_voxel_size",
                    clustering_params_.downsample_voxel_size);
  declare_parameter("clustering.remove_ground_plane_thres",
                    clustering_params_.remove_ground_plane_thres);
  declare_parameter("clustering.clustering_min_points",
                    clustering_params_.clustering_min_points);
  declare_parameter("clustering.clustering_max_points",
                    clustering_params_.clustering_max_points);
  declare_parameter("clustering.clustering_max_step_size",
                    clustering_params_.clustering_max_step_size);
  declare_parameter("clustering.sacle_up_objects",
                    clustering_params_.sacle_up_objects);
  declare_parameter("clustering.N",
                    clustering_params_.N);
  declare_parameter("clustering.dt",
                    clustering_params_.dt);
  declare_parameter("clustering.min_cluster_height",
                    clustering_params_.min_cluster_height);

  get_parameter("clustering.x_bound",
                clustering_params_.x_bound);
  get_parameter("clustering.y_bound",
                clustering_params_.y_bound);
  get_parameter("clustering.z_bound",
                clustering_params_.z_bound);
  get_parameter("clustering.downsample_voxel_size",
                clustering_params_.downsample_voxel_size);
  get_parameter("clustering.remove_ground_plane_thres",
                clustering_params_.remove_ground_plane_thres);
  get_parameter("clustering.clustering_min_points",
                clustering_params_.clustering_min_points);
  get_parameter("clustering.clustering_max_points",
                clustering_params_.clustering_max_points);
  get_parameter("clustering.clustering_max_step_size",
                clustering_params_.clustering_max_step_size);
  get_parameter("clustering.sacle_up_objects",
                clustering_params_.sacle_up_objects);
  get_parameter("clustering.N",
                clustering_params_.N);
  get_parameter("clustering.dt",
                clustering_params_.dt);
  get_parameter("clustering.min_cluster_height",
                clustering_params_.min_cluster_height);

  // Print parameters
  RCLCPP_INFO_STREAM(
      get_logger(), "x_bound " << clustering_params_.x_bound);
  RCLCPP_INFO_STREAM(
      get_logger(), "y_bound " << clustering_params_.y_bound);
  RCLCPP_INFO_STREAM(
      get_logger(), "z_bound " << clustering_params_.z_bound);
  RCLCPP_INFO_STREAM(
      get_logger(), "downsample_voxel_size " << clustering_params_.downsample_voxel_size);
  RCLCPP_INFO_STREAM(
      get_logger(), "remove_ground_plane_thres " << clustering_params_.remove_ground_plane_thres);
  RCLCPP_INFO_STREAM(
      get_logger(), "clustering_min_points " << clustering_params_.clustering_min_points);
  RCLCPP_INFO_STREAM(
      get_logger(), "clustering_max_points " << clustering_params_.clustering_max_points);
  RCLCPP_INFO_STREAM(
      get_logger(), "clustering_max_step_size " << clustering_params_.clustering_max_step_size);
  RCLCPP_INFO_STREAM(
      get_logger(), "sacle_up_objects " << clustering_params_.sacle_up_objects);
  RCLCPP_INFO_STREAM(
      get_logger(), "N " << clustering_params_.N);
  RCLCPP_INFO_STREAM(
      get_logger(), "dt " << clustering_params_.dt);
  RCLCPP_INFO_STREAM(
      get_logger(), "min_cluster_height " << clustering_params_.min_cluster_height);

  buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  RCLCPP_INFO(get_logger(), "Creating...");
}

NaiveLIDARClustering::~NaiveLIDARClustering()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void NaiveLIDARClustering::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud, *pcl_curr);

  // Crop the cloud
  pcl_curr = vox_nav_utilities::cropBox<pcl::PointXYZI>(
      pcl_curr,
      Eigen::Vector4f(
          -clustering_params_.x_bound,
          -clustering_params_.y_bound,
          -clustering_params_.z_bound, 1),
      Eigen::Vector4f(
          clustering_params_.x_bound,
          clustering_params_.y_bound,
          clustering_params_.z_bound, 1));

  // Grab the latest cloud -> map transform and transform the cloud
  geometry_msgs::msg::TransformStamped t;
  try
  {
    t = buffer_->lookupTransform(
        "map", cloud->header.frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(
        this->get_logger(), "Could not transform %s to map: %s",
        cloud->header.frame_id.c_str(), ex.what());
    return;
  }

  pcl_ros::transformPointCloud(*pcl_curr, *pcl_curr, t);

  // Downsample the cloud
  pcl_curr = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZI>(
      pcl_curr,
      clustering_params_.downsample_voxel_size);

  // Remove ground plane
  pcl_curr = vox_nav_utilities::segmentSurfacePlane<pcl::PointXYZI>(
      pcl_curr,
      clustering_params_.remove_ground_plane_thres,
      true);

  // Cluster the cloud with euclidean clustering
  auto clusters = vox_nav_utilities::euclideanClustering<pcl::PointXYZI>(
      pcl_curr,
      clustering_params_.clustering_min_points,
      clustering_params_.clustering_max_points,
      clustering_params_.clustering_max_step_size);
  std_msgs::msg::Header header = cloud->header;
  header.frame_id = "map";

  // Populate the object array
  vox_nav_msgs::msg::ObjectArray object_array;
  for (auto &&cluster : clusters)
  {
    vox_nav_msgs::msg::Object object;
    object.header.frame_id = "map";
    object.header.stamp = cloud->header.stamp;
    object.id = object_array.objects.size();
    object.detection_level = vox_nav_msgs::msg::Object::OBJECT_DETECTED;
    object.classification_label = vox_nav_msgs::msg::Object::CLASSIFICATION_UNKNOWN;
    object.classification_probability = 0.0;
    object.object_classified = false;

    // Get the bounding box (Oriented) for each cluster
    // This will change the pose and shape of the object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cluster, *cluster_pcl);
    vox_nav_utilities::fitBoxtoPointCloud(cluster_pcl, object);

    if (object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] < clustering_params_.min_cluster_height)
    {
      // Doe not add too short clusters as they are most probably noise
      continue;
    }

    // Get ROS msg from PCL point cloud
    pcl::toROSMsg(*cluster_pcl, object.cluster);
    object.cluster.header.frame_id = "map";
    object.cluster.header.stamp = cloud->header.stamp;

    // We dont know whether the object is dynamic or not
    object.is_dynamic = false;
    object_array.header = object.header;
    object_array.objects.push_back(object);
  }

  detection_objects_pub_->publish(object_array);
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaiveLIDARClustering>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
