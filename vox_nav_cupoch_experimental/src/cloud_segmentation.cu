// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_cupoch_experimental/cloud_segmentation.hpp"

CloudSegmentation::CloudSegmentation()
    : Node("dynamic_points_node"), recieved_first_(false)
{
  cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
  odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
  imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

  declare_parameter("dt", 0.0);
  get_parameter("dt", dt_);

  declare_parameter("sensor_height", 0.0);
  get_parameter("sensor_height", sensor_height_);

  cloud_odom_data_approx_time_syncher_.reset(
      new CloudOdomApprxTimeSyncer(
          CloudOdomApprxTimeSyncPolicy(500),
          cloud_subscriber_,
          odom_subscriber_));

  cloud_odom_data_approx_time_syncher_->registerCallback(
      std::bind(
          &CloudSegmentation::cloudOdomCallback, this,
          std::placeholders::_1,
          std::placeholders::_2));

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "merged", rclcpp::SystemDefaultsQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "correspondings", rclcpp::SystemDefaultsQoS());

  last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
  last_dynamic_pointcloud_cupoch_ = std::make_shared<cupoch::geometry::PointCloud>();

  last_recieved_msg_stamp_ = now();
}

CloudSegmentation::~CloudSegmentation() {}

void CloudSegmentation::cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
  if (!recieved_first_)
  {
    recieved_first_ = true;
    last_recieved_msg_stamp_ = cloud->header.stamp;
  }

  cupoch::utility::device_vector<size_t> ground_points_indices, dynamic_points_indices, static_points_indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());

  thrust::host_vector<Eigen::Vector3f> points, colors;
  size_t label_counter = 0;
  auto color = vox_nav_utilities::getColorByIndexEig(5);
  sensor_msgs::PointCloud2ConstIterator<float> iter_label(*cloud, "x");
  for (; (iter_label != iter_label.end()); ++iter_label)
  {
    pcl::PointXYZRGB point;
    point.x = iter_label[0];
    point.y = iter_label[1];
    point.z = iter_label[2];
    point.r = color.x();
    point.g = color.y();
    point.b = color.z();

    points.push_back(Eigen::Vector3f(point.x, point.y, point.z));
    colors.push_back(Eigen::Vector3f(point.r, point.g, point.b));

    int this_point_label = iter_label[3];
    if (this_point_label == 40) // ground point label
    {
      ground_points_indices.push_back(label_counter);
      ground_points_pcl->points.push_back(point);
    }
    else if (this_point_label == 30 ||
             this_point_label == 10) // person/car point label
    {
      dynamic_points_indices.push_back(label_counter);
      dynamic_points_pcl->points.push_back(point);
    }
    else // static obstacle point label
    {
      static_points_indices.push_back(label_counter);
      static_points_pcl->points.push_back(point);
    }

    label_counter++;
  }

  cupoch::geometry::PointCloud obstacle_cloud_cupoch;
  obstacle_cloud_cupoch.SetPoints(points);
  obstacle_cloud_cupoch.SetColors(colors);
  auto static_points_cupoch =
      obstacle_cloud_cupoch.SelectByIndex(static_points_indices, false);
  auto ground_points_cupoch =
      obstacle_cloud_cupoch.SelectByIndex(ground_points_indices, false);

  dynamic_points_pcl =
      vox_nav_utilities::remove_points_within_ground_plane_of_other_cloud<pcl::PointXYZRGB>(
          dynamic_points_pcl, ground_points_pcl, 0.4);

  points.clear();
  colors.clear();
  for (int i = 0; i < dynamic_points_pcl->points.size(); ++i)
  {
    auto p = dynamic_points_pcl->points[i];
    points.push_back(Eigen::Vector3f(p.x, p.y, p.z));
    auto c = vox_nav_utilities::getColorByIndexEig(5);
    colors.push_back(Eigen::Vector3f(c.x(), c.y(), c.z()));
  }

  auto dynamic_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
  dynamic_points_cupoch->SetPoints(points);
  dynamic_points_cupoch->SetColors(colors);

  dynamic_points_cupoch->PaintUniformColor(
      vox_nav_utilities::getColorByIndexEig(10));
  static_points_cupoch->PaintUniformColor(
      vox_nav_utilities::getColorByIndexEig(1));

  Eigen::Matrix<float, 3, 1> min(-20, -20, -2);
  Eigen::Matrix<float, 3, 1> max(20, 20, 2);
  cupoch::geometry::AxisAlignedBoundingBox<3> bbx(min, max);

  dynamic_points_cupoch = dynamic_points_cupoch->Crop(bbx);
  static_points_cupoch = static_points_cupoch->Crop(bbx);

  dynamic_points_cupoch = dynamic_points_cupoch->VoxelDownSample(0.05);
  static_points_cupoch = static_points_cupoch->VoxelDownSample(0.05);

  dynamic_points_cupoch = denoiseCupochCloud(dynamic_points_cupoch, static_points_cupoch, 0.5, 4);

  rclcpp::Time crr_stamp = cloud->header.stamp;
  if ((crr_stamp - last_recieved_msg_stamp_).seconds() > dt_)
  {

    auto odom_T = getTransfromfromConsecutiveOdoms(std::make_shared<nav_msgs::msg::Odometry>(*odom), last_odom_msg_);
    last_dynamic_pointcloud_cupoch_->Transform(odom_T.inverse());
    last_dynamic_pointcloud_cupoch_->PaintUniformColor(vox_nav_utilities::getColorByIndexEig(5));

    auto static_and_dynamic_obstacle_cloud =
        *dynamic_points_cupoch + *static_points_cupoch + *last_dynamic_pointcloud_cupoch_;
    auto static_and_dynamic_obstacle_cloud_ptr = std::make_shared<cupoch::geometry::PointCloud>(static_and_dynamic_obstacle_cloud);

    determineObjectMovements(dynamic_points_cupoch, last_dynamic_pointcloud_cupoch_, cloud->header);

    sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
    cupoch_conversions::cupochToRos(
        static_and_dynamic_obstacle_cloud_ptr,
        denoised_cloud_msg,
        cloud->header.frame_id);
    denoised_cloud_msg.header = cloud->header;
    cloud_pub_->publish(denoised_cloud_msg);
    last_recieved_msg_stamp_ = cloud->header.stamp;
    last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
    last_dynamic_pointcloud_cupoch_ =
        std::make_shared<cupoch::geometry::PointCloud>(*dynamic_points_cupoch);
  }
}

void CloudSegmentation::determineObjectMovements(
    std::shared_ptr<cupoch::geometry::PointCloud> a,
    std::shared_ptr<cupoch::geometry::PointCloud> b,
    std_msgs::msg::Header header)
{

  if (!a->points_.size() || !b->points_.size())
  {
    RCLCPP_WARN(
        get_logger(),
        "One of the cloud is empty, clouds have a: %d b: %d points",
        a->points_.size(), b->points_.size());
    return;
  }

  // REMOVE THE NOISE
  auto denoised_a = a->RemoveStatisticalOutliers(10, 0.1);
  auto denoised_b = b->RemoveStatisticalOutliers(10, 0.1);
  denoised_a = std::get<0>(denoised_a)->RemoveRadiusOutliers(8, 0.2);
  denoised_b = std::get<0>(denoised_b)->RemoveRadiusOutliers(8, 0.2);
  a = std::get<0>(denoised_a);
  b = std::get<0>(denoised_b);

  auto a_points = a->GetPoints();
  auto b_points = b->GetPoints();

  cupoch::utility::device_vector<int> a_clusters = a->ClusterDBSCAN(0.2, 8, false);
  cupoch::utility::device_vector<int> b_clusters = b->ClusterDBSCAN(0.2, 8, false);

  std::map<int, thrust::host_vector<Eigen::Vector3f>> a_cluster_set, b_cluster_set;

  for (int i = 0; i < a_clusters.size(); ++i)
  {
    if (a_clusters[i] < 0)
    {
      continue;
    }
    auto it = a_cluster_set.find(a_clusters[i]);
    if (it != a_cluster_set.end())
    {
      it->second.push_back(a_points[i]);
    }
    else
    {
      a_cluster_set.insert(
          std::pair<int, thrust::host_vector<Eigen::Vector3f>>(
              a_clusters[i],
              thrust::host_vector<Eigen::Vector3f>()));
    }
  }

  for (int i = 0; i < b_clusters.size(); ++i)
  {
    if (b_clusters[i] < 0)
    {
      continue;
    }
    auto it = b_cluster_set.find(b_clusters[i]);
    if (it != b_cluster_set.end())
    {
      it->second.push_back(b_points[i]);
    }
    else
    {
      b_cluster_set.insert(
          std::pair<int, thrust::host_vector<Eigen::Vector3f>>(
              b_clusters[i],
              thrust::host_vector<Eigen::Vector3f>()));
    }
  }

  std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> a_cluster_vector;
  std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> b_cluster_vector;

  for (auto it = a_cluster_set.begin(); it != a_cluster_set.end(); ++it)
  {
    if (!it->second.size())
    {
      continue;
    }
    RCLCPP_INFO(
        get_logger(),
        "A cluster label %d have : %d points",
        it->first, it->second.size());

    auto this_cluster = std::make_shared<cupoch::geometry::PointCloud>();
    this_cluster->SetPoints(it->second);
    a_cluster_vector.push_back(this_cluster);
  }

  for (auto it = b_cluster_set.begin(); it != b_cluster_set.end(); ++it)
  {
    if (!it->second.size())
    {
      continue;
    }
    RCLCPP_INFO(
        get_logger(),
        "B cluster label %d have : %d points",
        it->first, it->second.size());
    auto this_cluster = std::make_shared<cupoch::geometry::PointCloud>();
    this_cluster->SetPoints(it->second);
    b_cluster_vector.push_back(this_cluster);
  }

  RCLCPP_INFO(
      get_logger(),
      "Clouds have a: %d b: %d points",
      a->points_.size(), b->points_.size());

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.8;

  for (int i = 0; i < a_cluster_vector.size(); ++i)
  {
    auto oriented_bbx = a_cluster_vector[i]->GetOrientedBoundingBox();
    geometry_msgs::msg::Point center_point;
    center_point.x = oriented_bbx.GetCenter().x();
    center_point.y = oriented_bbx.GetCenter().y();
    center_point.z = oriented_bbx.GetCenter().z();
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.a = 1.0;
    marker.colors.push_back(color);
    marker.points.push_back(center_point);
  }
  for (int i = 0; i < b_cluster_vector.size(); ++i)
  {
    auto oriented_bbx = b_cluster_vector[i]->GetOrientedBoundingBox();
    geometry_msgs::msg::Point center_point;
    center_point.x = oriented_bbx.GetCenter().x();
    center_point.y = oriented_bbx.GetCenter().y();
    center_point.z = oriented_bbx.GetCenter().z();
    std_msgs::msg::ColorRGBA color;
    color.b = 1.0;
    color.a = 1.0;
    marker.colors.push_back(color);
    marker.points.push_back(center_point);
  }
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
}

std::shared_ptr<cupoch::geometry::PointCloud> CloudSegmentation::denoiseCupochCloud(
    std::shared_ptr<cupoch::geometry::PointCloud> a,
    const std::shared_ptr<cupoch::geometry::PointCloud> b,
    double radius,
    int max_nn)
{
  auto denoised_cupoch_cloud = std::make_shared<cupoch::geometry::PointCloud>();
  cupoch::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(*b);
  auto a_points = a->GetPoints();
  auto a_colors = a->GetColors();
  thrust::host_vector<Eigen::Vector3f> points, colors;

  int counter = 0;
  for (auto &&i : a_points)
  {
    thrust::host_vector<int> indices;
    thrust::host_vector<float> distance2;
    int k = kdtree.SearchRadius(i, radius, 100,
                                indices, distance2);
    if (k < max_nn)
    {
      points.push_back(i);
      colors.push_back(a_colors[counter]);
    }
    counter++;
  }

  denoised_cupoch_cloud->SetPoints(points);
  denoised_cupoch_cloud->SetColors(colors);
  return denoised_cupoch_cloud;
}

Eigen::Matrix4f CloudSegmentation::getTransfromfromConsecutiveOdoms(
    const nav_msgs::msg::Odometry::SharedPtr a,
    const nav_msgs::msg::Odometry::SharedPtr b)
{
  auto traveled_distance =
      Eigen::Vector3f(
          a->pose.pose.position.x - b->pose.pose.position.x,
          a->pose.pose.position.y - b->pose.pose.position.y,
          a->pose.pose.position.z - b->pose.pose.position.z)
          .norm();

  double yaw_latest, pitch_latest, roll_latest;
  double yaw, pitch, roll;

  vox_nav_utilities::getRPYfromMsgQuaternion(
      a->pose.pose.orientation, roll_latest, pitch_latest, yaw_latest);
  vox_nav_utilities::getRPYfromMsgQuaternion(
      b->pose.pose.orientation, roll, pitch, yaw);

  auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(
      Eigen::Vector3f(
          roll_latest - roll, pitch_latest - pitch, yaw_latest - yaw));

  auto trans =
      Eigen::Vector3f(
          traveled_distance * cos(yaw_latest - yaw),
          traveled_distance * sin(yaw_latest - yaw), sensor_height_);

  Eigen::Matrix4f odom_T = Eigen::Matrix4f::Identity();

  odom_T.block<3, 3>(0, 0) = rot;
  odom_T.block<3, 1>(0, 3) = trans;

  return odom_T;
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  cupoch::utility::InitializeAllocator();
  auto node = std::make_shared<CloudSegmentation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
