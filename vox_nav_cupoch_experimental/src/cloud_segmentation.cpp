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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>

#include <queue>
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <boost/geometry.hpp>

#include "vox_nav_cupoch_experimental/visibility_control.h"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#include "ApproxMVBB/AABB.hpp"
#include "cupoch/cupoch.h"
#include "cupoch/cupoch.h"

namespace bg = boost::geometry;

class CloudSegmentation : public rclcpp::Node
{

public:
  typedef std::tuple<sensor_msgs::msg::PointCloud2::SharedPtr,
      nav_msgs::msg::Odometry::SharedPtr,
      sensor_msgs::msg::Imu::SharedPtr> data_captute_t;

  CloudSegmentation();
  ~CloudSegmentation();

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry,
      sensor_msgs::msg::Imu>
    CloudOdomApprxTimeSyncPolicy;
  typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy>
    CloudOdomApprxTimeSyncer;

  void cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoise_segmented_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius,
    double tolerated_divergence_rate, int min_num_neighbours);

  void shoot(std::vector<data_captute_t> & cloud_vector);

  std::vector<geometry_msgs::msg::Point> Vector3List2GeometryMsgs(
    ApproxMVBB::TypeDefsPoints::Vector3List corners);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
  std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

  double dt_;
  double sensor_height_;

  std::vector<data_captute_t> cloud_odom_vector_;

  bool recieved_first_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dyn_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;

  rclcpp::Time last_recieved_msg_stamp_;
  rclcpp::Time stamp_;

  typedef bg::model::point<double, 3, bg::cs::cartesian> point_t;
  typedef bg::model::box<point_t> box_t;
};

CloudSegmentation::CloudSegmentation()
: Node("dynamic_points_node"),
  recieved_first_(false)
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
      CloudOdomApprxTimeSyncPolicy(3000),
      cloud_subscriber_,
      odom_subscriber_,
      imu_subscriber_));

  cloud_odom_data_approx_time_syncher_->registerCallback(
    std::bind(
      &CloudSegmentation::cloudOdomCallback, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "merged", rclcpp::SystemDefaultsQoS());

  dyn_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "dyn_point_pub", rclcpp::SystemDefaultsQoS());

  vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "markers", rclcpp::SystemDefaultsQoS());

  last_recieved_msg_stamp_ = now();

  auto captured_data = std::make_tuple<>(
    std::make_shared<sensor_msgs::msg::PointCloud2>(sensor_msgs::msg::PointCloud2()),
    std::make_shared<nav_msgs::msg::Odometry>(nav_msgs::msg::Odometry()),
    std::make_shared<sensor_msgs::msg::Imu>(sensor_msgs::msg::Imu()));

  cloud_odom_vector_ = std::vector<data_captute_t>(2, captured_data);

}

CloudSegmentation::~CloudSegmentation()
{
}

void CloudSegmentation::cloudOdomCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  last_recieved_msg_stamp_ = cloud->header.stamp;
  // convert to pcl type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, *curr_pcl);

  // remove ground points
  sensor_msgs::PointCloud2ConstIterator<float> iter_label(*cloud, "x");
  std::vector<u_int8_t> labels;
  pcl::IndicesPtr ground_points_indices(new std::vector<int>);
  int couter = 0;
  for (; (iter_label != iter_label.end()); ++iter_label) {
    labels.push_back(iter_label[3]);
    if (iter_label[3] == 40) { // ground point label
      ground_points_indices->push_back(couter);
    }
    couter++;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(curr_pcl);
  extract.setIndices(ground_points_indices);
  extract.setNegative(true);
  extract.filter(*curr_pcl);

  auto denoised_cloud = vox_nav_utilities::removeOutliersFromInputCloud(
    curr_pcl, 4, 0.3,
    vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);

  denoised_cloud = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(denoised_cloud, 0.05);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(denoised_cloud);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance(0.3);  // 2cm
  ec.setMinClusterSize(30);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(denoised_cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_clusters(new pcl::PointCloud<pcl::PointXYZRGB>());
  visualization_msgs::msg::MarkerArray marker_array;
  std::vector<box_t> bg_boxes_vector;
  std::vector<std::vector<geometry_msgs::msg::Point>> boxes;

  int cluster_counter = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
    it != cluster_indices.end(); ++it)
  {
    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes

    auto cluster_color = vox_nav_utilities::getColorByIndexEig(cluster_counter);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end();
      ++pit)
    {
      denoised_cloud->points[*pit].r = cluster_color.x() * 255;
      denoised_cloud->points[*pit].g = cluster_color.y() * 255;
      denoised_cloud->points[*pit].b = cluster_color.z() * 255;

      clusterPtr->points.push_back(denoised_cloud->points[*pit]);
    }

    *merged_clusters += *clusterPtr;
    cluster_counter++;

    ApproxMVBB::Matrix3Dyn points_for_mvbb;
    points_for_mvbb.resize(3, clusterPtr->points.size());

    if (clusterPtr->points.size() < 10) {
      RCLCPP_WARN(this->get_logger(), "THIS OBJECT HAVE TOO FEW POINTS NOT GONNA BUILD A BOX !!");
      continue;
    }
    for (int i = 0; i < clusterPtr->points.size(); i++) {
      points_for_mvbb(0, i) = clusterPtr->points[i].x;
      points_for_mvbb(1, i) = clusterPtr->points[i].y;
      points_for_mvbb(2, i) = clusterPtr->points[i].z;
    }

    // build the mininmal volume box here
    auto oobb =
      ApproxMVBB::approximateMVBB(points_for_mvbb, 0.0, clusterPtr->points.size(), 1, 1, 1);

    // get all 8 corners of builded object(it is a 3d box)
    ApproxMVBB::TypeDefsPoints::Vector3List mvbb_corners = oobb.getCornerPoints();
    // we need this corners in a familiar ros message type(geometry_msgs::Point is good
    std::vector<geometry_msgs::msg::Point> mvbb_corners_geometry_msgs = Vector3List2GeometryMsgs(
      mvbb_corners);

    box_t box(
      point_t(
        oobb.m_minPoint(0),
        oobb.m_minPoint(1),
        oobb.m_minPoint(2)),
      point_t(
        oobb.m_maxPoint(0),
        oobb.m_maxPoint(1),
        oobb.m_maxPoint(2)));

    bg::correct<box_t>(box);
    bg_boxes_vector.push_back(box);
    boxes.push_back(mvbb_corners_geometry_msgs);
  }

  std::vector<int> covered_indicies;
  for (size_t i = 0; i < bg_boxes_vector.size(); i++) {
    for (size_t m = 0; m < bg_boxes_vector.size(); m++) {

      if (i != m) {
        auto is_box1_covered_by_box2 = bg::covered_by<box_t, box_t>(
          bg_boxes_vector[i],
          bg_boxes_vector[m]);

        auto is_box2_covered_by_box1 = bg::covered_by<box_t, box_t>(
          bg_boxes_vector[m],
          bg_boxes_vector[i]);

        if (is_box1_covered_by_box2) {
          covered_indicies.push_back(i);
        }

        if (is_box2_covered_by_box1) {
          covered_indicies.push_back(m);
        }
      }

    }
  }

  sort(covered_indicies.begin(), covered_indicies.end() );
  covered_indicies.erase(
    unique(
      covered_indicies.begin(),
      covered_indicies.end()), covered_indicies.end());

  RCLCPP_INFO(get_logger(), "Removing boxes caused covered %d", covered_indicies.size());

  std::vector<std::vector<geometry_msgs::msg::Point>> bg_merged_boxes_vector;

  for (size_t i = 0; i < boxes.size(); i++) {
    if (std::find(covered_indicies.begin(), covered_indicies.end(), i) != covered_indicies.end()) {
      continue;
    } else {
      bg_merged_boxes_vector.push_back(boxes[i]);
    }
  }

  for (size_t i = 0; i < bg_merged_boxes_vector.size(); i++) {
    auto mvbb_corners_geometry_msgs = bg_merged_boxes_vector[i];
    visualization_msgs::msg::Marker marker;
    marker.header = cloud->header;
    marker.ns = "my_namespace";
    marker.id = i;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points.push_back(mvbb_corners_geometry_msgs[0]);
    marker.points.push_back(mvbb_corners_geometry_msgs[1]);
    marker.points.push_back(mvbb_corners_geometry_msgs[2]);
    marker.points.push_back(mvbb_corners_geometry_msgs[3]);
    marker.points.push_back(mvbb_corners_geometry_msgs[0]);
    marker.points.push_back(mvbb_corners_geometry_msgs[4]);
    marker.points.push_back(mvbb_corners_geometry_msgs[5]);
    marker.points.push_back(mvbb_corners_geometry_msgs[6]);
    marker.points.push_back(mvbb_corners_geometry_msgs[7]);
    marker.points.push_back(mvbb_corners_geometry_msgs[4]);
    marker.points.push_back(mvbb_corners_geometry_msgs[0]);
    marker.points.push_back(mvbb_corners_geometry_msgs[4]);
    marker.points.push_back(mvbb_corners_geometry_msgs[1]);
    marker.points.push_back(mvbb_corners_geometry_msgs[5]);
    marker.points.push_back(mvbb_corners_geometry_msgs[2]);
    marker.points.push_back(mvbb_corners_geometry_msgs[6]);
    marker.points.push_back(mvbb_corners_geometry_msgs[3]);
    marker.points.push_back(mvbb_corners_geometry_msgs[7]);
    marker_array.markers.push_back(marker);
  }

  sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
  pcl::toROSMsg(*merged_clusters, denoised_cloud_msg);
  denoised_cloud_msg.header = cloud->header;
  RCLCPP_INFO(get_logger(), " Writing a cloud with %d points", merged_clusters->points.size());

  pub_->publish(denoised_cloud_msg);
  vis_pub_->publish(marker_array);

}

void CloudSegmentation::shoot(std::vector<data_captute_t> & cloud_vector)
{
}

std::vector<geometry_msgs::msg::Point> CloudSegmentation::Vector3List2GeometryMsgs(
  ApproxMVBB::TypeDefsPoints::Vector3List corners)
{
  std::vector<geometry_msgs::msg::Point> corners_geometry_msgs;
  for (int i = 0; i < corners.size(); i++) {
    geometry_msgs::msg::Point korner_point;
    korner_point.x = corners[i].x();
    korner_point.y = corners[i].y();
    korner_point.z = corners[i].z();
    corners_geometry_msgs.push_back(korner_point);
  }
  return corners_geometry_msgs;
}


int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  cupoch::utility::InitializeAllocator();
  auto node = std::make_shared<CloudSegmentation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
