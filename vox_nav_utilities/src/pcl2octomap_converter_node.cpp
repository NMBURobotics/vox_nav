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

#include "vox_nav_utilities/pcl2octomap_converter_node.hpp"

#include <memory>

namespace vox_nav_utilities
{
  PCL2OctomapConverter::PCL2OctomapConverter(/* args */)
  : Node("pcl2octomap_converter_rclcpp_node")
  {
    this->declare_parameter("input_pcd_filename", "/home/ros2-foxy/f.pcd");
    this->declare_parameter("output_binary_octomap_filename", "/home/ros2-foxy/f.bt");
    this->declare_parameter("octomap_voxelsize", 0.01);
    this->declare_parameter("cloud_transform.translation.x", 0.0);
    this->declare_parameter("cloud_transform.translation.y", 0.0);
    this->declare_parameter("cloud_transform.translation.z", 0.0);
    this->declare_parameter("cloud_transform.rotation.r", 0.0);
    this->declare_parameter("cloud_transform.rotation.p", 0.0);
    this->declare_parameter("cloud_transform.rotation.y", 0.0);
    this->declare_parameter("apply_filters", true);
    this->declare_parameter("downsample_voxel_size", 0.05);
    this->declare_parameter("remove_outlier_mean_K", 10);
    this->declare_parameter("remove_outlier_stddev_threshold", 1.0);
    this->declare_parameter("remove_outlier_radius_search", 0.1);
    this->declare_parameter("remove_outlier_min_neighbors_in_radius", 1);

    input_pcd_filename_ = this->get_parameter("input_pcd_filename").as_string();
    output_binary_octomap_filename_ =
      this->get_parameter("output_binary_octomap_filename").as_string();
    octomap_voxelsize_ =
      this->get_parameter("octomap_voxelsize").as_double();
    pointloud_transform_matrix_.translation_.x() =
      this->get_parameter("cloud_transform.translation.x").as_double();
    pointloud_transform_matrix_.translation_.y() =
      this->get_parameter("cloud_transform.translation.y").as_double();
    pointloud_transform_matrix_.translation_.z() =
      this->get_parameter("cloud_transform.translation.z").as_double();
    pointloud_transform_matrix_.rpyIntrinsic_.x() =
      this->get_parameter("cloud_transform.rotation.r").as_double();
    pointloud_transform_matrix_.rpyIntrinsic_.y() =
      this->get_parameter("cloud_transform.rotation.p").as_double();
    pointloud_transform_matrix_.rpyIntrinsic_.z() =
      this->get_parameter("cloud_transform.rotation.y").as_double();

    downsample_voxel_size_ = this->get_parameter("downsample_voxel_size").as_double();
    remove_outlier_mean_K_ = this->get_parameter("remove_outlier_mean_K").as_int();
    remove_outlier_stddev_threshold_ =
      this->get_parameter("remove_outlier_stddev_threshold").as_double();
    remove_outlier_radius_search_ = this->get_parameter("remove_outlier_radius_search").as_double();
    remove_outlier_min_neighbors_in_radius_ =
      this->get_parameter("remove_outlier_min_neighbors_in_radius").as_int();

    pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(input_pcd_filename_.c_str());

    if (this->get_parameter("apply_filters").as_bool()) {

      pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
        pointcloud_,
        downsample_voxel_size_);
      pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
        pointcloud_,
        remove_outlier_mean_K_,
        remove_outlier_stddev_threshold_,
        vox_nav_utilities::OutlierRemovalType::StatisticalOutlierRemoval);
      pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
        pointcloud_,
        remove_outlier_min_neighbors_in_radius_,
        remove_outlier_radius_search_,
        vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);
    }
    pointcloud_ = vox_nav_utilities::transformCloud(
      pointcloud_,
      vox_nav_utilities::getRigidBodyTransform(
        pointloud_transform_matrix_.translation_,
        pointloud_transform_matrix_.rpyIntrinsic_,
        get_logger()));

    octomap_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "octomap_markers", rclcpp::SystemDefaultsQoS());

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1)),
      std::bind(&PCL2OctomapConverter::timerCallback, this));
  }

  PCL2OctomapConverter::~PCL2OctomapConverter()
  {

  }

  void PCL2OctomapConverter::timerCallback()
  {
    octomap_markers_publisher_->publish(octomap_markers_);
  }

  void PCL2OctomapConverter::calcThresholdedNodes(
    const octomap::ColorOcTree tree,
    unsigned int & num_thresholded,
    unsigned int & num_other)
  {
    num_thresholded = 0;
    num_other = 0;

    for (octomap::ColorOcTree::tree_iterator it = tree.begin_tree(), end = tree.end_tree();
      it != end;
      ++it)
    {
      if (tree.isNodeAtThreshold(*it)) {
        num_thresholded++;
      } else {
        num_other++;
      }
    }
  }

  void PCL2OctomapConverter::outputStatistics(const octomap::ColorOcTree tree)
  {
    unsigned int numThresholded, numOther;
    calcThresholdedNodes(tree, numThresholded, numOther);
    size_t memUsage = tree.memoryUsage();
    int64_t memFullGrid = tree.memoryFullGrid();
    size_t numLeafNodes = tree.getNumLeafNodes();

    std::cout << "Tree size: " << tree.size() << " nodes (" << numLeafNodes << " leafs). " <<
      numThresholded << " nodes thresholded, " << numOther << " other\n";
    std::cout << "Memory: " << memUsage << " byte (" << memUsage / (1024. * 1024.) << " MB)" <<
      std::endl;
    std::cout << "Full grid: " << memFullGrid << " byte (" << memFullGrid / (1024. * 1024.) <<
      " MB)" <<
      std::endl;
    double x, y, z;
    tree.getMetricSize(x, y, z);
    std::cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
    std::cout << std::endl;
  }

  void PCL2OctomapConverter::processConversion()
  {
    octomap::Pointcloud octocloud;
    octomap::ColorOcTree tree(octomap_voxelsize_);

    for (auto && i : pointcloud_->points) {
      octomap::point3d endpoint(i.x, i.y, i.z);

      // Yellow color pints are elevated Node centers, keep them out for now
      if (!(i.r && i.g)) {
        octocloud.push_back(endpoint);
      }
    }

    std::cout << "Octocloud size is:" << octocloud.size() << std::endl;
    octomap::point3d sensorOrigin(0, 0, 0);
    tree.insertPointCloud(octocloud, sensorOrigin);

    octomap::unordered_ns::unordered_multimap<octomap::OcTreeKey, double,
      octomap::OcTreeKey::KeyHash> node_values;

    for (auto && i : pointcloud_->points) {
      octomap::point3d crr_point(i.x, i.y, i.z);
      double cost = static_cast<double>(i.b) / static_cast<double>(255.0);
      // Obstacle point set the value to highest cost
      if (i.r) {
        cost = 1.0;
      }
      if (!(i.r && i.g)) {
        auto crr_point_node = tree.coordToKey(crr_point);
        auto pair = std::pair<octomap::OcTreeKey, double>(crr_point_node, cost);
        node_values.insert(pair);

        tree.setNodeValue(crr_point_node, cost, false);
        tree.setNodeColor(crr_point_node, i.r, i.g, i.b);
      }
    }

    auto m_treeDepth = tree.getTreeDepth();
    octomap_markers_.markers.resize(m_treeDepth + 1);

    // now, traverse all leafs in the tree:
    for (auto it = tree.begin(m_treeDepth),
      end = tree.end(); it != end; ++it)
    {
      if (tree.isNodeOccupied(*it)) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();

        unsigned idx = it.getDepth();
        assert(idx < octomap_markers_.markers.size());

        geometry_msgs::msg::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        octomap_markers_.markers[idx].points.push_back(cubeCenter);

        std_msgs::msg::ColorRGBA _color;

        // this ost is too high , it is NON-TRAVERSABLE
        if (it->getValue() == 1.0) {
          _color.r = it->getValue();
        } else {
          _color.g = 1.0 - it->getValue();
          _color.b = it->getValue();
        }
        _color.a = 1.0;
        octomap_markers_.markers[idx].colors.push_back(_color);
      }
    }
    for (unsigned i = 0; i < octomap_markers_.markers.size(); ++i) {
      double size = tree.getNodeSize(i);

      octomap_markers_.markers[i].header.frame_id = "map";
      octomap_markers_.markers[i].header.stamp = this->now();
      octomap_markers_.markers[i].ns = "map";
      octomap_markers_.markers[i].id = i;
      octomap_markers_.markers[i].type =
        visualization_msgs::msg::Marker::CUBE_LIST;
      octomap_markers_.markers[i].scale.x = size;
      octomap_markers_.markers[i].scale.y = size;
      octomap_markers_.markers[i].scale.z = size;

      if (octomap_markers_.markers[i].points.size() > 0) {
        octomap_markers_.markers[i].action =
          visualization_msgs::msg::Marker::ADD;
      } else {
        octomap_markers_.markers[i].action =
          visualization_msgs::msg::Marker::DELETE;
      }
    }

    outputStatistics(tree);
    tree.write(output_binary_octomap_filename_);
  }

}   // namespace vox_nav_utilities

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_utilities::PCL2OctomapConverter>();
  node->processConversion();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
