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

/*
 * Parts of code has been taken from
 *      Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <utility>
#include <algorithm>
#include "archived_vox_nav_grid_map/grid_map.hpp"

namespace archived_vox_nav_grid_map
{
  vox_navGridMap::vox_navGridMap()
  : Node("archived_vox_nav_grid_map_node"),
    map_(grid_map::GridMap({"elevation", "normal_x", "normal_y", "normal_z"}))
  {
    this->declare_parameter("pcd_file_full_path", "/home/ros2-foxy/test_map.pcd");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("topic_name", "grid_map");
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_publish_fps", 15);
    this->declare_parameter("min_points_in_cell", 4);
    this->declare_parameter("cloud_transform.translation.x", 0.0);
    this->declare_parameter("cloud_transform.translation.y", 0.0);
    this->declare_parameter("cloud_transform.translation.z", 0.0);
    this->declare_parameter("cloud_transform.rotation.r", 0.0);
    this->declare_parameter("cloud_transform.rotation.p", 0.0);
    this->declare_parameter("cloud_transform.rotation.y", 0.0);
    this->declare_parameter("downsample_voxel_size", 0.05);
    this->declare_parameter("remove_outlier_mean_K", 10);
    this->declare_parameter("remove_outlier_stddev_threshold", 1.0);

    pcd_file_full_path_ = this->get_parameter("pcd_file_full_path").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    map_publish_fps_ = this->get_parameter("map_publish_fps").as_int();
    min_points_in_cell_ = this->get_parameter("min_points_in_cell").as_int();

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

    pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(pcd_file_full_path_.c_str());
    pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
      pointcloud_, downsample_voxel_size_);
    pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
      pointcloud_,
      remove_outlier_mean_K_,
      remove_outlier_stddev_threshold_,
      vox_nav_utilities::OutlierRemovalType::StatisticalOutlierRemoval);

    pointcloud_ = vox_nav_utilities::transformCloud(
      pointcloud_,
      vox_nav_utilities::getRigidBodyTransform(
        pointloud_transform_matrix_.translation_,
        pointloud_transform_matrix_.rpyIntrinsic_,
        get_logger()));

    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      topic_name_.c_str(), rclcpp::QoS(1).transient_local());

    initializeGridMapGeometryfromPointcloud(pointcloud_, &map_);
    allocateSpaceForCloudsInsideCells(&map_);
    dispatchCloudToGridMapCells(pointcloud_, &map_);
    processGridMapCells(&map_);

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / map_publish_fps_),
      std::bind(&vox_navGridMap::perodicGridMapPublisherCallback, this));
    RCLCPP_INFO(
      this->get_logger(),
      "Constructed an Instance of vox_navGridMap");
  }

  vox_navGridMap::~vox_navGridMap()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Destroyed an Instance of vox_navGridMap");
  }

  void vox_navGridMap::initializeGridMapGeometryfromPointcloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    grid_map::GridMap * grid_map)
  {
    pcl::PointXYZ minBound;
    pcl::PointXYZ maxBound;
    pcl::getMinMax3D(*cloud, minBound, maxBound);

    // from min and max points we can compute the length
    grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);

    // we put the center of the grid map to be in the middle of the point cloud
    grid_map::Position position = grid_map::Position(
      (maxBound.x + minBound.x) / 2.0,
      (maxBound.y + minBound.y) / 2.0);

    grid_map->setFrameId(map_frame_.c_str());
    grid_map->setGeometry(length, resolution_, position);

    RCLCPP_INFO(
      get_logger(),
      "Created map with size %f x %f m (%i x %i cells).\n"
      " The center of the map is located at (%f, %f) in the %s frame.",
      grid_map->getLength().x(), grid_map->getLength().y(),
      grid_map->getSize()(0), grid_map->getSize()(1),
      grid_map->getPosition().x(), grid_map->getPosition().y(), grid_map->getFrameId().c_str());
  }

  void vox_navGridMap::perodicGridMapPublisherCallback()
  {
    // Add noise (using Eigen operators).
    map_.add(
      "noise",
      0.015 * grid_map::Matrix::Random(map_.getSize()(0), map_.getSize()(1)));

    map_.add("elevation_noisy", map_.get("elevation") + map_["noise"]);

    // Adding outliers (accessing cell by position).
    for (unsigned int i = 0; i < 500; ++i) {
      grid_map::Position randomPosition = grid_map::Position::Random();
      if (map_.isInside(randomPosition)) {
        map_.atPosition(
          "elevation_noisy",
          randomPosition) = std::numeric_limits<float>::infinity();
      }
    }

    // Filter values for submap (iterators).
    map_.add("elevation_filtered", map_.get("elevation_noisy"));
    grid_map::Position topLeftCorner(1.0, 0.4);
    grid_map::boundPositionToRange(topLeftCorner, map_.getLength(), map_.getPosition());
    grid_map::Index startIndex;
    map_.getIndex(topLeftCorner, startIndex);
    RCLCPP_INFO_ONCE(
      get_logger(),
      "Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
      topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    grid_map::Size size = (grid_map::Length(1.2, 0.8) / map_.getResolution()).cast<int>();
    grid_map::SubmapIterator it(map_, startIndex, size);
    for (; !it.isPastEnd(); ++it) {
      grid_map::Position currentPosition;
      map_.getPosition(*it, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (grid_map::CircleIterator circleIt(map_, currentPosition, radius);
        !circleIt.isPastEnd();
        ++circleIt)
      {
        if (!map_.isValid(*circleIt, "elevation_noisy")) {continue;}
        grid_map::Position currentPositionInCircle;
        map_.getPosition(*circleIt, currentPositionInCircle);

        // Computed weighted mean based on Euclidian distance.
        double distance = (currentPosition - currentPositionInCircle).norm();
        double weight = pow(radius - distance, 2);
        mean += weight * map_.at("elevation_noisy", *circleIt);
        sumOfWeights += weight;
      }

      map_.at("elevation_filtered", *it) = mean / sumOfWeights;
    }

    // Show absolute difference and compute mean squared error.
    map_.add(
      "error", (map_.get("elevation_filtered") - map_.get(
        "elevation")).cwiseAbs());

    // Publish grid map.
    map_.setTimestamp(this->now().nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_publisher_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "Grid map published.");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr vox_navGridMap::getPointcloudInsideGridMapCellBorder(
    const grid_map::Index & index) const
  {
    return pointcloudWithinGridMapCell_[index.x()][index.y()];
  }

  void vox_navGridMap::allocateSpaceForCloudsInsideCells(grid_map::GridMap * grid_map)
  {
    const unsigned int dimX = grid_map->getSize().x() + 1;
    const unsigned int dimY = grid_map->getSize().y() + 1;

    // resize vectors
    pointcloudWithinGridMapCell_.resize(dimX);

    // allocate pointClouds
    for (unsigned int i = 0; i < dimX; ++i) {
      pointcloudWithinGridMapCell_[i].resize(dimY);
      for (unsigned int j = 0; j < dimY; ++j) {
        pointcloudWithinGridMapCell_[i][j].reset(new pcl::PointCloud<pcl::PointXYZ>());
      }
    }
  }

  void vox_navGridMap::dispatchCloudToGridMapCells(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    grid_map::GridMap * grid_map)
  {
    // For each point in input point cloud, find which grid map
    // cell does it belong to. Then copy that point in the
    // right cell in the matrix of point clouds data structure.
    // This allows for faster access in the clustering stage.
    for (unsigned int i = 0; i < cloud->points.size(); ++i) {
      const pcl::PointXYZ & point = cloud->points[i];
      const double x = point.x;
      const double y = point.y;
      grid_map::Index index;
      grid_map->getIndex(grid_map::Position(x, y), index);
      pointcloudWithinGridMapCell_[index.x()][index.y()]->push_back(point);
    }
  }

  void vox_navGridMap::processGridMapCells(grid_map::GridMap * grid_map)
  {
    grid_map->add("elevation");

    grid_map::Matrix & gridMapData = grid_map->get("elevation");
    unsigned int linearGridMapSize = grid_map->getSize().prod();

    // Iterate through grid map and calculate the corresponding height based on the point cloud
    for (unsigned int linearIndex = 0; linearIndex < linearGridMapSize; ++linearIndex) {
      processGridMapCell(grid_map, linearIndex, &gridMapData);
    }
  }

  void vox_navGridMap::processGridMapCell(
    grid_map::GridMap * grid_map,
    const unsigned int linearGridMapIndex,
    grid_map::Matrix * gridMapData) const
  {
    // Get grid map index from linear index and check if enough points lie within the cell
    const grid_map::Index index(
      grid_map::getIndexFromLinearIndex(linearGridMapIndex, grid_map->getSize()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInsideCellBorder(new pcl::PointCloud<pcl::PointXYZ>());

    pointsInsideCellBorder = getPointcloudInsideGridMapCellBorder(index);
    if (pointsInsideCellBorder->size() < static_cast<size_t>(min_points_in_cell_)) {
      rclcpp::Clock clock;
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), clock,
        10.0, "Less than " << min_points_in_cell_ << " points in a cell");
      return;
    }

    (*gridMapData)(index(0), index(1)) = calculateElevationFromPointsInsideGridMapCell(
      pointsInsideCellBorder);
  }

  double vox_navGridMap::calculateElevationFromPointsInsideGridMapCell(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
  {
    // Extract point cloud cluster from point cloud and return if none is found.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds =
      vox_nav_utilities::extractClusterCloudsFromPointcloud(cloud);
    const bool isNoClustersFound = clusterClouds.empty();
    if (isNoClustersFound) {
      rclcpp::Clock clock;
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), clock, 10.0, "No clusters found in the grid map cell");
      return std::nan("1"); // this will leave the grid map cell uninitialized
    }

    // Extract mean z value of cluster vector and return smallest height value
    std::vector<double> clusterHeights(clusterClouds.size());
    std::transform(
      clusterClouds.begin(), clusterClouds.end(), clusterHeights.begin(),
      [this](pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> double {
        return vox_nav_utilities::calculateMeanOfPointPositions(cloud).z();
      });

    double minClusterHeight = *(std::min_element(clusterHeights.begin(), clusterHeights.end()));

    return minClusterHeight;
  }
}  // namespace archived_vox_nav_grid_map

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);

  auto gps_waypoint_follower_client_node = std::make_shared
    <archived_vox_nav_grid_map::vox_navGridMap>();

  rclcpp::spin(gps_waypoint_follower_client_node->get_node_base_interface());

  return 0;
}
