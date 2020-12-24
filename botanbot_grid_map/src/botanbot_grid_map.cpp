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
#include "botanbot_grid_map/botanbot_grid_map.hpp"

namespace botanbot_grid_map
{
BotanbotGridMap::BotanbotGridMap()
: Node("botanbot_grid_map_rclcpp_node")
{
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&BotanbotGridMap::startWaypointFollowing, this));
}

BotanbotGridMap::~BotanbotGridMap()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of BotanbotGridMap");
}

void BotanbotGridMap::startWaypointFollowing()
{
  using namespace std::placeholders;
  this->timer_->cancel();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BotanbotGridMap::getPointcloudInsideGridMapCellBorder(
  const grid_map::Index & index) const
{
  return pointcloudWithinGridMapCell_[index.x()][index.y()];
}

void BotanbotGridMap::allocateSpaceForCloudsInsideCells(grid_map::GridMap * grid_map)
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

void BotanbotGridMap::dispatchWorkingCloudToGridMapCells(
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

void BotanbotGridMap::processGridMapCell(
  grid_map::GridMap * grid_map,
  const unsigned int linearGridMapIndex,
  grid_map::Matrix * gridMapData) const
{
  // Get grid map index from linear index and check if enough points lie within the cell
  const grid_map::Index index(
    grid_map::getIndexFromLinearIndex(linearGridMapIndex, grid_map->getSize()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInsideCellBorder(new pcl::PointCloud<pcl::PointXYZ>());

  pointsInsideCellBorder = getPointcloudInsideGridMapCellBorder(index);
  const bool isTooFewPointsInCell = pointsInsideCellBorder->size() < 1;
  if (isTooFewPointsInCell) {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), clock,
      10.0, "Less than " << 1 << " points in a cell");
    return;
  }

  (*gridMapData)(index(0), index(1)) = calculateElevationFromPointsInsideGridMapCell(
    pointsInsideCellBorder);
}

double BotanbotGridMap::calculateElevationFromPointsInsideGridMapCell(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
  // Extract point cloud cluster from point cloud and return if none is found.
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds =
    botanbot_utilities::extractClusterCloudsFromPointcloud(cloud);
  const bool isNoClustersFound = clusterClouds.empty();
  if (isNoClustersFound) {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), clock, 10.0, "No clusters found in the grid map cell");
    return std::nan("1");  // this will leave the grid map cell uninitialized
  }

  // Extract mean z value of cluster vector and return smallest height value
  std::vector<double> clusterHeights(clusterClouds.size());
  std::transform(
    clusterClouds.begin(), clusterClouds.end(), clusterHeights.begin(),
    [this](pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> double {
      return botanbot_utilities::calculateMeanOfPointPositions(cloud).z();
    });

  double minClusterHeight = *(std::min_element(clusterHeights.begin(), clusterHeights.end()));

  return minClusterHeight;
}
}  // namespace botanbot_grid_map

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  /*auto gps_waypoint_follower_client_node = std::make_shared
  <botanbot_grid_map::BotanbotGridMap>();
while (!rclcpp::ok()) {
  rclcpp::spin_some(gps_waypoint_follower_client_node);
}
rclcpp::shutdown();*/

  rclcpp::init(argc, argv);

  rclcpp::Node node("grid_map_tutorial_demo");
  auto publisher = node.create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr = botanbot_utilities::loadPointcloudFromPcd(
    "/home/ros2-foxy/test_map.pcd");

  const double resolution = 0.1;

  pcl::PointXYZ minBound;
  pcl::PointXYZ maxBound;
  pcl::getMinMax3D(*pointcloud_ptr, minBound, maxBound);

  // from min and max points we can compute the length
  grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);

  // we put the center of the grid map to be in the middle of the point cloud
  grid_map::Position position = grid_map::Position(
    (maxBound.x + minBound.x) / 2.0,
    (maxBound.y + minBound.y) / 2.0);

  // Create grid map.
  grid_map::GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(length, resolution, position);

  RCLCPP_INFO(
    node.get_logger(),
    "Created map with size %f x %f m (%i x %i cells).\n"
    " The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());


  // Work with grid map in a loop.
  rclcpp::Rate rate(30.0);
  rclcpp::Clock clock;

  while (rclcpp::ok()) {
    rclcpp::Time time = node.now();

    // Add elevation and surface normal (iterating through grid map and adding data).
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at(
        "elevation",
        *it) = -0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();
      Eigen::Vector3d normal(-0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()),
        -position.x() * std::cos(3.0 * time.seconds() + 5.0 * position.y()), 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();
    }

    // Add noise (using Eigen operators).
    map.add("noise", 0.015 * grid_map::Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("elevation_noisy", map.get("elevation") + map["noise"]);

    // Adding outliers (accessing cell by position).
    for (unsigned int i = 0; i < 500; ++i) {
      grid_map::Position randomPosition = grid_map::Position::Random();
      if (map.isInside(randomPosition)) {
        map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
      }
    }

    // Filter values for submap (iterators).
    map.add("elevation_filtered", map.get("elevation_noisy"));
    grid_map::Position topLeftCorner(1.0, 0.4);
    grid_map::boundPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
    grid_map::Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    RCLCPP_INFO_ONCE(
      node.get_logger(),
      "Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
      topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    grid_map::Size size = (grid_map::Length(1.2, 0.8) / map.getResolution()).cast<int>();
    grid_map::SubmapIterator it(map, startIndex, size);
    for (; !it.isPastEnd(); ++it) {
      grid_map::Position currentPosition;
      map.getPosition(*it, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (grid_map::CircleIterator circleIt(map, currentPosition, radius); !circleIt.isPastEnd();
        ++circleIt)
      {
        if (!map.isValid(*circleIt, "elevation_noisy")) {continue;}
        grid_map::Position currentPositionInCircle;
        map.getPosition(*circleIt, currentPositionInCircle);

        // Computed weighted mean based on Euclidian distance.
        double distance = (currentPosition - currentPositionInCircle).norm();
        double weight = pow(radius - distance, 2);
        mean += weight * map.at("elevation_noisy", *circleIt);
        sumOfWeights += weight;
      }

      map.at("elevation_filtered", *it) = mean / sumOfWeights;
    }

    // Show absolute difference and compute mean squared error.
    map.add("error", (map.get("elevation_filtered") - map.get("elevation")).cwiseAbs());

    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    RCLCPP_INFO_THROTTLE(node.get_logger(), clock, 1000, "Grid map published.");

    rclcpp::spin_some(node.get_node_base_interface());
    rate.sleep();

    return 0;
  }
}
