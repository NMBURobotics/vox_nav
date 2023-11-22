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

#ifndef VOX_NAV_MISC__TRAVERSABILITY_ESTIMATOR_HPP_
#define VOX_NAV_MISC__TRAVERSABILITY_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_utilities/map_manager_helpers.hpp>

namespace vox_nav_misc
{
struct CostRegressionParams
{
  double uniform_sample_radius;
  double surfel_radius;
  double max_allowed_tilt;
  double max_allowed_point_deviation;
  double max_allowed_energy_gap;
  double node_elevation_distance;
  double plane_fit_threshold;
  double robot_mass;
  double average_speed;
  double max_color_range;
  std::vector<double> cost_critic_weights;
  CostRegressionParams()
    : uniform_sample_radius(0.2)
    , surfel_radius(0.1)
    , max_allowed_tilt(10)
    , max_allowed_point_deviation(0.1)
    , max_allowed_energy_gap(0.1)
    , node_elevation_distance(1)
    , plane_fit_threshold(10)
    , robot_mass(0.1)
    , average_speed(0.1)
    , max_color_range(255.0)
    , cost_critic_weights({ 0.33, 0.33, 0.33 })
  {
  }
};

class TraversabilityEstimator : public rclcpp::Node
{
public:
  TraversabilityEstimator();

  ~TraversabilityEstimator();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_cloud_publisher_;

  int traversable_cloud_publisher_counter_ = 0;

  // tf buffers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //  see the struct, it is used to keep cost regression params orginzed
  CostRegressionParams cost_params_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr regressCosts(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                      const std_msgs::msg::Header& header);
};

}  // namespace vox_nav_misc

#endif  // VOX_NAV_MISC__TRAVERSABILITY_ESTIMATOR_HPP_
