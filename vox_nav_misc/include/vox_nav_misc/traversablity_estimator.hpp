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
    : uniform_sample_radius(0.2),
      surfel_radius(0.1),
      max_allowed_tilt(10),
      max_allowed_point_deviation(0.1),
      max_allowed_energy_gap(0.1),
      node_elevation_distance(1),
      plane_fit_threshold(10),
      robot_mass(0.1),
      average_speed(0.1),
      max_color_range(255.0),
      cost_critic_weights({0.33, 0.33, 0.33})
    {}
  };


  class TraversabilityEstimator : public rclcpp::Node
  {
  public:
    TraversabilityEstimator();

    ~TraversabilityEstimator();

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

    // Publish supervoxel clusters
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr supervoxel_clusters_publisher_;

    // Visualize supervoxel graph
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr supervoxel_graph_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_cloud_publisher_;

    typedef std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> SuperVoxelClusters;
    SuperVoxelClusters supervoxel_clusters_;

    bool supervoxel_disable_transform_;
    float supervoxel_resolution_;
    float supervoxel_seed_resolution_;
    float supervoxel_color_importance_;
    float supervoxel_spatial_importance_;
    float supervoxel_normal_importance_;

    // tf buffers
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //  see the struct, it is used to keep cost regression params orginzed
    CostRegressionParams cost_params_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  regressCosts(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const std_msgs::msg::Header & header);

    void fillSuperVoxelMarkersfromAdjacency(
      const std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> & supervoxel_clusters,
      const std::multimap<std::uint32_t, std::uint32_t> & supervoxel_adjacency,
      const std_msgs::msg::Header & header,
      visualization_msgs::msg::MarkerArray & marker_array)
    {
      int index = 0;
      // To make a graph of the supervoxel adjacency,
      // we need to iterate through the supervoxel adjacency multimap
      for (auto label_itr = supervoxel_adjacency.cbegin();
        label_itr != supervoxel_adjacency.cend(); )
      {
        // First get the label
        std::uint32_t supervoxel_label = label_itr->first;
        // Now get the supervoxel corresponding to the label
        auto supervoxel = supervoxel_clusters.at(supervoxel_label);

        visualization_msgs::msg::Marker line_strip;
        line_strip.header = header;
        line_strip.ns = "supervoxel_markers_ns";
        line_strip.id = index;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.lifetime = rclcpp::Duration::from_seconds(0.5);
        line_strip.scale.x = 0.1;
        geometry_msgs::msg::Point point;
        point.x = supervoxel->centroid_.x;
        point.y = supervoxel->centroid_.y;
        point.z = supervoxel->centroid_.z;
        std_msgs::msg::ColorRGBA yellow_color;
        yellow_color.r = 1.0;
        yellow_color.g = 1.0;
        yellow_color.a = 0.4;
        line_strip.points.push_back(point);
        line_strip.colors.push_back(yellow_color);

        visualization_msgs::msg::Marker sphere;
        sphere.header = header;
        sphere.ns = "supervoxel_markers_ns";
        sphere.id = index + 10000;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
        sphere.pose.position = point;
        sphere.scale.x = 0.3;
        sphere.scale.y = 0.3;
        sphere.scale.z = 0.3;
        sphere.color.a = 1.0;
        sphere.color.g = 1.0;
        sphere.color.b = 1.0;

        for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
          adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
        {
          auto neighbor_supervoxel =
            supervoxel_clusters.at(adjacent_itr->second);

          geometry_msgs::msg::Point n_point;
          n_point.x = neighbor_supervoxel->centroid_.x;
          n_point.y = neighbor_supervoxel->centroid_.y;
          n_point.z = neighbor_supervoxel->centroid_.z;
          line_strip.points.push_back(n_point);
          line_strip.colors.push_back(yellow_color);
        }
        // Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
        index++;

        marker_array.markers.push_back(sphere);
        marker_array.markers.push_back(line_strip);
      }
    }
  };

} // namespace vox_nav_misc

#endif // VOX_NAV_MISC__TRAVERSABILITY_ESTIMATOR_HPP_
