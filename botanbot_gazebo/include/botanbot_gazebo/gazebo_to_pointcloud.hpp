/*
 * Copyright 2021 Fetullah Atas, Norwegian University of Life Sciences

 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLUGIN_BUILD_POINTCLOUD_HH
#define PLUGIN_BUILD_POINTCLOUD_HH

#include <iostream>
#include <math.h>

#include <gazebo_ros/node.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <botanbot_gazebo/gazebo_to_octomap_common.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <botanbot_msgs/srv/get_point_cloud.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gazebo
{

class PointCloudFromGazeboWorld : public WorldPlugin
{
public:
  PointCloudFromGazeboWorld()
  : WorldPlugin() {}
  virtual ~PointCloudFromGazeboWorld();

protected:
  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  bool CheckIfInterest(
    const ignition::math::Vector3d & central_point,
    gazebo::physics::RayShapePtr ray,
    const double leaf_size);

  void FloodFill(
    const ignition::math::Vector3d & seed_point,
    const ignition::math::Vector3d & bounding_box_origin,
    const ignition::math::Vector3d & bounding_box_lengths,
    const double leaf_size);

  void CreatePointCloud(const botanbot_msgs::srv::GetPointCloud::Request & msg);

private:
  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<botanbot_msgs::srv::GetPointCloud>::SharedPtr srv_;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

};

} // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_OCTOMAP_PLUGIN_H
