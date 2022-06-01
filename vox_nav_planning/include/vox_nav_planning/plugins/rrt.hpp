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

#ifndef VOX_NAV_PLANNING__PLUGINS__RRT_HPP_
#define VOX_NAV_PLANNING__PLUGINS__RRT_HPP_

#include "vox_nav_planning/plugins/rrt_utils.hpp"
#include "vox_nav_planning/planner_core.hpp"

namespace vox_nav_planning
{
  class RRT : public vox_nav_planning::PlannerCore
  {
  public:
    RRT(
      Node start,
      Node goal,
      double expand_dis = 3.0,
      double path_resolution = 0.5,
      double goal_sample_rate = 5,
      double max_iter = 500,
      double robot_radius = 0.0);
    ~RRT();


    void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) override;

    static int nearestNodeIndex(const pcl::PointCloud<Node>::Ptr nodes, const Node random_node);

    static Node getRandomNode(const pcl::PointCloud<Node>::Ptr nodes, const Node random_node);

    std::vector<geometry_msgs::msg::PoseStamped> createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

    bool isStateValid(const Node * state);

    void nodePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    void setupMap() override;

  private:
    Node start_;
    Node goal_;
    pcl::PointCloud<Node>::Ptr nodes_;
    double expand_dis_;
    double path_resolution_;
    double goal_sample_rate_;
    int max_iter_;
    double robot_radius_;

    rclcpp::Logger logger_{rclcpp::get_logger("elevation_control_planner")};
    // Surfels centers are elevated by node_elevation_distance_, and are stored in this
    // octomap, this maps is used by planner to sample states that are
    // strictly laying on ground but not touching. So it constrains the path to be on ground
    // while it can elevate thorogh ramps or slopes
    std::shared_ptr<octomap::OcTree> elevated_surfel_octomap_octree_;
    // it is also required to have orientation information of surfels, they are kept in
    // elevated_surfel_poses_msg_
    geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
    pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;
    std::shared_ptr<fcl::CollisionObject> elevated_surfels_collision_object_;
  };


}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__RRT_HPP_
