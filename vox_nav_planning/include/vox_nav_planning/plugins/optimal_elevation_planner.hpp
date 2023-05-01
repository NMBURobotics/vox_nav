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

#ifndef VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "vox_nav_utilities/boost_graph_utils.hpp"

namespace vox_nav_planning
{

  /**
   * @brief Optimal Elevation Planner is a planner plugin for vox_nav_planning based on a global point cloud map
   *        The global map is provided by vox_nav_map_server, and is a point cloud map with elevation information
   *        as well as traversability information. The method creates supervoxel adjacency graph from the point cloud
   *        The A* search is performed on this graph to find the optimal elevation path from start to goal.
   *
   */
  class OptimalElevationPlanner : public vox_nav_planning::PlannerCore
  {

  public:
    /**
     * @brief Construct a new OptimalElevationPlanner object
     *
     */
    OptimalElevationPlanner();

    /**
     * @brief Destroy the Optimal Elevation Planner object
     *
     */
    ~OptimalElevationPlanner();

    /**
     * @brief Method to initialize the planner plugin
     *
     */
    void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) override;

    /**
     * @brief Method create the plan from a starting and ending goal.
     *
     * @param start The starting pose of the robot
     * @param goal  The goal pose of the robot
     * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
     */
    std::vector<geometry_msgs::msg::PoseStamped> createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

    /**
    * @brief
    * @param state
    * @return true
    * @return false
    */
    bool isStateValid(const ompl::base::State * state) override;

    /**
     * @brief
     *
     * @param msg
     */
    void nodePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief
     *
     * @param si
     * @return ompl::base::ValidStateSamplerPtr
     */
    ompl::base::ValidStateSamplerPtr allocValidStateSampler(
      const ompl::base::SpaceInformation * si);

    /**
   * @brief Get the Overlayed Start and Goal poses, only x and y are provided for goal ,
   * but internally planner finds closest valid node on octomap and reassigns goal to this pose
   *
   * @return std::vector<geometry_msgs::msg::PoseStamped>
   */
    std::vector<geometry_msgs::msg::PoseStamped> getOverlayedStartandGoal() override;

    /**
     * @brief
     *
     */
    void setupMap() override;

    /**
     * @brief
     *
     * @param a
     * @param b
     * @return true
     * @return false
     */
    bool isEdgeinCollision(
      const pcl::PointXYZRGBA & a,
      const pcl::PointXYZRGBA & b);

  protected:
    typedef std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> SuperVoxelClusters;
    rclcpp::Logger logger_{rclcpp::get_logger("optimal_elevation_planner")};

    // Get the traversability map from vox_nav_map_server
    rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr
      get_traversability_map_client_;

    // Surfels centers are elevated by node_elevation_distance_, and are stored in this
    // octomap, this maps is used by planner to sample states that are
    // strictly laying on ground but not touching. So it constrains the path to be on ground
    // while it can elevate thorogh ramps or slopes
    std::shared_ptr<octomap::OcTree> elevated_surfel_octomap_octree_;
    // it is also required to have orientation information of surfels, they are kept in
    // elevated_surfel_poses_msg_
    geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
    pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr elevated_traversable_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;
    std::shared_ptr<fcl::CollisionObjectf> elevated_surfels_collision_object_;

    ompl::base::StateSpacePtr state_space_;
    std::shared_ptr<ompl::base::RealVectorBounds> z_bounds_;
    std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

    // SuperVoxel Clustering variables
    // https://pcl.readthedocs.io/en/latest/supervoxel_clustering.html#supervoxel-clustering
    // boost graph is constructed through supervoxels of elevated surfels
    // Optimal planning basing in Astar is perfromed on top of this graph
    // refer to PCL supervoxel_clustering for more details on algorithm
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      super_voxel_adjacency_marker_pub_;
    SuperVoxelClusters supervoxel_clusters_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_clusters_pub_;

    bool supervoxel_disable_transform_;
    float supervoxel_resolution_;
    float supervoxel_seed_resolution_;
    float supervoxel_color_importance_;
    float supervoxel_spatial_importance_;
    float supervoxel_normal_importance_;

    float distance_penalty_weight_;
    float elevation_penalty_weight_;

    std::string graph_search_method_; // astar ? , diskstra ?

    std::string selected_se2_space_name_;
    ompl::base::ElevationStateSpace::SE2StateType se2_space_type_;
    // curve radius for reeds and dubins only
    double rho_;

    // octomap acquired from original PCD map
    std::shared_ptr<octomap::OcTree> original_octomap_octree_;
    std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;
    std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;
    // Better t keep this parameter consistent with map_server, 0.2 is a OK default fo this
    double octomap_voxel_size_;
    // global mutex to guard octomap
    std::mutex octomap_mutex_;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
