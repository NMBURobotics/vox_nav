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
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>

#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_planning/plugins/elevation_state_space.hpp"


namespace vox_nav_planning
{

  class OptimalElevationPlanner : public vox_nav_planning::PlannerCore
  {

  public:
/**
 * @brief Construct a new OptimalElevationPlanner object
 *
 */
    OptimalElevationPlanner();

/**
 * @brief Destroy the OptimalElevationPlanner object
 *
 */
    ~OptimalElevationPlanner();

    /**
     * @brief
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
    *
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

  protected:
    rclcpp::Logger logger_{rclcpp::get_logger("optimal_elevation_planner")};
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
    ompl::base::OptimizationObjectivePtr octocost_optimization_;

    ompl::base::StateSpacePtr state_space_;
    std::shared_ptr<ompl::base::RealVectorBounds> z_bounds_;
    std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      super_voxel_adjacency_marker_pub_;
    std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters_;
  };

  typedef float cost;
  // euclidean distance heuristic
  template<class Graph, class CostType, class SuperVoxelClustersType>
  class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
  {
  public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(SuperVoxelClustersType sc, Vertex goal_vertex, Graph g)
    : supervoxel_clusters_(sc), goal_vertex_(goal_vertex), g_(g) {}
    CostType operator()(Vertex u)
    {
      auto u_vertex_label = g_[u].label;
      auto goal_vertex_label = g_[u].label;
      auto u_supervoxel_centroid = supervoxel_clusters_.at(u_vertex_label)->centroid_;
      auto goal_supervoxel_centroid = supervoxel_clusters_.at(goal_vertex_label)->centroid_;
      CostType dx = u_supervoxel_centroid.x - goal_supervoxel_centroid.x;
      CostType dy = u_supervoxel_centroid.y - goal_supervoxel_centroid.y;
      CostType dz = u_supervoxel_centroid.z - goal_supervoxel_centroid.z;
      return ::sqrt(dx * dx + dy * dy + dz * dz);
    }

  private:
    SuperVoxelClustersType supervoxel_clusters_;
    Vertex goal_vertex_;
    Graph g_;
  };

  struct found_goal {};   // exception for termination
  // visitor that terminates when we find the goal
  template<class Vertex>
  class astar_goal_visitor : public boost::default_astar_visitor
  {
  public:
    astar_goal_visitor(Vertex goal)
    : m_goal(goal) {}
    template<class Graph>
    void examine_vertex(Vertex u, Graph & g)
    {
      if (u == m_goal) {
        throw found_goal();
      }
    }

  private:
    Vertex m_goal;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
