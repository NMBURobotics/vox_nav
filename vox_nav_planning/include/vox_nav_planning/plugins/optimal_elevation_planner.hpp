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
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"

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
    struct VertexProperty
    {
      std::uint32_t label;
      std::string name;
    };

    typedef float Cost;

    // specify some types
    typedef boost::adjacency_list<
        boost::setS,            // edge
        boost::vecS,            // vertex
        boost::undirectedS,     // type
        VertexProperty,         // vertex property
        boost::property<boost::edge_weight_t, Cost>> // edge property
      GraphT;

    typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;
    typedef GraphT::vertex_descriptor vertex_descriptor;
    typedef GraphT::edge_descriptor edge_descriptor;
    typedef GraphT::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;
    typedef std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> SuperVoxelClusters;

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr elevated_traversable_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;
    std::shared_ptr<fcl::CollisionObject> elevated_surfels_collision_object_;

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
  };

  // euclidean distance heuristic
  template<class Graph, class CostType, class SuperVoxelClustersPtr>
  class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
  {
  public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(SuperVoxelClustersPtr sc, Vertex goal_vertex, Graph g)
    : supervoxel_clusters_(sc), goal_vertex_(goal_vertex), g_(g)
    {
    }
    CostType operator()(Vertex u)
    {
      auto u_vertex_label = g_[u].label;
      auto goal_vertex_label = g_[goal_vertex_].label;
      auto u_supervoxel_centroid = supervoxel_clusters_->at(u_vertex_label)->centroid_;
      auto goal_supervoxel_centroid = supervoxel_clusters_->at(goal_vertex_label)->centroid_;
      CostType dx = u_supervoxel_centroid.x - goal_supervoxel_centroid.x;
      CostType dy = u_supervoxel_centroid.y - goal_supervoxel_centroid.y;
      CostType dz = u_supervoxel_centroid.z - goal_supervoxel_centroid.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

  private:
    SuperVoxelClustersPtr supervoxel_clusters_;
    Vertex goal_vertex_;
    Graph g_;
  };

  // exception for termination
  struct FoundGoal {};
  template<class Vertex>
  class custom_goal_visitor : public boost::default_astar_visitor
  {
  public:
    custom_goal_visitor(Vertex goal_vertex, int * num_visits)
    : goal_vertex_(goal_vertex), num_visits_(num_visits)
    {
    }
    template<class Graph>
    void examine_vertex(Vertex u, Graph & g)
    {
      ++(*num_visits_);
      if (u == goal_vertex_) {
        throw FoundGoal();
      }
    }

  private:
    Vertex goal_vertex_;
    int * num_visits_;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__OPTIMAL_ELEVATION_PLANNER_HPP_
