// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_PLANNING__RRT__KINOPLANNER_HPP_
#define VOX_NAV_PLANNING__RRT__KINOPLANNER_HPP_

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/LPAstarOnGraph.h"
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/util/GeometricEquations.h"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"

#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>
#include <limits>
#include <cstdint>

namespace ompl
{
namespace control
{
/**
   @anchor cKinoPlanner
   @par Short description
   \ref Informed Simultaneous Geometric and Control Planner (KinoPlanner).
   The implementation is based on boost graph library.
   An accompanying paper explaining novelities of this planner will be published soon.
   @par External documentation
   TBD
*/
struct Parameters
{
  /** \brief All configurable parameters of KinoPlanner. */

  /** \brief The number of samples to be added to graph in each iteration. */
  int batch_size_{ 100 };
};

class KinoPlanner : public base::Planner
{
public:
  /** \brief Constructor */
  KinoPlanner(const SpaceInformationPtr& si);

  /** \brief Destructor */
  ~KinoPlanner() override;

  /** \brief Setup the planner */
  void setup() override;

  /** \brief Continue solving for some amount of time. Return true if solution was found. */
  base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc) override;

  /** \brief Get the internal data owned by planner, NOT IMPLEMENTED */
  void getPlannerData(base::PlannerData& data) const override;

  /** \brief Clear datastructures. Call this function if the
      input data to the planner has changed and you do not
      want to continue planning */
  void clear() override;

  /** \brief Free the memory allocated by this planner. That is mostly in nearest neihbours. */
  void freeMemory();

  /** \brief Properties of boost graph vertex, both geometriuc and control graphs share this vertex property.
   *  Some of the elements are not used in geometric graph (e.g., control, control_duration). */
  struct VertexProperty
  {
    ompl::base::State* state{ nullptr };
    ompl::control::Control* control{ nullptr };
    unsigned int control_duration{ 0 };
    std::size_t id{ 0 };
    std::size_t parent_id{ 0 };
    double g{ 1.0e+3 };
    bool blacklisted{ false };
    bool is_root{ false };
  };

  /** \brief Compute distance between Vertexes (actually distance between contained states) */
  double distanceFunction(const VertexProperty* a, const VertexProperty* b) const;

  /** \brief Compute distance between states */
  double distanceFunction(const base::State* a, const base::State* b) const;

  void setBatchSize(int batch_size);
  int getBatchSize() const;

private:
  /** \brief All configurable parames live here. */
  Parameters params_;

  /** \brief The radius to construct edges in construction of RGG, this is meant to be used in geometric graph,
   * determines max edge length. */
  double radius_{ std::numeric_limits<double>::infinity() };

  /** \brief A constant for the computation of the number of neighbors when using a k-nearest model. */
  std::size_t kRGG_{ std::numeric_limits<std::size_t>::max() };

  /** \brief Control space information */
  const SpaceInformation* siC_{ nullptr };

  /** \brief Control sampler */
  ControlSamplerPtr controlSampler_{ nullptr };

  /** \brief The optimization objective. */
  base::OptimizationObjectivePtr opt_{ nullptr };

  /** \brief The random number generator */
  RNG rng_;

  /** \brief The NN datastructure for control graph */
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>> nnControl_;

  /** \brief Keep a global copy of start and goal vertex properties*/
  VertexProperty* startVertex_{ nullptr };
  VertexProperty* goalVertex_{ nullptr };

  Eigen::VectorXd vertexes_mean_;
  Eigen::MatrixXd vertexes_covariance_;

  // RVIZ Visualization of planner progess, this will be removed in the future
  /** \brief static method to visulize a graph in RVIZ*/
  static void visualizeRGG(const std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& g,
                           const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
                           const std::string& ns, const std_msgs::msg::ColorRGBA& color,
                           const VertexProperty* start_vertex, const VertexProperty* goal_vertex,
                           const int& state_space_type);

  /** \brief static method to visulize a path in RVIZ*/
  static void visualizePath(const std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& g,
                            const std::list<int>& path,
                            const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
                            const std::string& ns, const std_msgs::msg::ColorRGBA& color, const int& state_space_type);

  static void visualizePath(const std::shared_ptr<PathControl>& path,
                            const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
                            const std::string& ns, const std_msgs::msg::ColorRGBA& color, const int& state_space_type);

  /** \brief get std_msgs::msg::ColorRGBA given the color name with a std::string*/
  static std_msgs::msg::ColorRGBA getColor(std::string& color);

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rgg_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometric_path_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr first_control_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr second_control_graph_pub_;

  /** \brief The publishers for the geometric and control graph/path visulization*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_path_pub_;

  /** \brief The node*/
  rclcpp::Node::SharedPtr node_;

};  // class KinoPlanner
}  // namespace control
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__RRT__KINOPLANNER_HPP_
