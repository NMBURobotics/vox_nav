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

#ifndef VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_
#define VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_

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

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"

#include <limits>
#include <cstdint>

namespace ompl
{
  namespace control
  {

    class AITStarKin : public base::Planner
    {
    public:
      /** \brief Constructor */
      AITStarKin(const SpaceInformationPtr & si);

      ~AITStarKin() override;

      void setup() override;

      /** \brief Continue solving for some amount of time. Return true if solution was found. */
      base::PlannerStatus solve(const base::PlannerTerminationCondition & ptc) override;

      void getPlannerData(base::PlannerData & data) const override;

      /** \brief Clear datastructures. Call this function if the
          input data to the planner has changed and you do not
          want to continue planning */
      void clear() override;

      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      /** \brief Set a different nearest neighbors datastructure */
      template<template<typename T> class NN>
      void setNearestNeighbors()
      {
        if (nn_ && nn_->size() != 0) {
          OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        }
        clear();
        nn_ = std::make_shared<NN<VertexProperty *>>();
        control_nn_ = std::make_shared<NN<VertexProperty *>>();
        setup();
      }

      struct VertexProperty
      {
        std::vector<ompl::base::State *> path;
        ompl::base::State * state{nullptr};
        ompl::control::Control * control{nullptr};
        unsigned int control_duration{0};
        std::size_t id{0};
        double g{1.0e+3};
        bool blacklisted{false};
      };

      double distanceFunction(const VertexProperty * a, const VertexProperty * b) const
      {
        return si_->distance(a->state, b->state);
      }

      double distanceFunction(const base::State * a, const base::State * b) const
      {
        return si_->distance(a, b);
      }

      const VertexProperty * getVertex(std::size_t id)
      {
        return &g_[id];
      }

      VertexProperty * getVertexMutable(std::size_t id)
      {
        return &g_[id];
      }

    private:
      int batch_size_{500};
      double radius_{1.5}; // max edge length
      int max_neighbors_{10};
      double min_dist_between_vertices_{0.1};
      bool use_valid_sampler_{true};
      int k_number_of_controls_{1};
      static bool const use_astar_hueristic_{false};

      /** \brief State sampler */
      base::StateSamplerPtr sampler_{nullptr};

      const SpaceInformation * siC_{nullptr};

      /** \brief State sampler */
      base::ValidStateSamplerPtr valid_state_sampler_{nullptr};

      std::shared_ptr<base::PathLengthDirectInfSampler> path_informed_sampler_{nullptr};

      std::shared_ptr<base::RejectionInfSampler> rejection_informed_sampler_{nullptr};

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_{nullptr};

      ompl::base::Cost bestCost_{std::numeric_limits<double>::infinity()};

      DirectedControlSamplerPtr controlSampler_;

      typedef float Cost;
      typedef boost::adjacency_list<
          boost::setS,          // edge
          boost::vecS,          // vertex
          boost::undirectedS,   // type
          VertexProperty,       // vertex property
          boost::property<boost::edge_weight_t, Cost>> // edge property
        GraphT;

      typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;
      typedef GraphT::vertex_descriptor vertex_descriptor;
      typedef GraphT::edge_descriptor edge_descriptor;
      typedef GraphT::vertex_iterator vertex_iterator;
      typedef std::pair<int, int> edge;
      using WeightProperty = boost::property<boost::edge_weight_t, Cost>;

      std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> nn_;
      std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> control_nn_;

      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rgg_graph_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometric_path_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_graph_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_path_pub_;
      rclcpp::Node::SharedPtr node_;

      friend class GenericDistanceHeuristic;
      template<class Graph, class VertexProperty, class CostType>
      class GenericDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        GenericDistanceHeuristic(AITStarKin * alg, VertexProperty * goal, bool control = false)
        : alg_(alg),
          goal_(goal),
          control_(control)
        {
        }
        double operator()(vertex_descriptor i)
        {
          return alg_->distanceFunction(alg_->getVertex(i), goal_);
        }

      private:
        AITStarKin * alg_{nullptr};
        VertexProperty * goal_{nullptr};
        bool control_{false};
      };  // GenericDistanceHeuristic

      friend class PrecomputedCostHeuristic;
      template<class Graph, class CostType>
      class PrecomputedCostHeuristic : public boost::astar_heuristic<Graph, CostType>
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        PrecomputedCostHeuristic(AITStarKin * alg)
        : alg_(alg)
        {
        }
        double operator()(vertex_descriptor i)
        {
          double cost{std::numeric_limits<double>::infinity()};
          // verify that the state is valid
          if (alg_->si_->isValid(alg_->getVertex(i)->state)) {
            cost = alg_->getVertex(i)->g;
          } else {
            alg_->getVertexMutable(i)->blacklisted = true;
          }
          return cost;
        }

      private:
        AITStarKin * alg_;
      };  // PrecomputedCostHeuristic


      // exception for termination
      struct FoundVertex {};
      template<class Vertex>
      class SimpleVertexVisitor : public boost::default_astar_visitor
      {
      public:
        SimpleVertexVisitor(Vertex goal_vertex, int * num_visits)
        : goal_vertex_(goal_vertex),
          num_visits_(num_visits)
        {
        }
        template<class Graph>
        void examine_vertex(Vertex u, Graph & g)
        {
          // check whether examined vertex was goal, if yes throw
          ++(*num_visits_);
          if (u == goal_vertex_) {
            throw FoundVertex();
          }
        }

      private:
        Vertex goal_vertex_;
        int * num_visits_;
      };  // SimpleVertexVisitor


      VertexProperty * start_vertex_{nullptr};
      VertexProperty * goal_vertex_{nullptr};

      GraphT g_;
      GraphT g_control_;

      static void visualizeRGG(
        const GraphT & g,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
        const std::string & ns,
        const double & blue_color,
        const vertex_descriptor & start_vertex,
        const vertex_descriptor & goal_vertex);

      static void visualizePath(
        const GraphT & g,
        const std::list<vertex_descriptor> & path,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
        const std::string & ns);

      void generateBatchofSamples(
        int batch_size,
        bool use_valid_sampler,
        std::vector<ompl::base::State *> & samples);

    };
  }   // namespace control
}  // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_
