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
        setup();
      }

      struct VertexProperty
      {
        ompl::base::State * state{nullptr};
        std::uintptr_t state_label{0};
        std::size_t id{0};
        double g{0.0};
        double rhs{0.0};
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

    private:
      int batch_size_{500};
      double radius_{1.5}; // max edge length
      int max_neighbors_{10};
      double min_dist_between_vertices_{0.025};
      bool use_valid_sampler_{false};

      /** \brief State sampler */
      base::StateSamplerPtr sampler_{nullptr};

      const SpaceInformation * siC_{nullptr};

      /** \brief State sampler */
      base::ValidStateSamplerPtr valid_state_sampler_{nullptr};

      std::shared_ptr<base::PathLengthDirectInfSampler> informed_sampler_{nullptr};

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_{nullptr};

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
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heur_pub_;
      rclcpp::Node::SharedPtr node_;


      friend class GenericDistanceHeuristic;
      template<class Graph, class VertexProperty, class CostType>
      class GenericDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        GenericDistanceHeuristic(AITStarKin * alg, VertexProperty * goal)
        : alg_(alg),
          goal_(goal)
        {
        }
        double operator()(vertex_descriptor i)
        {
          return alg_->distanceFunction(alg_->getVertex(i), goal_);
        }

      private:
        AITStarKin * alg_;
        VertexProperty * goal_;
      };  // GenericDistanceHeuristic


      // exception for termination
      struct FoundGoal {};

      template<class Vertex>
      class custom_goal_visitor : public boost::default_astar_visitor
      {
      public:
        custom_goal_visitor(Vertex start_vertex, int * num_visits, AITStarKin * alg)
        : start_vertex_(start_vertex),
          num_visits_(num_visits),
          alg_(alg)
        {
        }
        template<class Graph>
        void examine_vertex(Vertex u, Graph & g)
        {
          // check whether examined vertex was goal, if yes throw
          ++(*num_visits_);
          if (u == start_vertex_) {
            throw FoundGoal();
          }
        }

      private:
        Vertex start_vertex_;
        int * num_visits_;
        AITStarKin * alg_;
      };

      VertexProperty start_vertex_;
      VertexProperty goal_vertex_;

      GraphT g_;

      void visualizeRGG(const GraphT & g);

      void visualizeLPAHuer(const std::list<vertex_descriptor> & heur);

      void generateBatchofSamples(
        int batch_size,
        bool use_valid_sampler,
        std::vector<ompl::base::State *> & samples);

    };
  }   // namespace control
}  // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_
