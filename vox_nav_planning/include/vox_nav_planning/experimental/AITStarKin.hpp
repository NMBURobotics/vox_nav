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

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/sst/SST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/LPAstarOnGraph.h"

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
        std::string name;
        ompl::base::State * state;
        std::uintptr_t state_label;
        std::size_t id;
        double g;
        double rhs;
      };

      double distanceFunction(const VertexProperty * a, const VertexProperty * b) const
      {
        return si_->distance(a->state, b->state);
      }

      double distanceFunction(const base::State * a, const base::State * b) const
      {
        return si_->distance(a, b);
      }

      void addVertex(const VertexProperty * a)
      {
        // Add goal and start to graph
        vertex_descriptor g = boost::add_vertex(g_);
        vertex_descriptor apx = boost::add_vertex(graphApx_);
        vertex_descriptor lb = boost::add_vertex(graphLb_);
        g_[g].state = a->state;
        g_[g].state_label = reinterpret_cast<std::uintptr_t>(a->state);
        g_[g].id = g;
        graphApx_[apx] = g_[g];
        graphLb_[lb] = g_[g];
      }

      void addEdgeApx(VertexProperty * a, VertexProperty * b, double c)
      {
        WeightProperty w(c);
        boost::add_edge(a->id, b->id, w, graphApx_);
        LPAstarApx_->insertEdge(a->id, b->id, c);
        LPAstarApx_->insertEdge(b->id, a->id, c);
      }

      void addEdgeLb(const VertexProperty * a, const VertexProperty * b, double c)
      {
        WeightProperty w(c);
        boost::add_edge(a->id, b->id, w, graphLb_);
        LPAstarLb_->insertEdge(a->id, b->id, c);
        LPAstarLb_->insertEdge(b->id, a->id, c);
      }

      const VertexProperty * getVertex(std::size_t id)
      {
        return &g_[id];
      }

      void generateBatchofSamples(
        int batch_size,
        bool use_valid_sampler,
        std::vector<ompl::base::State *> & samples);

    private:
      int batch_size_{500};
      double radius_{1.5};

      /** \brief State sampler */
      base::StateSamplerPtr sampler_;

      const SpaceInformation * siC_;

      /** \brief State sampler */
      base::ValidStateSamplerPtr valid_state_sampler_;

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_;

      typedef float Cost;
      typedef boost::adjacency_list<
          boost::vecS,          // edge
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
      rclcpp::Node::SharedPtr node_;

      // allow CostEstimatorApx access to private members
      friend class CostEstimatorApx;
      class CostEstimatorApx
      {
      public:
        CostEstimatorApx(AITStarKin * alg)
        : alg_(alg)
        {
        }
        double operator()(std::size_t i)
        {
          double lb_estimate = (*(alg_->LPAstarLb_))(i);
          if (lb_estimate != std::numeric_limits<double>::infinity()) {
            return lb_estimate;
          }
          return alg_->distanceFunction(alg_->getVertex(i), alg_->start_vertex_);
        }

      private:
        AITStarKin * alg_;
      };  // CostEstimatorApx

      class CostEstimatorLb
      {
      public:
        CostEstimatorLb(base::Goal * goal, AITStarKin * alg)
        : goal_(goal), alg_(alg)
        {
        }
        double operator()(std::size_t i)
        {
          double dist = 0.0;
          goal_->isSatisfied(alg_->getVertex(i)->state, &dist);

          return dist;
        }

      private:
        base::Goal * goal_;
        AITStarKin * alg_;
      };  // CostEstimatorLb

      using LPAstarApx = LPAstarOnGraph<GraphT, CostEstimatorApx>;
      using LPAstarLb = LPAstarOnGraph<GraphT, CostEstimatorLb>;

      LPAstarApx * LPAstarApx_{nullptr};             // rooted at target
      LPAstarLb * LPAstarLb_{nullptr};               // rooted at source

      VertexProperty * start_vertex_;
      VertexProperty * goal_vertex_;

      GraphT g_;
      GraphT graphLb_;
      GraphT graphApx_;

      void visulizeRGG(const GraphT & g);

    };
  }   // namespace control
}  // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_
