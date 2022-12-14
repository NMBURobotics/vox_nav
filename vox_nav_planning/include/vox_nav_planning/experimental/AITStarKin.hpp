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
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

#include <limits>

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

    protected:
      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      /** \brief State sampler */
      base::StateSamplerPtr sampler_;

      const SpaceInformation * siC_;

      /** \brief State sampler */
      base::ValidStateSamplerPtr valid_state_sampler_;

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_;

      double goalBias_{0.05};
      double connect_circle_dist_{10.0};
      double expand_dis_{2.5};
      double goal_tolerance_{0.5};

      /** \brief The random number generator */
      RNG rng_;

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

      struct VertexProperty
      {
        std::uint32_t label;
        std::string name;
        ompl::base::State * state;
      };

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

      std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> nn_;

      double distanceFunction(const VertexProperty * a, const VertexProperty * b) const
      {
        return si_->distance(a->state, b->state);
      }

      double distanceFunction(const base::State * a, const base::State * b) const
      {
        return si_->distance(a, b);
      }

    };
  }   // namespace control
}  // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__AITSTARKIN_HPP_
