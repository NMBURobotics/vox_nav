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

#ifndef VOX_NAV_PLANNING__RRT__LQRRRTSTAR_HPP_
#define VOX_NAV_PLANNING__RRT__LQRRRTSTAR_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "nav_msgs/msg/path.hpp"
#include "vox_nav_planning/rrt/LQRPlanner.hpp"

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
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

    class LQRRRTStar : public base::Planner
    {
    public:
      /** \brief Constructor */
      LQRRRTStar(const SpaceInformationPtr & si);

      ~LQRRRTStar() override;

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
        nn_ = std::make_shared<NN<Node *>>();
        setup();
      }

    protected:
      /** \brief Representation of a motion

          This only contains pointers to parent motions as we
          only need to go backwards in the tree. */
      class Node
      {
      public:
        Node() = default;

        /** \brief Constructor that allocates memory for the state and the control */
        Node(const SpaceInformation * si)
        : state_(si->allocState())
        {
        }

        virtual ~Node() = default;

        virtual base::State * getState() const
        {
          return state_;
        }
        virtual Node * getParent() const
        {
          return parent_;
        }

        base::Cost cost_{0};

        /** \brief The state contained by the motion */
        base::State * state_{nullptr};

        /** \brief The parent motion in the exploration tree */
        Node * parent_{nullptr};

        std::vector<base::State *> path_;

      };

      Node * steer(Node * from_node, Node * to_node, double * relative_cost)
      {
        std::vector<base::State *> resulting_path;
        lqr_planner_->compute_LQR_plan(from_node->state_, to_node->state_, resulting_path);

        std::vector<double> clen;
        if (resulting_path.size() > 1) {

          for (int i = 1; i < resulting_path.size(); i++) {
            double this_segment_dist = distanceFunction(resulting_path[i], resulting_path[i - 1]);
            clen.push_back(this_segment_dist);
          }

          auto * new_node = new Node(siC_);
          new_node->state_ = si_->allocState();

          auto * new_node_cstate =
            new_node->state_->as<ompl::base::ElevationStateSpace::StateType>();

          auto * last_node_cstate =
            resulting_path.back()->as<ompl::base::ElevationStateSpace::StateType>();

          new_node_cstate->setSE2(
            last_node_cstate->getSE2()->getX(),
            last_node_cstate->getSE2()->getY(),
            last_node_cstate->getSE2()->getYaw());
          new_node_cstate->setZ(last_node_cstate->getZ()->values[0]);

          double cost = std::accumulate(clen.begin(), clen.end(), 0.0);
          *relative_cost = cost;

          auto new_cost = new_node->cost_.value() + cost;
          new_node->cost_ = base::Cost(new_cost);
          new_node->path_ = resulting_path;
          new_node->parent_ = from_node;

          return new_node;
        } else {
          return nullptr;
        }

      }

      Node * get_nearest_node(Node * rnd)
      {
        Node * nearest_node = nn_->nearest(rnd);
        return nearest_node;
      }

      bool check_collision(Node * new_node)
      {
        for (auto i : new_node->path_) {
          if (!si_->isValid(i)) {
            return false;
          }
        }
        return true;
      }

      std::tuple<double, double, double> calc_distance_and_angle(Node * from_node, Node * to_node)
      {
        const auto * from_node_se2 =
          from_node->state_->as<base::ElevationStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(
          0);
        const auto * from_node_z =
          from_node->state_->as<base::ElevationStateSpace::StateType>()->as<base::RealVectorStateSpace::StateType>(
          1);
        const auto * to_node_se2 =
          to_node->state_->as<base::ElevationStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(
          0);
        const auto * to_node_z =
          to_node->state_->as<base::ElevationStateSpace::StateType>()->as<base::RealVectorStateSpace::StateType>(
          1);
        double dx = to_node_se2->getX() - from_node_se2->getX();
        double dy = to_node_se2->getY() - from_node_se2->getY();
        double dz = to_node_z->values[0] - from_node_z->values[0];
        double d = std::hypot(dx, dy, dz);
        double theta = std::atan2(dy, dx);
        double beta = std::atan2(dz, std::hypot(dx, dy));

        return std::make_tuple(d, theta, beta);
      }

      std::vector<Node *> find_near_nodes(Node * new_node)
      {
        std::vector<Node *> near_nodes;
        auto nnode = nn_->size() + 1;
        double r = connect_circle_dist_ * std::sqrt((std::log(nnode) / nnode));
        r = std::min(r, expand_dis_);
        nn_->nearestR(new_node, r * r, near_nodes);
        return near_nodes;
      }

      double calc_new_cost(Node * from_node, double relative_cost)
      {
        double new_cost = from_node->cost_.value() + relative_cost;
        return new_cost;
      }


      Node * choose_parent(Node * new_node, std::vector<Node *> near_nodes)
      {
        if (!near_nodes.size()) {
          return nullptr;
        }
        std::vector<double> costs;
        for (auto near_node : near_nodes) {
          double relative_cost = 0.0;
          Node * t_node = steer(near_node, new_node, &relative_cost);
          if (t_node && check_collision(t_node)) {
            costs.push_back(calc_new_cost(near_node, relative_cost));
          } else {
            costs.push_back(INFINITY);
          }
        }
        double min_cost = *std::min_element(costs.begin(), costs.end());
        int min_cost_index = std::min_element(costs.begin(), costs.end()) - costs.begin();

        if (min_cost == INFINITY) {
          return nullptr;
        }
        double relative_cost = 0.0;
        new_node = steer(near_nodes[min_cost_index], new_node, &relative_cost);
        new_node->cost_ = base::Cost(min_cost);
        return new_node;
      }

      void rewire(Node * new_node, std::vector<Node *> near_nodes)
      {
        lqr_planner_->set_max_time(50.0);

        for (auto near_node : near_nodes) {
          double relative_cost = 0.0;
          Node * edge_node = steer(new_node, near_node, &relative_cost);

          if (!edge_node) {
            continue;
          }

          edge_node->cost_ = base::Cost(calc_new_cost(new_node, relative_cost));
          bool no_collision = check_collision(edge_node);
          bool improved_cost = near_node->cost_.value() > edge_node->cost_.value();

          if (no_collision && improved_cost) {

            si_->copyState(near_node->state_, edge_node->state_);

            near_node->cost_ = edge_node->cost_;
            near_node->path_ = edge_node->path_;
            near_node->parent_ = edge_node->parent_;
            propagate_cost_to_leaves(new_node);
          }
        }
        lqr_planner_->set_max_time(2.0);

      }

      void propagate_cost_to_leaves(Node * parent_node)
      {
        if (nn_) {
          std::vector<Node *> nodes;
          nn_->list(nodes);
          for (auto & node : nodes) {
            if (!node->parent_ || !node) {
              break;
            }
            if (node == parent_node) {
              node->cost_ = base::Cost(relative_path_cost(parent_node, node));
              propagate_cost_to_leaves(node);
            }
          }
        }
      }

      double relative_path_cost(Node * from_node, Node * to_node)
      {
        std::vector<double> clen;
        if (to_node->path_.size() > 2) {
          for (int i = 1; i < to_node->path_.size(); i++) {
            double this_segment_dist = distanceFunction(to_node->path_[i], to_node->path_[i - 1]);
            clen.push_back(this_segment_dist);
          }
        }
        double cost = std::accumulate(clen.begin(), clen.end(), 0.0);
        auto new_cost = from_node->cost_.value() + cost;
        return new_cost;
      }

      Node * search_best_goal_node(Node * goal_node)
      {
        std::vector<Node *> near_nodes;
        nn_->nearestR(goal_node, expand_dis_, near_nodes);
        base::Cost bestCost = opt_->infiniteCost();
        Node * selected = nullptr;

        for (auto & i : near_nodes) {
          if (opt_->isCostBetterThan(i->cost_, bestCost)) {
            selected = i;
            bestCost = i->cost_;
          }
        }
        return selected;
      }

      double distanceFunction(const Node * a, const Node * b) const
      {
        return si_->distance(a->state_, b->state_);
      }

      double distanceFunction(const base::State * a, const base::State * b) const
      {
        return si_->distance(a, b);
      }

      void fine_tune_final_course(Node * goal_node)
      {
        lqr_planner_->set_max_time(20.0);
        std::vector<Node *> path_nodes;
        path_nodes.push_back(goal_node);

        auto node = goal_node;
        while (node->parent_) {
          path_nodes.push_back(node);
          node = node->parent_;
        }
        if (path_nodes.size() < 4) {
          return;
        }
        std::reverse(path_nodes.begin(), path_nodes.end());

        for (int i = 2; i < path_nodes.size(); i += 2) {
          if (i >= path_nodes.size()) {
            break;
          }

          auto prev_node = path_nodes[i - 2];
          auto cur_node = path_nodes[i];
          double relative_cost = 0.0;
          Node * new_node = steer(prev_node, cur_node, &relative_cost);
          if (new_node) {
            new_node->cost_ = base::Cost(calc_new_cost(prev_node, relative_cost));
            bool no_collision = check_collision(new_node);
            bool improved_cost = cur_node->cost_.value() > new_node->cost_.value();
            if (no_collision && improved_cost) {
              si_->copyState(cur_node->state_, new_node->state_);
              cur_node->cost_ = new_node->cost_;
              cur_node->path_ = new_node->path_;
              cur_node->parent_ = new_node->parent_;
              propagate_cost_to_leaves(prev_node);
            }
          }
        }
        lqr_planner_->set_max_time(4.0);
      }

      void smooth_final_course(Node * goal_node, int segment_framing)
      {
        lqr_planner_->set_max_time(50.0);
        lqr_planner_->set_goal_tolerance(0.8);

        std::vector<Node *> path_nodes;
        path_nodes.push_back(goal_node);

        auto node = goal_node;
        while (node->parent_) {
          path_nodes.push_back(node);
          node = node->parent_;
        }
        if (path_nodes.size() < segment_framing) {
          return;
        }
        std::reverse(path_nodes.begin(), path_nodes.end());

        for (int i = segment_framing; i < path_nodes.size(); i += segment_framing) {

          if (i >= path_nodes.size()) {
            break;
          }

          auto prev_node = path_nodes[i - segment_framing];
          auto cur_node = path_nodes[i];
          double relative_cost = 0.0;
          Node * new_node = steer(prev_node, cur_node, &relative_cost);
          if (new_node) {
            new_node->cost_ = base::Cost(calc_new_cost(prev_node, relative_cost));
            bool no_collision = check_collision(new_node);
            bool improved_cost = cur_node->cost_.value() > new_node->cost_.value();
            if (no_collision && improved_cost) {
              si_->copyState(cur_node->state_, new_node->state_);
              cur_node->cost_ = new_node->cost_;
              cur_node->path_ = new_node->path_;
              cur_node->parent_ = new_node->parent_;
              // propogate the cost along the rest of the path
              for (int j = i - segment_framing; j < path_nodes.size(); j++) {
                auto node = path_nodes[j];
                if (!node->parent_ || !node) {
                  break;
                }
                node->cost_ = base::Cost(relative_path_cost(node->parent_, node));
              }
            }
          }
        }
        lqr_planner_->set_max_time(4.0);
      }

      std::vector<base::State *> generate_final_course(Node * goal_node)
      {
        std::vector<base::State *> final_path;
        final_path.push_back(goal_node->state_);
        auto node = goal_node;

        while (node->parent_) {
          for (auto i : node->path_) {
            final_path.push_back(i);
          }
          node = node->parent_;
        }

        final_path.push_back(node->state_);
        return final_path;
      }

      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      /** \brief State sampler */
      base::StateSamplerPtr sampler_;

      const SpaceInformation * siC_;

      /** \brief State sampler */
      base::ValidStateSamplerPtr valid_state_sampler_;

      /** \brief A nearest-neighbors datastructure containing the tree of motions */
      std::shared_ptr<NearestNeighbors<Node *>> nn_;

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_;

      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;

      rclcpp::Node::SharedPtr node_;

      std::shared_ptr<LQRPlanner> lqr_planner_;

      double goalBias_{0.05};
      double connect_circle_dist_{10.0};
      double expand_dis_{2.5};
      double goal_tolerance_{0.5};

      /** \brief The random number generator */
      RNG rng_;


    };
  }   // namespace control
}  // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__LQRRRTSTAR_HPP_
