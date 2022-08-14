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

#ifndef VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_
#define VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_planning/rrt/LQRPlanner.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "nav_msgs/msg/path.hpp"

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

    class RRTStarF : public base::Planner
    {
    public:
      /** \brief Constructor */
      RRTStarF(const SpaceInformationPtr & si);

      ~RRTStarF() override;

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

      Node * steer(Node * from_node, Node * to_node, float extend_length = INFINITY)
      {
        auto * new_node = new Node(siC_);
        new_node->state_ = si_->allocState();
        si_->copyState(new_node->state_, from_node->state_);

        auto d_theta_beta = calc_distance_and_angle(from_node, to_node);
        auto d = std::get<0>(d_theta_beta);
        auto theta = std::get<1>(d_theta_beta);
        auto beta = std::get<2>(d_theta_beta);

        new_node->path_.push_back(new_node->state_);

        if (extend_length > d) {
          extend_length = d;
        }

        int n_expand = static_cast<int>(std::floor(extend_length / path_resolution_));

        for (int i = 0; i < n_expand; i++) {
          auto * new_node_cstate =
            new_node->state_->as<ompl::base::ElevationStateSpace::StateType>();
          double x = new_node_cstate->getSE2()->getX() + path_resolution_ * std::cos(theta);
          double y = new_node_cstate->getSE2()->getY() + path_resolution_ * std::sin(theta);
          double z = new_node_cstate->getZ()->values[0] + path_resolution_ * std::sin(beta);
          new_node_cstate->setSE2(x, y, 0);
          new_node_cstate->setZ(z);
          new_node->path_.push_back(new_node->state_);
        }

        d_theta_beta = calc_distance_and_angle(new_node, to_node);
        d = std::get<0>(d_theta_beta);
        theta = std::get<1>(d_theta_beta);
        beta = std::get<2>(d_theta_beta);

        if (d <= path_resolution_) {
          auto * new_node_cstate =
            new_node->state_->as<ompl::base::ElevationStateSpace::StateType>();
          auto * to_node_cstate =
            to_node->state_->as<ompl::base::ElevationStateSpace::StateType>();

          auto * to_node_se2 = to_node_cstate->as<ompl::base::SE2StateSpace::StateType>(0);

          new_node->path_.push_back(to_node->state_);
          new_node_cstate->setSE2(to_node_se2->getX(), to_node_se2->getY(), 0);
          new_node_cstate->setZ(to_node_cstate->getZ()->values[0]);
        }

        new_node->parent_ = from_node;
        return new_node;

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

      std::vector<Node *> find_near_nodes(Node * new_node)
      {
        std::vector<Node *> near_nodes;
        auto nnode = nn_->size() + 1;
        double r = connect_circle_dist_ * std::sqrt((std::log(nnode) / nnode));
        r = std::min(r, expand_dis_);
        nn_->nearestR(new_node, r * r, near_nodes);
        return near_nodes;
      }

      double calc_new_cost(Node * from_node, Node * to_node)
      {
        base::Cost relative_cost = opt_->motionCost(from_node->state_, to_node->state_);
        return opt_->combineCosts(from_node->cost_, relative_cost).value();
      }

      Node * choose_parent(Node * new_node, std::vector<Node *> near_nodes)
      {
        if (!near_nodes.size()) {
          return nullptr;
        }
        std::vector<double> costs;
        for (auto near_node : near_nodes) {
          Node * t_node = steer(near_node, new_node);
          if (t_node && check_collision(t_node)) {
            costs.push_back(calc_new_cost(near_node, new_node));
          } else {
            costs.push_back(INFINITY);
          }
        }
        double min_cost = *std::min_element(costs.begin(), costs.end());
        int min_cost_index = std::min_element(costs.begin(), costs.end()) - costs.begin();

        if (min_cost == INFINITY) {
          std::cerr << "There is no good path.(min_cost is inf)" << std::endl;
          return nullptr;
        }
        new_node = steer(near_nodes[min_cost_index], new_node);
        new_node->cost_ = base::Cost(min_cost);
        return new_node;
      }

      void rewire(Node * new_node, std::vector<Node *> near_nodes)
      {
        for (auto near_node : near_nodes) {
          Node * edge_node = steer(new_node, near_node);
          if (!edge_node) {
            continue;
          }

          edge_node->cost_ = base::Cost(calc_new_cost(new_node, near_node));
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

      }

      void propagate_cost_to_leaves(Node * parent_node)
      {
        if (nn_) {
          std::vector<Node *> nodes;
          nn_->list(nodes);
          for (auto & node : nodes) {
            if (node == parent_node) {
              node->cost_ = base::Cost(calc_new_cost(parent_node, node));
              propagate_cost_to_leaves(node);
            }
          }
        }
      }

      double calc_dist_to_goal(Node * node, ompl::base::Goal * goal)
      {
        double dist = 0.0;
        bool solv = goal->isSatisfied(node->state_, &dist);
        return dist;
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
      std::vector<base::State *> lqrize(std::vector<base::State *> path_nodes, int segment_framing)
      {
        lqr_planner_->set_max_time(4.0);
        lqr_planner_->set_goal_tolerance(0.5);
        lqr_planner_->set_phi_bound(0.8);
        lqr_planner_->set_dt(0.2);
        lqr_planner_->set_v(1.0);

        std::vector<base::State *> lqr_path_states;

        if (path_nodes.size() < segment_framing) {
          return lqr_path_states;
        }
        std::reverse(path_nodes.begin(), path_nodes.end());

        for (int i = segment_framing; i < path_nodes.size(); i += segment_framing) {

          if (i >= path_nodes.size()) {
            i = path_nodes.size() - 1;
          }

          auto prev_node = path_nodes[i - segment_framing];
          auto cur_node = path_nodes[i];
          if (i != segment_framing) {
            prev_node = lqr_path_states.back();
          }

          double relative_cost = 0.0;

          std::vector<base::State *> resulting_path;

          if (i == segment_framing) {
            lqr_planner_->compute_LQR_plan(
              prev_node, cur_node, resulting_path);
          } else {
            auto * prev_node_cstate =
              prev_node->as<ompl::base::ElevationStateSpace::StateType>();
            auto * latest_arrived_cstate =
              lqr_path_states.back()->as<ompl::base::ElevationStateSpace::StateType>();
            prev_node_cstate->setYaw(latest_arrived_cstate->getSE2()->getYaw());
            lqr_planner_->compute_LQR_plan(
              prev_node, cur_node, resulting_path);
          }

          for (auto seg : resulting_path) {
            lqr_path_states.push_back(seg);

          }
        }
        lqr_planner_->set_max_time(4.0);

        return lqr_path_states;
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

      std::vector<base::State *> remove_duplicate_states(std::vector<base::State *> all_states)
      {

        multiCont : /* Empty statement using the semicolon */;

        for (int i = 0; i < all_states.size(); i++) {
          for (int j = 0; j < all_states.size(); j++) {
            if ( (i != j) && (distanceFunction(all_states[i], all_states[j]) < 0.05)) {
              // j is duplicate
              all_states.erase(all_states.begin() + j);
              goto multiCont;
            }
          }
        }

        std::vector<base::State *> sorted;
        sorted.push_back(all_states.front());
        all_states.erase(all_states.begin());

        sortCont : /* Empty statement using the semicolon */;

        for (int i = 0; i < all_states.size(); i++) {
          int closest_idx = 100000;
          double currnet_min = 100000.0;
          all_states[i] = sorted.back();

          for (int j = 0; j < all_states.size(); j++) {
            double dist = distanceFunction(all_states[i], all_states[j]);
            if (dist < currnet_min && ( i != j)) {
              currnet_min = dist;
              closest_idx = j;
            }
          }

          if (closest_idx > all_states.size() - 1) {
            sorted.push_back(all_states.back());
            break;
          }

          sorted.push_back(all_states[closest_idx]);
          all_states.erase(all_states.begin() + closest_idx);
          goto sortCont;
        }

        return sorted;
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

      double goalBias_{0.05};
      double path_resolution_{0.025};
      double connect_circle_dist_{100.0};
      double expand_dis_{1.5};

      std::shared_ptr<LQRPlanner> lqr_planner_;

      /** \brief The random number generator */
      RNG rng_;


    };
  }
}


#endif  // VOX_NAV_PLANNING__RRT__RRTSTARF_HPP_
