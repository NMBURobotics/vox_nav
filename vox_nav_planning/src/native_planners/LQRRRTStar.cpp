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

#include "vox_nav_planning/native_planners/LQRRRTStar.hpp"


ompl::control::LQRRRTStar::LQRRRTStar(const SpaceInformationPtr & si)
: base::Planner(si, "LQRRRTStar")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
  lqr_planner_ = std::make_shared<LQRPlanner>(si);
}

ompl::control::LQRRRTStar::~LQRRRTStar()
{
  freeMemory();
}

void ompl::control::LQRRRTStar::setup()
{
  base::Planner::setup();
  if (!nn_) {
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Node *>(this));
  }
  nn_->setDistanceFunction(
    [this](const Node * a, const Node * b)
    {
      return distanceFunction(a, b);
    });

  if (pdef_) {
    if (pdef_->hasOptimizationObjective()) {
      opt_ = pdef_->getOptimizationObjective();
    } else {
      OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
      opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
      pdef_->setOptimizationObjective(opt_);
    }
  }

  // ros2 node to publish rrt nodes
  node_ = std::make_shared<rclcpp::Node>("rrtstarf_rclcpp_node");

  rrt_nodes_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/rrtstar/nodes", rclcpp::SystemDefaultsQoS());

}

void ompl::control::LQRRRTStar::clear()
{
  Planner::clear();
  sampler_.reset();
  valid_state_sampler_.reset();
  freeMemory();
  if (nn_) {
    nn_->clear();
  }
}

void ompl::control::LQRRRTStar::freeMemory()
{

}


ompl::base::PlannerStatus ompl::control::LQRRRTStar::solve(
  const base::PlannerTerminationCondition & ptc)
{
  checkValidity();
  base::Goal * goal = pdef_->getGoal().get();
  auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

  // get the goal node and state
  auto * goal_node = new Node(siC_);
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_node->state_, goal);
  }

  // get start node and state,push  the node inton nn_ as well
  auto * start_node = new Node(siC_);
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_node->state_, st);
    nn_->add(start_node);
  }

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  // Use valid state sampler
  if (!valid_state_sampler_) {
    valid_state_sampler_ = si_->allocValidStateSampler();
  }
  if (!sampler_) {
    sampler_ = si_->allocStateSampler();
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n",
    getName().c_str(), nn_->size());

  auto * random_node = new Node(siC_);
  base::State * random_state = random_node->state_;

  unsigned iterations = 0;

  Node * last_node = new Node(siC_);
  Node * last_valid_node = new Node(siC_);
  last_valid_node->cost_ = base::Cost(std::numeric_limits<double>::max());

  while (ptc == false) {
    /* sample random state (with goal biasing) */
    if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
      goal_s->sampleGoal(random_state);
    } else {
      // sampler_->sampleUniform(random_state);
      valid_state_sampler_->sample(random_state);
    }

    auto nearest_node = get_nearest_node(random_node);

    double relative_cost = 0.0;
    auto new_node = steer(nearest_node, random_node, &relative_cost);
    if (!new_node) {continue;}

    if (check_collision(new_node)) {
      std::vector<Node *> near_nodes = find_near_nodes(new_node);
      auto node_with_updated_parent = choose_parent(new_node, near_nodes);
      if (node_with_updated_parent != nullptr) {
        nn_->add(node_with_updated_parent);
        last_node = search_best_goal_node(goal_node);
      }
    }

    if (last_node != nullptr && last_node->cost_.value() < last_valid_node->cost_.value()) {
      last_valid_node = last_node;
    }
    iterations++;

    if (iterations % 200 == 0 && last_valid_node &&
      static_cast<int>(last_valid_node->cost_.value()))
    {
      OMPL_INFORM("Current solution cost %.2f", last_valid_node->cost_.value());
    }

  }

  double relative_cost = 0.0;
  lqr_planner_->set_max_time(10.0);
  auto final_node = steer(last_valid_node, goal_node, &relative_cost);
  lqr_planner_->set_max_time(4.0);

  if (final_node) {
    if (check_collision(final_node)) {
      std::vector<Node *> near_nodes = find_near_nodes(final_node);
      auto final_node_parent = choose_parent(final_node, near_nodes);
      if (final_node_parent != nullptr) {
        nn_->add(final_node_parent);
      }
    }
  }

  OMPL_INFORM(
    "%s: Created %u states in %u iterations", getName().c_str(), nn_->size(),
    iterations);

  // visualize rrt node tree growth in RVIZ
  std::vector<Node *> all_nodes;
  nn_->list(all_nodes);
  visualization_msgs::msg::MarkerArray rrt_nodes;
  int node_index_counter = 0;
  for (auto i : all_nodes) {
    if (i) {
      if (!i->parent_) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "rrt_nodes";
      marker.id = node_index_counter;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.text = std::to_string(node_index_counter);
      marker.scale.x = 0.1;
      marker.scale.y = 0.25;
      marker.scale.z = 0.25;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      const auto * cstate = i->state_->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      geometry_msgs::msg::Point node_point, parent_point;
      node_point.x = xyzv->values[0];
      node_point.y = xyzv->values[1];
      node_point.z = xyzv->values[2];
      const auto * parent_cstate =
        i->parent_->state_->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * parent_so2 = parent_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * parent_xyzv = parent_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      parent_point.x = parent_xyzv->values[0];
      parent_point.y = parent_xyzv->values[1];
      parent_point.z = parent_xyzv->values[2];

      marker.points.push_back(parent_point);
      marker.colors.push_back(marker.color);
      for (auto && seg : i->path_) {
        const auto * cstate = seg->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto * xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        geometry_msgs::msg::Point seg_point;
        seg_point.x = xyzv->values[0];
        seg_point.y = xyzv->values[1];
        seg_point.z = xyzv->values[2];
        marker.points.push_back(seg_point);
        marker.colors.push_back(marker.color);
      }
      marker.points.push_back(node_point);
      marker.colors.push_back(marker.color);
      rrt_nodes.markers.push_back(marker);
      node_index_counter++;
    }
  }
  rrt_nodes_pub_->publish(rrt_nodes);

  bool solved = false;
  bool approximate = false;
  double approxdif = std::numeric_limits<double>::infinity();

  if (final_node) {

    OMPL_INFORM("Final solution cost %.2f", final_node->cost_.value());

    std::vector<base::State *> final_course = generate_final_course(last_valid_node);

    final_course = remove_duplicate_states(final_course);
    double dist = 0.0;
    bool solv = goal->isSatisfied(final_course.front(), &dist);

    /* set the solution path */
    auto path(std::make_shared<PathControl>(si_));
    for (auto i : final_course) {
      if (i) {
        path->append(i);
      }
    }
    OMPL_INFORM("Solution Lenght %.2f", path->length());
    OMPL_INFORM("Distance of solution to goal %.2f", dist);

    approxdif = dist;

    if (solv) {
      solved = true;
      approximate = false;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_INFORM("Found solution with cost %.2f", last_valid_node->cost_.value());
    } else if (!solv && (path->length() > 1.0 )) { // approx
      solved = true;
      approximate = true;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_INFORM(
        "%s: Approx solution with cost %.2f",
        getName().c_str(), last_valid_node->cost_.value());
    } else {
      solved = false;
      approximate = false;
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      OMPL_WARN("%s: Failed to cretae a plan", getName().c_str());
    }
  }

  clear();

  return {solved, approximate};
}

void ompl::control::LQRRRTStar::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);

  std::vector<Node *> Nodes;
  std::vector<Node *> allNodes;
  if (nn_) {
    nn_->list(Nodes);
  }

  for (unsigned i = 0; i < allNodes.size(); i++) {
    if (allNodes[i]->parent_ != nullptr) {
      allNodes.push_back(allNodes[i]->parent_);
    }
  }

  double delta = siC_->getPropagationStepSize();

  for (auto m : allNodes) {
    if (m->parent_) {
      data.addEdge(
        base::PlannerDataVertex(m->parent_->state_),
        base::PlannerDataVertex(m->state_));
    } else {
      data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
  }
}
