#include "vox_nav_planning/rrt/RRTStarF.hpp"


ompl::control::RRTStarF::RRTStarF(const SpaceInformationPtr & si)
: base::Planner(si, "RRTStarF")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
}

ompl::control::RRTStarF::~RRTStarF()
{
  freeMemory();
}

void ompl::control::RRTStarF::setup()
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

void ompl::control::RRTStarF::clear()
{
  Planner::clear();
  sampler_.reset();
  valid_state_sampler_.reset();
  freeMemory();
  if (nn_) {
    nn_->clear();
  }
}

void ompl::control::RRTStarF::freeMemory()
{
  if (nn_) {
    std::vector<Node *> nodes;
    nn_->list(nodes);
    for (auto & node : nodes) {
      if (node->state_) {
        si_->freeState(node->state_);
      }
      delete node;
    }
  }
}

ompl::base::PlannerStatus ompl::control::RRTStarF::solve(
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
    auto new_node = steer(nearest_node, random_node, expand_dis_);
    auto inc_cost = opt_->motionCost(nearest_node->state_, new_node->state_);
    new_node->cost_ = opt_->combineCosts(nearest_node->cost_, inc_cost);

    if (check_collision(new_node)) {
      std::vector<Node *> near_nodes = find_near_nodes(new_node);
      auto node_with_updated_parent = choose_parent(new_node, near_nodes);
      if (node_with_updated_parent != nullptr) {
        rewire(node_with_updated_parent, near_nodes);
        nn_->add(node_with_updated_parent);
      } else {
        nn_->add(new_node);
      }
      last_node = search_best_goal_node(goal_node);
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

  bool solved = false;
  bool approximate = false;

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
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.text = std::to_string(node_index_counter);
      marker.scale.x = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.colors.push_back(marker.color);
      const auto * cstate = i->state_->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * se2 = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
      const auto * z = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      geometry_msgs::msg::Point node_point, parent_point;
      node_point.x = se2->getX();
      node_point.y = se2->getY();
      node_point.z = z->values[0];
      const auto * parent_cstate =
        i->parent_->state_->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * parent_se2 = parent_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
      const auto * parent_z = parent_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      parent_point.x = parent_se2->getX();
      parent_point.y = parent_se2->getY();
      parent_point.z = parent_z->values[0];
      marker.points.push_back(parent_point);
      marker.points.push_back(node_point);
      rrt_nodes.markers.push_back(marker);
      node_index_counter++;
    }
  }
  rrt_nodes_pub_->publish(rrt_nodes);

  if (last_valid_node) {
    std::vector<base::State *> final_course = generate_final_course(last_valid_node);
    solved = true;

    /* set the solution path */
    auto path(std::make_shared<PathControl>(si_));
    for (auto i : final_course) {
      if (i) {
        path->append(i);
      }
    }
    solved = true;
    pdef_->addSolutionPath(path, approximate, 0.0 /*approxdif*/, getName());
    OMPL_INFORM("Found solution with cost %.2f", last_valid_node->cost_.value());
  } else {
    OMPL_WARN("%s: Failed to cretae a plan", getName().c_str());
  }

  clear();

  return {solved, approximate};
}

void ompl::control::RRTStarF::getPlannerData(base::PlannerData & data) const
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
