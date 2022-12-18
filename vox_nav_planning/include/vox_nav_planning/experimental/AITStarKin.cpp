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

#include "AITStarKin.hpp"

ompl::control::AITStarKin::AITStarKin(const SpaceInformationPtr & si)
: base::Planner(si, "AITStarKin")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
}

ompl::control::AITStarKin::~AITStarKin()
{
  freeMemory();
}

void ompl::control::AITStarKin::setup()
{
  base::Planner::setup();
  if (!nn_) {
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
  }
  nn_->setDistanceFunction(
    [this](const VertexProperty * a, const VertexProperty * b)
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

  // Use valid state sampler
  if (!valid_state_sampler_) {
    valid_state_sampler_ = si_->allocValidStateSampler();
  }
  if (!sampler_) {
    sampler_ = si_->allocStateSampler();
  }

  // ros2 node to publish rrt nodes
  node_ = std::make_shared<rclcpp::Node>("aitstarkin_rclcpp_node");

  rrt_nodes_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/rrtstar/nodes", rclcpp::SystemDefaultsQoS());

}

void ompl::control::AITStarKin::clear()
{
  Planner::clear();
  sampler_.reset();
  valid_state_sampler_.reset();
  freeMemory();
  if (nn_) {
    nn_->clear();
  }
}

void ompl::control::AITStarKin::freeMemory()
{
  if (nn_) {
    std::vector<VertexProperty *> nodes;
    nn_->list(nodes);
    for (auto & node : nodes) {
      if (node->state) {
        si_->freeState(node->state);
      }
      delete node;
    }
  }

  //delete LPAstarApx_;
  //delete LPAstarLb_;
}

ompl::base::PlannerStatus ompl::control::AITStarKin::solve(
  const base::PlannerTerminationCondition & ptc)
{

  checkValidity();
  base::Goal * goal = pdef_->getGoal().get();
  auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

  // get the goal node and state
  auto * goal_state = si_->allocState();
  VertexProperty * goal_vertex = new VertexProperty();
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }
  goal_vertex->state = goal_state;
  nn_->add(goal_vertex);

  // get start node and state,push  the node inton nn_ as well
  auto * start_state = si_->allocState();
  VertexProperty * start_vertex = new VertexProperty();
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }
  start_vertex->state = start_state;
  nn_->add(start_vertex);

  // Add goal and start to graph
  vertex_descriptor start_vertex_descriptor = boost::add_vertex(g_);
  vertex_descriptor goal_vertex_descriptor = boost::add_vertex(g_);
  g_[start_vertex_descriptor].state = start_state;
  g_[start_vertex_descriptor].state_label = reinterpret_cast<std::uintptr_t>(start_state);
  g_[start_vertex_descriptor].id = start_vertex_descriptor;
  g_[goal_vertex_descriptor].state = goal_state;
  g_[goal_vertex_descriptor].state_label = reinterpret_cast<std::uintptr_t>(goal_state);
  g_[goal_vertex_descriptor].id = goal_vertex_descriptor;
  start_vertex_ = g_[start_vertex_descriptor];
  goal_vertex_ = g_[goal_vertex_descriptor];

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n", getName().c_str(),
    nn_->size());

  std::vector<ompl::base::State *> samples;
  generateBatchofSamples(batch_size_, true, samples);

  for (auto && i : samples) {
    vertex_descriptor this_vertex_descriptor = boost::add_vertex(g_);
    g_[this_vertex_descriptor].state = (i);
    g_[this_vertex_descriptor].state_label = (reinterpret_cast<std::uintptr_t>(i));
    g_[this_vertex_descriptor].id = (this_vertex_descriptor);
    VertexProperty * this_vertex_property = new VertexProperty(g_[this_vertex_descriptor]);
    nn_->add(this_vertex_property);
  }

  CostEstimatorLb costEstimatorLb(goal, this);
  LPAstarLb_ = new LPAstarLb(goal_vertex_.id, start_vertex_.id, graphLb_, costEstimatorLb);    // rooted at source
  CostEstimatorApx costEstimatorApx(this);
  LPAstarApx_ = new LPAstarApx(start_vertex_.id, goal_vertex_.id, graphApx_, costEstimatorApx);    // rooted at target

  for (auto vd : boost::make_iterator_range(vertices(g_))) {
    std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
    nn_->nearestR(&g_[vd], radius_, nbh);
    for (auto && nb : nbh) {
      vertex_descriptor u = g_[vd].id;
      vertex_descriptor v = nb->id;
      double dist = distanceFunction(g_[u].state, g_[v].state);
      edge_descriptor e; bool edge_added;
      if (u == v || dist > radius_) {
        continue;
      }
      boost::tie(e, edge_added) = boost::add_edge(u, v, WeightProperty(dist), g_);
      addEdgeApx(&g_[vd], nb, dist);
      addEdgeLb(&g_[vd], nb, dist);
    }
  }

  std::list<std::size_t> pathApx, pathLb;
  double costApx = LPAstarApx_->computeShortestPath(pathApx);
  double costLb = LPAstarLb_->computeShortestPath(pathLb);

  std::cout << costApx << std::endl;
  std::cout << costLb << std::endl;

  visulizeRGG(g_);

  clear();

  return {};
}

void ompl::control::AITStarKin::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);
  std::vector<VertexProperty *> Nodes;
  std::vector<VertexProperty *> allNodes;
  if (nn_) {
    nn_->list(Nodes);
  }
  double delta = siC_->getPropagationStepSize();
}

void ompl::control::AITStarKin::generateBatchofSamples(
  int batch_size,
  bool use_valid_sampler,
  std::vector<ompl::base::State *> & samples)
{
  samples.reserve(batch_size);
  do{
    // Create a new vertex.
    auto state = si_->allocState();
    samples.push_back(state);
    if (use_valid_sampler) {
      valid_state_sampler_->sample(samples.back());
    } else {
      do{
        // Sample the associated state uniformly within the informed set.
        sampler_->sampleUniform(samples.back());
        // Count how many states we've checked.
      } while (!si_->getStateValidityChecker()->isValid(samples.back()));
    }
  } while (samples.size() < batch_size);
}

void ompl::control::AITStarKin::visulizeRGG(const GraphT & g)
{
  visualization_msgs::msg::MarkerArray marker_array;
  rrt_nodes_pub_->publish(marker_array);

  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap
  for (auto vd : boost::make_iterator_range(vertices(g))) {
    double lb_estimate = (*(LPAstarApx_))(g[vd].id);  // cost from goal to start
    double apx_estimate = (*(LPAstarLb_))(g[vd].id);  // cost from start to goal
    std::stringstream ss_lb, ss_apx;
    ss_lb << std::setprecision(2) << lb_estimate;
    ss_apx << std::setprecision(2) << apx_estimate;

    const auto * target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_se2 = target_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * target_z = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    geometry_msgs::msg::Point point;
    point.x = target_se2->getX();
    point.y = target_se2->getY();
    point.z = target_z->values[0];
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map";
    sphere.header.stamp = rclcpp::Clock().now();
    sphere.ns = "rgg_vertex";
    sphere.id = g[vd].id;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.text = std::to_string(g[vd].id);
    sphere.pose.position = point;
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;
    sphere.color.a = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;
    marker_array.markers.push_back(sphere);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = "rgg_costs";
    text.id = g[vd].id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0);
    text.text = ss_apx.str(); // + "/" + ss_lb.str();
    text.pose = sphere.pose;
    text.pose.position.z += 0.5;
    text.scale.x = 0.3;
    text.scale.y = 0.3;
    text.scale.z = 0.3;
    text.color.a = 1.0;
    text.color.r = 1.0;
    marker_array.markers.push_back(text);
  }

  auto es = boost::edges(g);
  vertex_descriptor u, v;
  int edge_index = 0;
  for (auto eit = es.first; eit != es.second; ++eit) {

    u = boost::source(*eit, g);
    v = boost::target(*eit, g);
    geometry_msgs::msg::Point source_point, target_point;

    const auto * source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * source_se2 = source_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * source_z = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    source_point.x = source_se2->getX();
    source_point.y = source_se2->getY();
    source_point.z = source_z->values[0];

    const auto * target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_se2 = target_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * target_z = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    target_point.x = target_se2->getX();
    target_point.y = target_se2->getY();
    target_point.z = target_z->values[0];

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.ns = "rgg_edges";
    line_strip.id = edge_index;
    line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.lifetime = rclcpp::Duration::from_seconds(0);
    line_strip.header.stamp = rclcpp::Clock().now();
    line_strip.scale.x = 0.05;
    line_strip.scale.y = 0.05;
    line_strip.scale.z = 0.05;
    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.b = 1.0;
    std_msgs::msg::ColorRGBA blue_color;
    blue_color.b = 1.0;
    blue_color.a = 1.0;
    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(blue_color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(blue_color);

    marker_array.markers.push_back(line_strip);
    edge_index++;
  }

  rrt_nodes_pub_->publish(marker_array);
}
