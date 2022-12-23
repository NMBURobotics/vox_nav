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

  if (!valid_state_sampler_) {
    valid_state_sampler_ = si_->allocValidStateSampler();
  }

  if (!sampler_) {
    sampler_ = si_->allocStateSampler();
  }

  if (!informed_sampler_) {
    informed_sampler_ = std::make_shared<base::PathLengthDirectInfSampler>(
      pdef_,
      std::numeric_limits<double>::infinity());
  }

  if (!controlSampler_) {
    controlSampler_ = std::make_shared<SimpleDirectedControlSampler>(siC_, 100);
  }

  // RVIZ VISUALIZATIONS
  node_ = std::make_shared<rclcpp::Node>("aitstarkin_rclcpp_node");
  rrt_nodes_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/rrtstar/nodes", rclcpp::SystemDefaultsQoS());

  heur_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "vox_nav/rrtstar/heur", rclcpp::SystemDefaultsQoS());

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

  bool goal_reached;
  std::list<std::size_t> forwardPath, reversePath;

  WeightMap weightmap = get(boost::edge_weight, g_);

  while (ptc == false) {

    std::vector<ompl::base::State *> samples;
    generateBatchofSamples(batch_size_, use_valid_sampler_, samples);

    // Add batch_size_ number of samples to graphs
    // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
    // But too close vertices will be discarded in order for memory not to sink
    for (auto && i : samples) {
      VertexProperty * this_vertex_property = new VertexProperty();
      this_vertex_property->state = (i);
      this_vertex_property->state_label = (reinterpret_cast<std::uintptr_t>(i));
      std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
      nn_->nearestR(this_vertex_property, radius_, nbh);
      if (nbh.size() > max_neighbors_) {
        nbh.resize(max_neighbors_);
      }

      bool does_vertice_exits{false};
      for (auto && nb : nbh) {
        double dist = distanceFunction(i, nb->state);
        if (dist < min_dist_between_vertices_ /*do not add same vertice twice*/) {
          does_vertice_exits = true;
        }
      }

      if (!does_vertice_exits) {
        vertex_descriptor this_vertex_descriptor = boost::add_vertex(g_);
        this_vertex_property->id = this_vertex_descriptor;
        g_[this_vertex_descriptor] = *this_vertex_property;
        nn_->add(this_vertex_property);
        for (auto && nb : nbh) {
          vertex_descriptor u = this_vertex_descriptor;
          vertex_descriptor v = nb->id;
          double dist = distanceFunction(g_[u].state, g_[v].state);
          edge_descriptor e; bool edge_added;
          // not to construct edges with self, and if nbh is further than radius_, continue
          if (u == v || dist > radius_) {
            continue;
          }
          if (boost::edge(u, v, g_).second || boost::edge(v, u, g_).second) {
            continue;
          }
          // Once suitable edges are found, populate them over graphs
          boost::tie(e, edge_added) = boost::add_edge(u, v, g_);
          weightmap[e] = dist;
        }
      }
    }

    std::vector<ompl::control::AITStarKin::VertexProperty *> goal_nbh;
    nn_->nearestR(goal_vertex, radius_, goal_nbh);
    if (goal_nbh.size() > max_neighbors_) {
      goal_nbh.resize(max_neighbors_);
    }
    for (auto && nb : goal_nbh) {
      vertex_descriptor u = goal_vertex_descriptor;
      vertex_descriptor v = nb->id;
      double dist = distanceFunction(g_[u].state, g_[v].state);
      edge_descriptor e; bool edge_added;
      // not to construct edges with self, and if nbh is further than radius_, continue
      if (u == v || dist > radius_) {
        continue;
      }
      if (boost::edge(u, v, g_).second || boost::edge(v, u, g_).second) {
        continue;
      }
      // Once suitable edges are found, populate them over graphs
      boost::tie(e, edge_added) = boost::add_edge(u, v, g_);
      weightmap[e] = dist;
    }

    int num_visited_nodes = 0;
    std::vector<vertex_descriptor> p(boost::num_vertices(g_));
    std::vector<Cost> d(boost::num_vertices(g_));
    std::list<vertex_descriptor> shortest_path;

    try {
      // Run A* backwards from goal to start
      auto heuristic = GenericDistanceHeuristic<GraphT, VertexProperty, Cost>(this, &start_vertex_);
      auto c_visitor = custom_goal_visitor<vertex_descriptor>(
        start_vertex_descriptor, &num_visited_nodes, this);

      boost::astar_search_tree(
        g_, goal_vertex_descriptor, heuristic,
        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));

      OMPL_INFORM(
        "%s: A* Failed to produce final collision free path after %d node visits.\n",
        getName().c_str(), num_visited_nodes);

    } catch (FoundGoal found_goal) {

      OMPL_INFORM("%s: A valid geometric path has been found.\n", getName().c_str() );
      // Found a path to the goal, catch the exception
      for (vertex_descriptor v = start_vertex_.id;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v) {break;}
      }
    }

    OMPL_INFORM(
      "%s: Advancing with %d vertices and %d edges.\n",
      getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));

    visualizeLPAHuer(shortest_path);

    visualizeRGG(g_);

  }

  if (true) {
    OMPL_INFORM("%s: Calculating Kinodynamic path.\n", getName().c_str());
    for (size_t i = 1; i < forwardPath.size(); i++) {
      auto c = siC_->allocControl();
      auto prev_state_it = *std::next(forwardPath.begin(), i - 1);
      auto curr_state_it = *std::next(forwardPath.begin(), i);
      auto steps = controlSampler_->sampleTo(c, g_[prev_state_it].state, g_[curr_state_it].state);
    }
  }

  /* set the solution path */
  auto path(std::make_shared<PathControl>(si_));
  for (auto i : forwardPath) {
    if (g_[i].state) {
      path->append(g_[i].state);
    }
  }


  pdef_->addSolutionPath(path, false, 0.0, getName());

  clear();

  return {goal_reached, false};
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
        //informed_sampler_->sampleUniform()
        // Count how many states we've checked.
      } while (!si_->getStateValidityChecker()->isValid(samples.back()));
    }
  } while (samples.size() < batch_size);
}

void ompl::control::AITStarKin::visualizeRGG(const GraphT & g)
{

  bool show_infs = true;

  // Clear All previous markers
  visualization_msgs::msg::MarkerArray clear_markers;
  visualization_msgs::msg::Marker rgg_vertex, rgg_costs, rgg_edges;
  rgg_vertex.id = 0;
  rgg_costs.id = 0;
  rgg_edges.id = 0;
  rgg_vertex.ns = "rgg_vertex";
  rgg_costs.ns = "rgg_costs";
  rgg_edges.ns = "rgg_edges";
  rgg_vertex.action = visualization_msgs::msg::Marker::DELETEALL;
  rgg_costs.action = visualization_msgs::msg::Marker::DELETEALL;
  rgg_edges.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_markers.markers.push_back(rgg_vertex);
  clear_markers.markers.push_back(rgg_costs);
  clear_markers.markers.push_back(rgg_edges);
  rrt_nodes_pub_->publish(clear_markers);


  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = "map";
  sphere.header.stamp = rclcpp::Clock().now();
  sphere.ns = "rgg_vertex";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.scale.x = 0.1;
  sphere.scale.y = 0.1;
  sphere.scale.z = 0.1;

  visualization_msgs::msg::MarkerArray marker_array;

  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap
  for (auto vd : boost::make_iterator_range(vertices(g))) {
    double apx_estimate = 0;//(*(LPAstarCost2Come_))(g[vd].id);   // cpst to come
    double lb_estimate = 0;//(*(LPAstarCost2Go_))(g[vd].id);   // cost to go
    std::stringstream ss_lb, ss_apx;
    ss_lb << std::setprecision(2) << lb_estimate;
    ss_apx << std::setprecision(2) << apx_estimate;


    if (!show_infs &&
      (lb_estimate == std::numeric_limits<double>::infinity() ||
      apx_estimate == std::numeric_limits<double>::infinity() ))
    {
      continue;
    }

    double is_goal_or_start{0.0};
    if (g[vd].id == start_vertex_.id || g[vd].id == goal_vertex_.id) {
      is_goal_or_start = 1.0;
    }

    const auto * target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

    geometry_msgs::msg::Point point;
    point.x = target_xyzv->values[0];
    point.y = target_xyzv->values[1];
    point.z = target_xyzv->values[2];
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.g = is_goal_or_start;
    color.b = 1.0;
    sphere.points.push_back(point);
    sphere.colors.push_back(color);


    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = "rgg_costs";
    text.id = g[vd].id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0);
    text.text = ss_apx.str() + "/" + ss_lb.str();
    text.pose = sphere.pose;
    text.pose.position.z += 0.5;
    text.scale.x = 0.3;
    text.scale.y = 0.3;
    text.scale.z = 0.3;
    text.color.a = 1.0;
    text.color.r = 1.0;
    //marker_array.markers.push_back(text);
  }
  marker_array.markers.push_back(sphere);

  auto es = boost::edges(g);
  vertex_descriptor u, v;
  int edge_index = 0;
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.ns = "rgg_edges";
  line_strip.id = edge_index;
  line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.lifetime = rclcpp::Duration::from_seconds(0);
  line_strip.header.stamp = rclcpp::Clock().now();
  line_strip.scale.x = 0.005;
  line_strip.scale.y = 0.005;
  line_strip.scale.z = 0.005;
  line_strip.color.a = 1.0;
  line_strip.color.r = 0.0;
  line_strip.color.b = 1.0;
  std_msgs::msg::ColorRGBA blue_color;
  blue_color.b = 1.0;
  blue_color.a = 1.0;
  for (auto eit = es.first; eit != es.second; ++eit) {

    u = boost::source(*eit, g);
    v = boost::target(*eit, g);
    double u_estimate = 0;//(*(LPAstarCost2Go_))(g[u].id);   // cpst to come
    double v_estimate = 0;//(*(LPAstarCost2Go_))(g[v].id);   // cost to go
    double apxu_estimate = 0;//(*(LPAstarCost2Come_))(g[u].id);       // cpst to come
    double apxv_estimate = 0;//(*(LPAstarCost2Come_))(g[v].id);   // cost to go

    if (!show_infs &&
      (u_estimate == std::numeric_limits<double>::infinity() ||
      v_estimate == std::numeric_limits<double>::infinity() ||
      apxu_estimate == std::numeric_limits<double>::infinity() ||
      apxv_estimate == std::numeric_limits<double>::infinity() ))
    {
      continue;
    }

    geometry_msgs::msg::Point source_point, target_point;
    const auto * source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * source_xyzv = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    source_point.x = source_xyzv->values[0];
    source_point.y = source_xyzv->values[1];
    source_point.z = source_xyzv->values[2];

    const auto * target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

    target_point.x = target_xyzv->values[0];
    target_point.y = target_xyzv->values[1];
    target_point.z = target_xyzv->values[2];
    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(blue_color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(blue_color);

  }
  marker_array.markers.push_back(line_strip);

  rrt_nodes_pub_->publish(marker_array);
}

void ompl::control::AITStarKin::visualizeLPAHuer(const std::list<vertex_descriptor> & heur)
{
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.ns = "heur";
  line_strip.id = 0;
  line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.lifetime = rclcpp::Duration::from_seconds(0);
  line_strip.header.stamp = rclcpp::Clock().now();
  line_strip.scale.x = 0.05;
  line_strip.scale.y = 0.05;
  line_strip.scale.z = 0.05;
  line_strip.color.a = 1.0;
  line_strip.color.r = 1.0;
  line_strip.color.b = 0.3;
  std_msgs::msg::ColorRGBA blue_color;
  blue_color.b = 1.0;
  blue_color.a = 1.0;

  for (size_t i = 1; i < heur.size(); i++) {
    auto u = *std::next(heur.begin(), i - 1);
    auto v = *std::next(heur.begin(), i);
    geometry_msgs::msg::Point source_point, target_point;
    const auto * source_cstate = g_[u].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * source_se2 = source_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * source_z = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    source_point.x = source_se2->getX();
    source_point.y = source_se2->getY();
    source_point.z = source_z->values[0];
    const auto * target_cstate = g_[v].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_se2 = target_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * target_z = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    target_point.x = target_se2->getX();
    target_point.y = target_se2->getY();
    target_point.z = target_z->values[0];
    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(blue_color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(blue_color);
  }
  heur_pub_->publish(line_strip);
}
