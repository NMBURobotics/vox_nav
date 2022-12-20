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

  if (!controlSampler_) {
    controlSampler_ = std::make_shared<SimpleDirectedControlSampler>(siC_, 100);
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

  //delete LPAstarCost2Come_;
  //delete LPAstarCost2Go_;
}

ompl::base::PlannerStatus ompl::control::AITStarKin::solve(
  const base::PlannerTerminationCondition & ptc)
{

  auto a1 = std::chrono::high_resolution_clock::now();

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

  Cost2GoEstimator cost2GoEstimator(goal, this);
  LPAstarCost2Go_ =
    new LPAstarCost2Go(goal_vertex_.id, start_vertex_.id, graphLb_, cost2GoEstimator);       // rooted at source

  Cost2ComeEstimator cost2ComeEstimator(this);
  LPAstarCost2Come_ =
    new LPAstarCost2Come(start_vertex_.id, goal_vertex_.id, graphApx_, cost2ComeEstimator);   // rooted at target

  bool goal_reached;
  std::list<std::size_t> forwardPath, reversePath;


  while (ptc == false) {

    std::vector<ompl::base::State *> samples;
    generateBatchofSamples(batch_size_, true, samples);

    // Add batch_size_ number of samples to graphs
    // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
    // But too close vertices will be discarded in order for memory not to sink
    for (auto && i : samples) {
      VertexProperty * this_vertex_property = new VertexProperty();
      this_vertex_property->state = (i);
      this_vertex_property->state_label = (reinterpret_cast<std::uintptr_t>(i));
      std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
      nn_->nearestR(this_vertex_property, radius_, nbh);
      bool does_vertice_exits{false};
      for (auto && nb : nbh) {
        double dist = distanceFunction(i, nb->state);
        if (dist < 0.01 /*do not add same vertice twice*/) {
          does_vertice_exits = true;
        }
      }
      if (!does_vertice_exits) {
        vertex_descriptor this_vertex_descriptor = boost::add_vertex(g_);
        this_vertex_property->id = this_vertex_descriptor;
        g_[this_vertex_descriptor] = *this_vertex_property;
        nn_->add(this_vertex_property);
      }
    }

    std::vector<vertex_descriptor> vertices_to_be_removed;
    for (auto vd : boost::make_iterator_range(vertices(g_))) {
      std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
      nn_->nearestR(&g_[vd], radius_, nbh);
      for (auto && nb : nbh) {
        vertex_descriptor u = g_[vd].id;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(g_[u].state, g_[v].state);
        edge_descriptor e; bool edge_added;
        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > radius_) {
          continue;
        }
        // Too close vertice, remove it from g_ and do not construct the edge to it
        if (dist < 0.01 /*do not add same vertice twice*/) {
          bool is_nb_start_or_goal = (nb->id == start_vertex_.id || nb->id == goal_vertex_.id);
          if (!is_nb_start_or_goal) {
            vertices_to_be_removed.push_back(v);
            nn_->remove(nb);
            continue;
          }
        }
        // Once suitable edges are found, populate them over graphs
        boost::tie(e, edge_added) = boost::add_edge(u, v, WeightProperty(dist), g_);
        addEdgeApx(&g_[vd], nb, dist);
        addEdgeLb(&g_[vd], nb, dist);
      }
    }

    // Remove close/duplicate vertices
    for (auto && i : vertices_to_be_removed) {
      boost::remove_vertex(i, g_);
    }

    forwardPath.clear(); reversePath.clear();
    double costApx = LPAstarCost2Come_->computeShortestPath(reversePath);
    double costLb = LPAstarCost2Go_->computeShortestPath(forwardPath);

    bool adaptive_h_available = (*(LPAstarCost2Go_))(start_vertex_.id) !=
      std::numeric_limits<double>::infinity();

    int num_visited_nodes = 0;
    std::vector<vertex_descriptor> p(boost::num_vertices(g_));
    std::vector<Cost> d(boost::num_vertices(g_));

    if (!adaptive_h_available) {
      OMPL_INFORM(
        "%s: Adaptive H function not available, skipping this cycle.\n",
        getName().c_str());
      goto skip_this_cycle;
    }
/*
    try {
      // Run A* qith Hueroistic being from ForwardPropogateHeuristic
      auto heuristic = ForwardPropogateHeuristic(this);
      auto c_visitor = custom_goal_visitor<vertex_descriptor>(
        goal_vertex_.id, &num_visited_nodes, this);

      boost::astar_search_tree(
        g_, start_vertex_.id, heuristic,
        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));

      OMPL_INFORM("%s: A* Failed to produce final collision free path.\n", getName().c_str() );

    } catch (FoundGoal found_goa) {

      OMPL_INFORM("%s: A valid geometric path has been found.\n", getName().c_str() );

      // Found a path to the goal, catch the exception
      std::list<vertex_descriptor> shortest_path;
      for (vertex_descriptor v = goal_vertex_.id;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v) {break;}
      }

      // Lets clean the graphs and remove useless vertex/edges
      // The documentation is pretty clear that you cannot remove edges while iterating over them,
      // as it invalidates the iterators. You need to store the edge descriptors, and remove them at later step.

      std::vector<std::pair<vertex_descriptor, vertex_descriptor>> edges_to_be_removed;
      auto es = boost::edges(g_);
      for (auto eit = es.first; eit != es.second; ++eit) {
        vertex_descriptor u, v;
        u = boost::source(*eit, g_);
        v = boost::target(*eit, g_);
        double u_estimate = (*(LPAstarCost2Go_))(g_[u].id); // cost to come
        double v_estimate = (*(LPAstarCost2Go_))(g_[v].id); // cost to go

        bool is_u_or_v_start_or_goal =
          (u == start_vertex_.id || u == goal_vertex_.id) ||
          (v == start_vertex_.id || v == goal_vertex_.id);

        if (is_u_or_v_start_or_goal) {
          continue;
        }

        double edge_dist = distanceFunction(g_[u].state, g_[v].state);
        if ((g_[u].blacklisted || g_[v].blacklisted) || (edge_dist > radius_) ||
          (  u_estimate == std::numeric_limits<double>::infinity() &&
          v_estimate == std::numeric_limits<double>::infinity()))
        {
          edges_to_be_removed.push_back(std::make_pair(u, v));
        }
      }

      for (auto && i : edges_to_be_removed) {
        if (edgeExistsApx(i.first, i.second)) {
          removeEdgeApx(i.first, i.second);
        }
        if (edgeExistsLb(i.first, i.second)) {
          removeEdgeLb(i.first, i.second);
        }
        if (boost::edge(i.first, i.second, g_).second) {
          boost::remove_edge(i.first, i.second, g_);
        }
      }
    }
*/
skip_this_cycle:
    OMPL_INFORM(
      "%s: Advancing with %d vertices and %d edges.\n",
      getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));

    visualizeRGG(g_);

  }

  goal_reached = (*(LPAstarCost2Go_))(goal_vertex_.id) !=
    std::numeric_limits<double>::infinity();
  if (goal_reached) {
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

  auto a2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = a2 - a1;

  //OMPL_INFORM("%s: Took  %.4f milliseconds.\n", getName().c_str(), elapsed.count());

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
        // Count how many states we've checked.
      } while (!si_->getStateValidityChecker()->isValid(samples.back()));
    }
  } while (samples.size() < batch_size);
}

void ompl::control::AITStarKin::visualizeRGG(const GraphT & g)
{

  bool show_infs = false;

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

  visualization_msgs::msg::MarkerArray marker_array;

  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap
  for (auto vd : boost::make_iterator_range(vertices(g))) {
    double apx_estimate = (*(LPAstarCost2Come_))(g[vd].id);   // cpst to come
    double lb_estimate = (*(LPAstarCost2Go_))(g[vd].id);   // cost to go
    std::stringstream ss_lb, ss_apx;
    ss_lb << std::setprecision(2) << lb_estimate;
    ss_apx << std::setprecision(2) << apx_estimate;


    if (!show_infs && lb_estimate == std::numeric_limits<double>::infinity()) {
      continue;
    }

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
    text.text = ss_apx.str() + "/" + ss_lb.str();
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
    double u_estimate = (*(LPAstarCost2Go_))(g[u].id);   // cpst to come
    double v_estimate = (*(LPAstarCost2Go_))(g[v].id);   // cost to go

    if (!show_infs &&
      (u_estimate == std::numeric_limits<double>::infinity() ||
      v_estimate == std::numeric_limits<double>::infinity())  )
    {
      continue;
    }

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
