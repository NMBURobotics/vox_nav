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
    control_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
  }
  nn_->setDistanceFunction(
    [this](const VertexProperty * a, const VertexProperty * b)
    {
      return distanceFunction(a, b);
    });

  control_nn_->setDistanceFunction(
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
  if (!path_informed_sampler_) {
    path_informed_sampler_ = std::make_shared<base::PathLengthDirectInfSampler>(
      pdef_,
      std::numeric_limits<double>::infinity());
  }
  if (!rejection_informed_sampler_) {
    rejection_informed_sampler_ = std::make_shared<base::RejectionInfSampler>(
      pdef_,
      std::numeric_limits<double>::infinity());
  }
  if (!controlSampler_) {
    controlSampler_ = std::make_shared<SimpleDirectedControlSampler>(siC_, k_number_of_controls_);
  }

  // RVIZ VISUALIZATIONS
  node_ = std::make_shared<rclcpp::Node>("aitstarkin_rclcpp_node");

  rgg_graph_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/aitstarkin/rgg", rclcpp::SystemDefaultsQoS());

  geometric_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/aitstarkin/g_plan", rclcpp::SystemDefaultsQoS());

  control_graph_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/aitstarkin/control_rgg", rclcpp::SystemDefaultsQoS());

  control_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/aitstarkin/c_plan", rclcpp::SystemDefaultsQoS());

}

void ompl::control::AITStarKin::clear()
{
  Planner::clear();
  sampler_.reset();
  valid_state_sampler_.reset();
  path_informed_sampler_.reset();
  rejection_informed_sampler_.reset();

  bestCost_ = opt_->infiniteCost();

  freeMemory();
  if (nn_) {
    nn_->clear();
    control_nn_->clear();
  }
  g_.clear();
  g_ = GraphT();
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

  goal_vertex_ = new VertexProperty();
  start_vertex_ = new VertexProperty();

  base::Goal * goal = pdef_->getGoal().get();
  auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

  // get the goal node and state
  auto * goal_state = si_->allocState();
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }
  goal_vertex_->state = goal_state;
  nn_->add(goal_vertex_);
  control_nn_->add(goal_vertex_);

  // get start node and state,push  the node inton nn_ as well
  auto * start_state = si_->allocState();
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }
  start_vertex_->state = start_state;
  nn_->add(start_vertex_);
  control_nn_->add(start_vertex_);

  // Add goal and start to graph
  vertex_descriptor start_vertex_descriptor = boost::add_vertex(g_);
  vertex_descriptor goal_vertex_descriptor = boost::add_vertex(g_);
  start_vertex_->id = start_vertex_descriptor;
  goal_vertex_->id = goal_vertex_descriptor;
  g_[start_vertex_descriptor] = *start_vertex_;
  g_[goal_vertex_descriptor] = *goal_vertex_;

  vertex_descriptor start_vertex_descriptor_control = boost::add_vertex(g_control_);
  vertex_descriptor goal_vertex_descriptor_control = boost::add_vertex(g_control_);
  g_control_[start_vertex_descriptor_control] = *start_vertex_;
  g_control_[goal_vertex_descriptor_control] = *goal_vertex_;

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n", getName().c_str(),
    nn_->size());

  std::list<std::size_t> forwardPath, reversePath;

  WeightMap weightmap = get(boost::edge_weight, g_);
  WeightMap weightmap_control = get(boost::edge_weight, g_control_);

  auto path(std::make_shared<PathControl>(si_));
  bool goal_reached{false};

  while (ptc == false) {

    std::vector<ompl::base::State *> samples;
    generateBatchofSamples(batch_size_, use_valid_sampler_, samples);

    // Add batch_size_ number of samples to graphs
    // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
    // But too close vertices will be discarded in order for memory not to sink
    for (auto && i : samples) {
      VertexProperty * this_vertex_property = new VertexProperty();
      this_vertex_property->state = (i);
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
          weightmap[e] = opt_->motionCost(g_[u].state, g_[v].state).value();
        }
      }
    }

    std::vector<ompl::control::AITStarKin::VertexProperty *> goal_nbh;
    nn_->nearestR(goal_vertex_, radius_, goal_nbh);
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
      weightmap[e] = opt_->motionCost(g_[u].state, g_[v].state).value();
    }

    int num_visited_nodes = 0;
    std::vector<vertex_descriptor> p(boost::num_vertices(g_));
    std::vector<Cost> d(boost::num_vertices(g_));
    std::list<vertex_descriptor> shortest_path;
    std::list<vertex_descriptor> shortest_path_control;

    // Run A* backwards from goal to start
    try {
      auto heuristic = GenericDistanceHeuristic<GraphT, VertexProperty, Cost>(this, start_vertex_);
      auto c_visitor = SimpleVertexVisitor<vertex_descriptor>(
        start_vertex_descriptor, &num_visited_nodes);

      if constexpr (use_astar_hueristic_) {
        boost::astar_search_tree(
          g_, goal_vertex_descriptor, heuristic,
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));
      } else {
        boost::dijkstra_shortest_paths(
          g_, goal_vertex_descriptor,
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));
      }

      OMPL_WARN(
        "%s: A* Failed to produce Heuristic that connects goal to start vertex after %d vertex visits.\n",
        getName().c_str(), num_visited_nodes);

    } catch (FoundVertex found_goal) {

      // Catch the exception
      // Found a Heuristic from start to the goal (no collision checks),
      // We now have H function
      for (auto vd : boost::make_iterator_range(vertices(g_))) {
        g_[vd].g = d[vd];
      }

      p.clear(); d.clear();
      p.resize(boost::num_vertices(g_));  d.resize(boost::num_vertices(g_));
      num_visited_nodes = 0;

      // Now we can run A* forwards from start to goal and check for collisions
      try {
        auto h_heuristic = PrecomputedCostHeuristic<GraphT, Cost>(this);
        auto generic_visitor = SimpleVertexVisitor<vertex_descriptor>(
          goal_vertex_descriptor, &num_visited_nodes);

        boost::astar_search_tree(
          g_, start_vertex_descriptor, h_heuristic,
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(generic_visitor));

        OMPL_INFORM(
          "%s: A* Failed to produce final collision free path after %d node visits.\n",
          getName().c_str(), num_visited_nodes);

      } catch (FoundVertex found_goal) {

        // Found a collision free path to the goal, catch the exception
        shortest_path.clear();
        for (vertex_descriptor v = goal_vertex_->id;; v = p[v]) {
          if (g_[v].blacklisted) {
            OMPL_WARN(
              "%s: Found a blacklisted node most likely due to collision, this path is invalid but we will find another one.\n",
              getName().c_str());
            for (auto ed : boost::make_iterator_range(boost::out_edges(v, g_))) {
              weightmap[ed] = std::numeric_limits<double>::infinity();
            }
          }
          shortest_path.push_front(v);
          if (p[v] == v) {break;}
        }

        // Now we have a collision free path, we can now find a control path
        // Add all samples to the control NN and contol graph
        for (auto && i : samples) {
          VertexProperty * this_vertex_property = new VertexProperty();
          // Do not modiy original sample, as that will affect geometric RGG
          auto deep_copy_sample_state = si_->allocState();
          si_->copyState(deep_copy_sample_state, i);
          this_vertex_property->state = deep_copy_sample_state;
          std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
          control_nn_->nearestK(this_vertex_property, 1, nbh);

          if (nbh.size() == 0) {
            continue;
          }

          bool does_vertice_exits{false};
          double dist = distanceFunction(i, nbh.front()->state);
          if (dist < min_dist_between_vertices_ /*do not add same vertice twice*/) {
            does_vertice_exits = true;
          }

          if (!does_vertice_exits) {
            // Attempt to drive towards newly added sample
            // modify the sample to latest arrived state
            auto c = siC_->allocControl();
            auto duration = controlSampler_->sampleTo(
              c,
              nbh.front()->state,
              this_vertex_property->state);

            if (duration == 0) {
              // perhaprs due to invalidy of the state, we cannot proceed
              siC_->freeControl(c);
              si_->freeState(this_vertex_property->state);
              continue;
            }

            vertex_descriptor this_vertex_descriptor = boost::add_vertex(g_control_);
            this_vertex_property->id = this_vertex_descriptor;
            g_control_[this_vertex_descriptor] = *this_vertex_property;
            g_control_[this_vertex_descriptor].control = c;
            g_control_[this_vertex_descriptor].control_duration = duration;

            control_nn_->add(this_vertex_property);
            vertex_descriptor u = this_vertex_descriptor;
            vertex_descriptor v = nbh.front()->id;
            double dist = distanceFunction(g_control_[u].state, g_control_[v].state);
            edge_descriptor e; bool edge_added;
            // not to construct edges with self, and if nbh is further than radius_, continue
            if (u == v || dist > radius_) {
              continue;
            }
            if (boost::edge(u, v, g_control_).second || boost::edge(v, u, g_control_).second) {
              continue;
            }
            // Once suitable edges are found, populate them over graphs
            boost::tie(e, edge_added) = boost::add_edge(u, v, g_control_);
            weightmap_control[e] =
              opt_->motionCost(g_control_[u].state, g_control_[v].state).value();
          }
        }

        p.clear(); d.clear();
        p.resize(boost::num_vertices(g_control_));  d.resize(boost::num_vertices(g_control_));
        num_visited_nodes = 0;
        try {
          auto heuristic = GenericDistanceHeuristic<GraphT, VertexProperty, Cost>(
            this,
            &g_control_[goal_vertex_descriptor_control], true);
          auto c_visitor = SimpleVertexVisitor<vertex_descriptor>(
            goal_vertex_descriptor_control, &num_visited_nodes);

          boost::astar_search_tree(
            g_control_, start_vertex_descriptor_control, heuristic,
            boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));

          OMPL_INFORM(
            "%s: A* Failed to produce Kinodynamic path after %d node visits.\n",
            getName().c_str(), num_visited_nodes);

        } catch (FoundVertex found_goal_control) {

          // Found a collision free path to the goal, catch the exception
          shortest_path_control.clear();
          for (vertex_descriptor v = goal_vertex_descriptor_control;; v = p[v]) {
            shortest_path_control.push_front(v);
            if (p[v] == v) {break;}
          }
        }

        /*std::vector<VertexProperty> copy_shortest_path(shortest_path.size(), VertexProperty());
        for (size_t i = 0; i < shortest_path.size(); i++) {
          copy_shortest_path[i].id = g_[*std::next(shortest_path.begin(), i)].id;
          copy_shortest_path[i].state = si_->allocState();
          copy_shortest_path[i].blacklisted = g_[*std::next(shortest_path.begin(), i)].blacklisted;
          copy_shortest_path[i].g = g_[*std::next(shortest_path.begin(), i)].g;
          copy_shortest_path[i].control = siC_->allocControl();
          si_->copyState(
            copy_shortest_path[i].state,
            g_[*std::next(shortest_path.begin(), i)].state);
        }
        goal_reached = true;
        OMPL_INFORM("%s: Calculating Kinodynamic path.\n", getName().c_str());
        for (size_t i = 1; i < copy_shortest_path.size(); i++) {
          auto duration = controlSampler_->sampleTo(
            copy_shortest_path[i - 1].control,
            copy_shortest_path[i - 1].state,
            copy_shortest_path[i].state);
          copy_shortest_path[i - 1].control_duration = duration;
          if (duration == 0) {
            OMPL_WARN(
              "%s: Control sampler failed to find a valid control between two states.\n",
              getName().c_str());
          }
        }
        //
        path = std::make_shared<PathControl>(si_);
        for (int i = 0; i < copy_shortest_path.size(); i++) {
          if (i == 0) {
            path->append(copy_shortest_path[i].state);
          } else {
            path->append(
              copy_shortest_path[i].state, copy_shortest_path[i].control,
              copy_shortest_path[i].control_duration * siC_->getPropagationStepSize());
          }
        }
        // Set the cost of the solution.
        bestCost_ = ompl::base::Cost(path->length());

        // Create a solution.
        ompl::base::PlannerSolution solution(path);
        solution.setPlannerName(getName());
        solution.setOptimized(opt_, bestCost_, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(solution); */

      }

    }

    OMPL_INFORM(
      "%s: Advancing with %d vertices and %d edges.\n",
      getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));

    visualizePath(g_, shortest_path, geometric_path_pub_, "g");
    visualizePath(g_control_, shortest_path_control, control_path_pub_, "c");

    visualizeRGG(
      g_,
      rgg_graph_pub_,
      "g",
      0.87 /*only for color of vis*/,
      start_vertex_descriptor,
      goal_vertex_descriptor);

    visualizeRGG(
      g_control_,
      control_graph_pub_,
      "c",
      0.24,
      start_vertex_descriptor_control,
      goal_vertex_descriptor_control);

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
        //sampler_->sampleUniform(samples.back());
        //path_informed_sampler_->sampleUniform(samples.back(), bestCost_);
        rejection_informed_sampler_->sampleUniform(samples.back(), bestCost_);

        // Count how many states we've checked.
      } while (!si_->getStateValidityChecker()->isValid(samples.back()));
    }
  } while (samples.size() < batch_size);
}

void ompl::control::AITStarKin::visualizeRGG(
  const GraphT & g,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns,
  const double & blue_color,
  const vertex_descriptor & start_vertex,
  const vertex_descriptor & goal_vertex)
{

  // Clear All previous markers
  visualization_msgs::msg::MarkerArray clear_markers;
  visualization_msgs::msg::Marker rgg_vertex, rgg_edges;
  rgg_vertex.id = 0;
  rgg_edges.id = 0;
  rgg_vertex.ns = ns + "rgg_vertex";
  rgg_edges.ns = ns + "rgg_edges";
  rgg_vertex.action = visualization_msgs::msg::Marker::DELETEALL;
  rgg_edges.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_markers.markers.push_back(rgg_vertex);
  clear_markers.markers.push_back(rgg_edges);
  publisher->publish(clear_markers);

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = "map";
  sphere.header.stamp = rclcpp::Clock().now();
  sphere.ns = ns + "rgg_vertex";
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
    // Paint the start and goal vertices differently.
    double is_goal_or_start{0.0};
    if (g[vd].id == start_vertex || g[vd].id == goal_vertex) {
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
    color.b = blue_color;
    sphere.points.push_back(point);
    sphere.colors.push_back(color);
  }
  marker_array.markers.push_back(sphere);

  auto es = boost::edges(g);
  vertex_descriptor u, v;
  int edge_index = 0;
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.ns = ns + "rgg_edges";
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
  line_strip.color.b = blue_color;
  std_msgs::msg::ColorRGBA color;
  color.b = blue_color;
  color.a = 1.0;

  for (auto eit = es.first; eit != es.second; ++eit) {
    u = boost::source(*eit, g);
    v = boost::target(*eit, g);

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
    line_strip.colors.push_back(color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(color);
  }
  marker_array.markers.push_back(line_strip);

  publisher->publish(marker_array);
}

void ompl::control::AITStarKin::visualizePath(
  const GraphT & g,
  const std::list<vertex_descriptor> & path,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns
)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.ns = ns + "path";
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

  for (size_t i = 1; i < path.size(); i++) {
    auto u = *std::next(path.begin(), i - 1);
    auto v = *std::next(path.begin(), i);
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

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = "rgg_costs";
    text.id = g[u].id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0);
    text.text = std::to_string(g[u].g);
    text.pose.position = source_point;
    text.pose.position.z += 0.5;
    text.scale.x = 0.3;
    text.scale.y = 0.3;
    text.scale.z = 0.3;
    text.color.a = 1.0;
    text.color.r = 1.0;
    marker_array.markers.push_back(text);
  }
  marker_array.markers.push_back(line_strip);

  publisher->publish(marker_array);
}
