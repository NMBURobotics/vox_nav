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
    forward_control_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
    backward_control_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
  }
  nn_->setDistanceFunction(
    [this](const VertexProperty * a, const VertexProperty * b)
    {
      return distanceFunction(a, b);
    });

  forward_control_nn_->setDistanceFunction(
    [this](const VertexProperty * a, const VertexProperty * b)
    {
      return distanceFunction(a, b);
    });
  backward_control_nn_->setDistanceFunction(
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
    forward_control_nn_->clear();
    backward_control_nn_->clear();
  }
  g_.clear();
  g_ = GraphT();

  g_backward_control_.clear();
  g_backward_control_ = GraphT();
  g_forward_control_.clear();
  g_forward_control_ = GraphT();
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
  backward_control_nn_->add(goal_vertex_);

  // get start node and state,push  the node inton nn_ as well
  auto * start_state = si_->allocState();
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }
  start_vertex_->state = start_state;
  nn_->add(start_vertex_);
  forward_control_nn_->add(start_vertex_);

  // Add goal and start to graph
  vertex_descriptor start_vertex_descriptor = boost::add_vertex(g_);
  vertex_descriptor goal_vertex_descriptor = boost::add_vertex(g_);
  start_vertex_->id = start_vertex_descriptor;
  goal_vertex_->id = goal_vertex_descriptor;
  g_[start_vertex_descriptor] = *start_vertex_;
  g_[goal_vertex_descriptor] = *goal_vertex_;

  vertex_descriptor forward_control_g_root = boost::add_vertex(g_forward_control_);
  vertex_descriptor forward_control_g_target = boost::add_vertex(g_forward_control_);
  g_forward_control_[forward_control_g_root] = *start_vertex_;
  g_forward_control_[forward_control_g_root].id = forward_control_g_root;
  g_forward_control_[forward_control_g_target] = *goal_vertex_;
  g_forward_control_[forward_control_g_target].id = forward_control_g_target;

  vertex_descriptor backward_control_g_root = boost::add_vertex(g_backward_control_);
  g_backward_control_[backward_control_g_root] = *goal_vertex_; // rooted in goal, aims for start
  g_backward_control_[backward_control_g_root].id = backward_control_g_root; // rooted in goal, aims for start

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n", getName().c_str(),
    nn_->size());

  std::list<std::size_t> forwardPath, reversePath;

  WeightMap weightmap = get(boost::edge_weight, g_);
  WeightMap weightmap_forward_control = get(boost::edge_weight, g_forward_control_);
  WeightMap weightmap_backward_control = get(boost::edge_weight, g_backward_control_);

  auto path(std::make_shared<PathControl>(si_));
  bool goal_reached{false};

  while (ptc == false) {

    std::vector<ompl::base::State *> samples;
    generateBatchofSamples(batch_size_, use_valid_sampler_, samples);

    expandGeometricGraph(samples, g_, nn_, weightmap);

    ensureGoalVertexConnectivity(goal_vertex_, g_, nn_, weightmap);

    int num_visited_nodes = 0;
    std::vector<vertex_descriptor> p(boost::num_vertices(g_));
    std::vector<GraphEdgeCost> d(boost::num_vertices(g_));
    std::list<vertex_descriptor> shortest_path;
    std::list<vertex_descriptor> shortest_path_control;

    // Run A* backwards from goal to start
    try {
      auto heuristic = GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
        this,
        start_vertex_);
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
        auto h_heuristic = PrecomputedCostHeuristic<GraphT, GraphEdgeCost>(this);
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
              weightmap[ed] = opt_->infiniteCost().value();
            }
          }
          shortest_path.push_front(v);
          if (p[v] == v) {break;}
        }

        expandControlGraph(
          samples,
          goal_state /*goal vertex state*/,
          forward_control_g_target /*goal vertex desc*/,
          g_forward_control_,
          forward_control_nn_,
          weightmap_forward_control);

        p.clear(); d.clear();
        p.resize(boost::num_vertices(g_forward_control_));
        d.resize(boost::num_vertices(g_forward_control_));
        num_visited_nodes = 0;
        try {

          auto heuristic = GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
            this,
            &g_forward_control_[forward_control_g_target], true, false);

          auto c_visitor = SimpleVertexVisitor<vertex_descriptor>(
            forward_control_g_target, &num_visited_nodes);

          boost::astar_search_tree(
            g_forward_control_, forward_control_g_root, heuristic,
            boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));

          OMPL_INFORM(
            "%s: A* Failed to produce Kinodynamic path after %d node visits.\n",
            getName().c_str(), num_visited_nodes);

        } catch (FoundVertex found_goal_control) {

          for (auto vd : boost::make_iterator_range(vertices(g_forward_control_))) {
            g_forward_control_[vd].g = d[vd];
          }

          // Found a collision free path to the goal, catch the exception
          shortest_path_control.clear();
          for (vertex_descriptor v = forward_control_g_target;; v = p[v]) {
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

    std::string red("red");
    std::string green("green");

    visualizePath(
      g_,
      shortest_path,
      geometric_path_pub_,
      "g",
      getColor(green));

    visualizePath(
      g_forward_control_,
      shortest_path_control,
      control_path_pub_,
      "c",
      getColor(red));

    visualizeRGG(
      g_,
      rgg_graph_pub_,
      "g",
      getColor(green),
      start_vertex_descriptor,
      goal_vertex_descriptor);

    visualizeRGG(
      g_forward_control_,
      control_graph_pub_,
      "c",
      getColor(red),
      forward_control_g_root,
      forward_control_g_target);

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

void ompl::control::AITStarKin::expandGeometricGraph(
  const std::vector<ompl::base::State *> & samples,
  GraphT & geometric_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
  WeightMap & geometric_weightmap)
{
  // Add batch_size_ number of samples to graphs
  // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
  // But too close vertices will be discarded in order for memory not to sink
  for (auto && i : samples) {
    VertexProperty * this_vertex_property = new VertexProperty();
    this_vertex_property->state = (i);
    std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
    geometric_nn->nearestR(this_vertex_property, radius_, nbh);
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
      vertex_descriptor this_vertex_descriptor = boost::add_vertex(geometric_graph);
      this_vertex_property->id = this_vertex_descriptor;
      geometric_graph[this_vertex_descriptor] = *this_vertex_property;
      geometric_nn->add(this_vertex_property);
      for (auto && nb : nbh) {
        vertex_descriptor u = this_vertex_descriptor;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(geometric_graph[u].state, geometric_graph[v].state);
        edge_descriptor e; bool edge_added;
        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > radius_) {
          continue;
        }
        if (boost::edge(
            u, v,
            geometric_graph).second || boost::edge(v, u, geometric_graph).second)
        {
          continue;
        }
        // Once suitable edges are found, populate them over graphs
        boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
        geometric_weightmap[e] =
          opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
      }
    }
  }
}

void ompl::control::AITStarKin::ensureGoalVertexConnectivity(
  VertexProperty * target_vertex_property,
  GraphT & geometric_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
  WeightMap & geometric_weightmap)
{
  // Neihbors of goal vertex
  std::vector<ompl::control::AITStarKin::VertexProperty *> goal_nbh;
  geometric_nn->nearestR(target_vertex_property, radius_, goal_nbh);
  if (goal_nbh.size() > max_neighbors_) {
    goal_nbh.resize(max_neighbors_);
  }
  for (auto && nb : goal_nbh) {
    vertex_descriptor u = target_vertex_property->id;
    vertex_descriptor v = nb->id;
    double dist = distanceFunction(geometric_graph[u].state, geometric_graph[v].state);
    edge_descriptor e; bool edge_added;
    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || dist > radius_) {
      continue;
    }
    if (boost::edge(u, v, geometric_graph).second || boost::edge(v, u, geometric_graph).second) {
      continue;
    }
    // Once suitable edges are found, populate them over graphs
    boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
    geometric_weightmap[e] =
      opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
  }
}

void ompl::control::AITStarKin::expandControlGraph(
  const std::vector<ompl::base::State *> & samples,
  const ompl::base::State * target_vertex_state,
  const vertex_descriptor & target_vertex_descriptor,
  GraphT & control_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
  WeightMap & control_weightmap)
{
  // Now we have a collision free path, we can now find a control path
  // Add all samples to the control NN and contol graph
  int ith_sample = static_cast<int>(1.0 / goal_bias_);
  int index_of_goal_bias{0};

  for (auto && i : samples) {
    VertexProperty * this_vertex_property = new VertexProperty();
    this_vertex_property->state = i;

    // Every ith sample is goal biasing
    if ((index_of_goal_bias & ith_sample) == 0) {
      auto deep_copy_target_state = si_->allocState();
      si_->copyState(deep_copy_target_state, target_vertex_state);
      this_vertex_property->state = deep_copy_target_state;
    }
    index_of_goal_bias++;

    std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
    control_nn->nearestR(this_vertex_property, radius_, nbh);

    if (nbh.size() == 0) {
      continue;
    }
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
      for (auto && nb : nbh) {
        if (nb->id == target_vertex_descriptor) {
          // Do not add edge to target vertex
          continue;
        }
        // Do not modify original sample, as that will affect geometric RGG
        auto deep_copy_sample_state = si_->allocState();
        si_->copyState(deep_copy_sample_state, i);
        // Attempt to drive towards newly added sample
        // modify the sample to latest arrived state
        auto c = siC_->allocControl();
        auto duration = controlSampler_->sampleTo(
          c,
          nb->state,
          deep_copy_sample_state);

        if (duration == 0) {
          // perhaprs due to invalidy of the state, we cannot proceed
          siC_->freeControl(c);
          si_->freeState(deep_copy_sample_state);
          continue;
        }

        vertex_descriptor arrived_vertex_descriptor = boost::add_vertex(control_graph);
        auto arrived_vertex_property = new VertexProperty();
        arrived_vertex_property->id = arrived_vertex_descriptor;
        arrived_vertex_property->state = deep_copy_sample_state;
        control_graph[arrived_vertex_descriptor] = *arrived_vertex_property;
        control_graph[arrived_vertex_descriptor].state = deep_copy_sample_state;
        control_graph[arrived_vertex_descriptor].control = c;
        control_graph[arrived_vertex_descriptor].control_duration = duration;
        control_nn->add(arrived_vertex_property);

        vertex_descriptor u = arrived_vertex_descriptor;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(control_graph[u].state, control_graph[v].state);
        edge_descriptor e; bool edge_added;
        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > radius_) {
          continue;
        }
        if (boost::edge(
            u, v,
            control_graph).second || boost::edge(v, u, control_graph).second)
        {
          continue;
        }
        // Once suitable edges are found, populate them over graphs
        boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
        control_weightmap[e] =
          opt_->motionCost(control_graph[u].state, control_graph[v].state).value();

        // calculate the distance between the arrived state and the start/goal

        double dist_to_target = distanceFunction(
          arrived_vertex_property->state,
          target_vertex_state);

        // if the distance is less than the radius to target, then add the edge
        if (dist_to_target < radius_ / 2.0) {

          vertex_descriptor u = arrived_vertex_property->id;
          vertex_descriptor v = target_vertex_descriptor;
          double dist = distanceFunction(control_graph[u].state, control_graph[v].state);
          edge_descriptor e; bool edge_added;
          // not to construct edges with self, and if nbh is further than radius_, continue
          if (u == v || dist > radius_) {
            continue;
          }
          if (boost::edge(u, v, control_graph).second ||
            boost::edge(v, u, control_graph).second)
          {
            continue;
          }
          // Once suitable edges are found, populate them over graphs
          boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
          control_weightmap[e] =
            opt_->motionCost(control_graph[u].state, control_graph[v].state).value();
        }

      }
    }
  }
}

void ompl::control::AITStarKin::visualizeRGG(
  const GraphT & g,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
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
    std_msgs::msg::ColorRGBA color_vd = color;
    double is_goal_or_start{0.0};
    if (g[vd].id == start_vertex || g[vd].id == goal_vertex) {
      color_vd.b *= 0.5;
    }
    const auto * target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    geometry_msgs::msg::Point point;
    point.x = target_xyzv->values[0];
    point.y = target_xyzv->values[1];
    point.z = target_xyzv->values[2];
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
  line_strip.color = color;

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
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color
)
{
  // Clear All previous markers
  visualization_msgs::msg::MarkerArray clear_markers;
  visualization_msgs::msg::Marker path_marker, cost_marker;
  path_marker.id = 0;
  cost_marker.id = 0;
  path_marker.ns = ns + "path";
  cost_marker.ns = ns + "costs";
  path_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  cost_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_markers.markers.push_back(path_marker);
  clear_markers.markers.push_back(cost_marker);
  publisher->publish(clear_markers);

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
  line_strip.color = color;

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
    line_strip.colors.push_back(color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(color);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = ns + "costs";
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

std_msgs::msg::ColorRGBA ompl::control::AITStarKin::getColor(std::string & color)
{
  std_msgs::msg::ColorRGBA color_rgba;
  if (color == "red") {
    color_rgba.r = 1.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  } else if (color == "green") {
    color_rgba.r = 0.0;
    color_rgba.g = 1.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  } else if (color == "blue") {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  } else if (color == "yellow") {
    color_rgba.r = 1.0;
    color_rgba.g = 1.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  } else if (color == "magenta") {
    color_rgba.r = 1.0;
    color_rgba.g = 0.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  } else if (color == "cyan") {
    color_rgba.r = 0.0;
    color_rgba.g = 1.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  } else if (color == "white") {
    color_rgba.r = 1.0;
    color_rgba.g = 1.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  } else if (color == "black") {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  } else {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  return color_rgba;
}
