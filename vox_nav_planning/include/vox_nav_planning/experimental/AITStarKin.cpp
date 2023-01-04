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
  if (!geometric_nn_) {
    geometric_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
    forward_control_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
    backward_control_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
  }

  geometric_nn_->setDistanceFunction(
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

  k_rgg_ = boost::math::constants::e<double>() +
    (boost::math::constants::e<double>() / si_->getStateDimension());

  bestPath_ = std::make_shared<PathControl>(si_);

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
  radius_ = std::numeric_limits<double>::infinity();
  numNeighbors_ = std::numeric_limits<std::size_t>::max();
  bestCost_ = opt_->infiniteCost();

  freeMemory();
  if (geometric_nn_) {
    geometric_nn_->clear();
    forward_control_nn_->clear();
    backward_control_nn_->clear();
  }
  // clear the graphs
  g_geometric_.clear();
  g_geometric_ = GraphT();
  g_backward_control_.clear();
  g_backward_control_ = GraphT();
  g_forward_control_.clear();
  g_forward_control_ = GraphT();
}

void ompl::control::AITStarKin::freeMemory()
{
  if (geometric_nn_) {
    std::vector<VertexProperty *> nodes;
    geometric_nn_->list(nodes);
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
  // check if the problem is setup properly
  checkValidity();

  // reset goal and start vertex properties
  goal_vertex_ = new VertexProperty();
  start_vertex_ = new VertexProperty();

  // get the goal node and state
  auto * goal_state = si_->allocState();
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }
  goal_vertex_->state = goal_state;
  geometric_nn_->add(goal_vertex_);
  backward_control_nn_->add(goal_vertex_);

  // get start node and state
  auto * start_state = si_->allocState();
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }
  start_vertex_->state = start_state;
  geometric_nn_->add(start_vertex_);
  forward_control_nn_->add(start_vertex_);

  // Add goal and start to geomteric graph
  vertex_descriptor start_vertex_descriptor = boost::add_vertex(g_geometric_);
  vertex_descriptor goal_vertex_descriptor = boost::add_vertex(g_geometric_);
  start_vertex_->id = start_vertex_descriptor;
  goal_vertex_->id = goal_vertex_descriptor;
  g_geometric_[start_vertex_descriptor] = *start_vertex_;
  g_geometric_[goal_vertex_descriptor] = *goal_vertex_;

  // Add goal and start to forward control graph
  vertex_descriptor forward_control_g_root = boost::add_vertex(g_forward_control_);
  vertex_descriptor forward_control_g_target = boost::add_vertex(g_forward_control_);
  g_forward_control_[forward_control_g_root] = *start_vertex_;
  g_forward_control_[forward_control_g_root].id = forward_control_g_root;
  g_forward_control_[forward_control_g_target] = *goal_vertex_;
  g_forward_control_[forward_control_g_target].id = forward_control_g_target;

  // Add goal and start to backward control graph
  vertex_descriptor backward_control_g_root = boost::add_vertex(g_backward_control_);
  vertex_descriptor backward_control_g_target = boost::add_vertex(g_backward_control_);
  g_backward_control_[backward_control_g_root] = *goal_vertex_; // rooted in goal, aims for start
  g_backward_control_[backward_control_g_root].id = backward_control_g_root; // rooted in goal, aims for start
  g_backward_control_[backward_control_g_target] = *start_vertex_; // rooted in goal, aims for start
  g_backward_control_[backward_control_g_target].id = backward_control_g_target; // rooted in goal, aims for start

  if (geometric_nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n", getName().c_str(),
    geometric_nn_->size());

  std::list<std::size_t>
  shortest_path_geometric, shortest_path_forward_control, shortest_path_backward_control;

  WeightMap weightmap_geometric = get(boost::edge_weight, g_geometric_);
  WeightMap weightmap_forward_control = get(boost::edge_weight, g_forward_control_);
  WeightMap weightmap_backward_control = get(boost::edge_weight, g_backward_control_);

  auto geometric_path(std::make_shared<PathControl>(si_));
  auto forward_control_path(std::make_shared<PathControl>(si_));
  auto backward_control_path(std::make_shared<PathControl>(si_));

  bool goal_reached{false};

  while (ptc == false) {

    std::vector<ompl::base::State *> samples;

    generateBatchofSamples(batch_size_, use_valid_sampler_, samples);

    // Geometric graph Thread
    geometric_thread_ = new std::thread(
      [this,
      &samples,
      &weightmap_geometric,
      &shortest_path_geometric,
      &goal_vertex_descriptor,
      &start_vertex_descriptor
      ]
      {
        expandGeometricGraph(samples, g_geometric_, geometric_nn_, weightmap_geometric);

        ensureGoalVertexConnectivity(
          goal_vertex_, g_geometric_, geometric_nn_,
          weightmap_geometric);

        // First lets compute an heuristic working backward from goal -> start
        // This is done by running A*/Dijstra backwards from goal to start with no collision checking
        auto heuristic =
        GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(this, start_vertex_);
        shortest_path_geometric =
        computeShortestPath<GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>>(
          g_geometric_, weightmap_geometric, heuristic, goal_vertex_descriptor,
          start_vertex_descriptor, true, false);

        if (shortest_path_geometric.size() > 0) {
          // precomputed heuristic is available, lets use it for actual path search with collision checking
          auto precomputed_heuristic = PrecomputedCostHeuristic<GraphT, GraphEdgeCost>(this);
          shortest_path_geometric =
          computeShortestPath<PrecomputedCostHeuristic<GraphT, GraphEdgeCost>>(
            g_geometric_, weightmap_geometric, precomputed_heuristic, start_vertex_descriptor,
            goal_vertex_descriptor,
            false, true);
        }
      });

    // Forward Control graph Thread
    forward_control_thread_ = new std::thread(
      [this,
      &samples,
      &goal_state,
      &forward_control_g_target,
      &forward_control_g_root,
      &weightmap_forward_control,
      &shortest_path_forward_control
      ]
      {
        expandControlGraph(
          samples,
          goal_state,
          forward_control_g_target,
          g_forward_control_,
          forward_control_nn_,
          weightmap_forward_control);

        // Run A* for control graph
        auto control_forward_heuristic =
        GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
          this, &g_forward_control_[forward_control_g_target], true, false);
        shortest_path_forward_control =
        computeShortestPath<GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>>(
          g_forward_control_, weightmap_forward_control, control_forward_heuristic,
          forward_control_g_root, forward_control_g_target, false, false);
      });

    // Backward Control graph Thread
    backward_control_thread_ = new std::thread(
      [this,
      &samples,
      &start_state,
      &backward_control_g_target,
      &backward_control_g_root,
      &weightmap_backward_control,
      &shortest_path_backward_control
      ]
      {
        expandControlGraph(
          samples,
          start_state,
          backward_control_g_target,
          g_backward_control_,
          backward_control_nn_,
          weightmap_backward_control);

        // Run A* for control graph
        auto control_backward_heuristic =
        GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
          this, &g_backward_control_[backward_control_g_target], false, true);
        shortest_path_backward_control =
        computeShortestPath<GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>>(
          g_backward_control_, weightmap_backward_control, control_backward_heuristic,
          backward_control_g_root, backward_control_g_target, false, false);
      });

    // Let the threads finish
    geometric_thread_->join();
    forward_control_thread_->join();
    backward_control_thread_->join();

    // Delete the threads
    delete geometric_thread_;
    delete forward_control_thread_;
    delete backward_control_thread_;

    // Popolate the OMPL paths from vertexes found by A*
    populateOmplPathfromVertexPath(
      shortest_path_forward_control, g_forward_control_, forward_control_path);
    populateOmplPathfromVertexPath(
      shortest_path_backward_control, g_backward_control_, backward_control_path);

    // Check if we have a solution from either backward or forward control graph
    if (shortest_path_forward_control.size() > 0 || shortest_path_backward_control.size() > 0) {
      goal_reached = true;
    }

    // Also populate the geometric path from vertexes found by A*
    geometric_path = std::make_shared<PathControl>(si_);
    int index = 0;
    for (auto && i : shortest_path_geometric) {
      geometric_path->append(g_geometric_[i].state);
      index++;
    }
    // Determine best control path
    auto best_control_path(std::make_shared<PathControl>(si_));

    if (forward_control_path->length() > distanceFunction(start_state, goal_state)  ) {
      best_control_path = forward_control_path;
      if (backward_control_path->length() > distanceFunction(start_state, goal_state) ) {
        // Both paths are valid
        if (forward_control_path->length() < backward_control_path->length()) {
          best_control_path = forward_control_path;
          OMPL_INFORM(
            "%s: Better path in forward graph with %.2f length.\n",
            getName().c_str(), forward_control_path->length());
        } else {
          best_control_path = backward_control_path;
          OMPL_INFORM(
            "%s: Better path in backward graph with %.2f length.\n",
            getName().c_str(), backward_control_path->length());
        }
      }
    }

    // Set the cost of the solution.
    if (best_control_path->length() > distanceFunction(start_state, goal_state)) {

      // This solution is valid
      auto current_solution_cost = ompl::base::Cost(best_control_path->length());
      if (opt_->isCostBetterThan(current_solution_cost, bestCost_)) {
        bestCost_ = current_solution_cost;
        bestPath_ = best_control_path;
      }
      // Reset Control Graph
      g_forward_control_.clear();
      g_forward_control_ = GraphT();
      forward_control_nn_->clear();
      g_backward_control_.clear();
      g_backward_control_ = GraphT();
      backward_control_nn_->clear();

      // Add the start and goal vertex to the control graph
      forward_control_nn_->add(start_vertex_);
      backward_control_nn_->add(goal_vertex_);

      // Add goal and start to forward control graph
      forward_control_g_root = boost::add_vertex(g_forward_control_);
      forward_control_g_target = boost::add_vertex(g_forward_control_);
      g_forward_control_[forward_control_g_root] = *start_vertex_;
      g_forward_control_[forward_control_g_root].id = forward_control_g_root;
      g_forward_control_[forward_control_g_target] = *goal_vertex_;
      g_forward_control_[forward_control_g_target].id = forward_control_g_target;

      // Add goal and start to backward control graph
      backward_control_g_root = boost::add_vertex(g_backward_control_);
      backward_control_g_target = boost::add_vertex(g_backward_control_);
      g_backward_control_[backward_control_g_root] = *goal_vertex_;   // rooted in goal, aims for start
      g_backward_control_[backward_control_g_root].id = backward_control_g_root;   // rooted in goal, aims for start
      g_backward_control_[backward_control_g_target] = *start_vertex_;   // rooted in goal, aims for start
      g_backward_control_[backward_control_g_target].id = backward_control_g_target;   // rooted in goal, aims for start

    }

    OMPL_INFORM(
      "%s: Advancing with %d vertices and %d edges.\n",
      getName().c_str(), boost::num_vertices(g_geometric_), boost::num_edges(g_geometric_));

    OMPL_INFORM(
      "%s: For this iteration auto calculated radius_ was %.2f and numNeighbors_ %d, bestPath_ cost %.2f.\n",
      getName().c_str(), radius_, numNeighbors_, bestCost_.value());

    std::string red("red");
    std::string green("green");
    std::string blue("blue");

    /*// geometric path
    visualizePath(
      g_geometric_,
      shortest_path_geometric,
      geometric_path_pub_,
      "g",
      getColor(green));

    // best control path
    visualizePath(
      bestPath_,
      control_path_pub_,
      "c",
      getColor(red));

    visualizeRGG(
      g_geometric_,
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
      forward_control_g_target);*/

  }

  pdef_->addSolutionPath(bestPath_, false, 0.0, getName());

  clear();

  return {goal_reached, false};
}

void ompl::control::AITStarKin::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);
  std::vector<VertexProperty *> Nodes;
  std::vector<VertexProperty *> allNodes;
  if (geometric_nn_) {
    geometric_nn_->list(Nodes);
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

  auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();

  if (use_k_nearest_) {
    numNeighbors_ = computeNumberOfNeighbors(
      numSamplesInInformedSet + samples.size() /*- 2 goal and start */);
  } else {
    radius_ = computeConnectionRadius(
      numSamplesInInformedSet + samples.size() /*- 2 goal and start*/);
  }

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

    if constexpr (use_k_nearest_) {
      geometric_nn->nearestK(this_vertex_property, numNeighbors_, nbh);
    } else {
      geometric_nn->nearestR(this_vertex_property, radius_, nbh);
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
        if (u == v || dist > max_edge_length_) {
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

  if constexpr (use_k_nearest_) {
    geometric_nn->nearestK(target_vertex_property, numNeighbors_, goal_nbh);
  } else {
    geometric_nn->nearestR(target_vertex_property, radius_, goal_nbh);
  }

  if (goal_nbh.size() > max_neighbors_) {
    goal_nbh.resize(max_neighbors_);
  }
  for (auto && nb : goal_nbh) {
    vertex_descriptor u = target_vertex_property->id;
    vertex_descriptor v = nb->id;
    double dist = distanceFunction(geometric_graph[u].state, geometric_graph[v].state);
    edge_descriptor e; bool edge_added;
    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || dist > max_edge_length_) {
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

    if constexpr (use_k_nearest_) {
      control_nn->nearestK(this_vertex_property, numNeighbors_, nbh);
    } else {
      control_nn->nearestR(this_vertex_property, radius_, nbh);
    }

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
        if (u == v || dist > max_edge_length_) {
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
        if (dist_to_target < max_edge_length_) {

          vertex_descriptor u = arrived_vertex_property->id;
          vertex_descriptor v = target_vertex_descriptor;
          double dist = distanceFunction(control_graph[u].state, control_graph[v].state);
          edge_descriptor e; bool edge_added;
          // not to construct edges with self, and if nbh is further than radius_, continue
          if (u == v || dist > max_edge_length_) {
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

std::size_t ompl::control::AITStarKin::computeNumberOfSamplesInInformedSet() const
{
  // Loop over all vertices and count the ones in the informed set.
  std::size_t numberOfSamplesInInformedSet{0u};
  for (auto vd : boost::make_iterator_range(vertices(g_forward_control_))) {

    auto vertex = g_forward_control_[vd].state;
    // Get the best cost to come from any start.
    auto costToCome = opt_->infiniteCost();
    costToCome = opt_->betterCost(
      costToCome, opt_->motionCostHeuristic(start_vertex_->state, vertex));

    // Get the best cost to go to any goal.
    auto costToGo = opt_->infiniteCost();
    costToGo = opt_->betterCost(
      costToCome, opt_->motionCostHeuristic(vertex, goal_vertex_->state));

    // If this can possibly improve the current solution, it is in the informed set.
    if (opt_->isCostBetterThan(
        opt_->combineCosts(costToCome, costToGo),
        bestCost_))
    {
      ++numberOfSamplesInInformedSet;
    }
  }

  return numberOfSamplesInInformedSet;
}

double ompl::control::AITStarKin::computeConnectionRadius(std::size_t numSamples) const
{
  // Define the dimension as a helper variable.
  auto dimension = static_cast<double>(si_->getStateDimension());

  // Compute the RRT* factor.
  return
    rewire_factor_ * std::pow(
    2.0 * (1.0 + 1.0 / dimension) *
    (rejection_informed_sampler_->getInformedMeasure(bestCost_) /
    unitNBallMeasure(si_->getStateDimension())) *
    (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
    1.0 / dimension);
}

std::size_t ompl::control::AITStarKin::computeNumberOfNeighbors(std::size_t numSamples) const
{
  return std::ceil(rewire_factor_ * k_rgg_ * std::log(static_cast<double>(numSamples)));
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
    /*const auto * target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
    const auto * target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);*/
    const auto * target_cstate = g[vd].state->as<ompl::base::RealVectorStateSpace::StateType>();
    geometry_msgs::msg::Point point;
    point.x = target_cstate->values[0];
    point.y = target_cstate->values[1];
    point.z = target_cstate->values[2];
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

    const auto * source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
    source_point.x = source_cstate->values[0];
    source_point.y = source_cstate->values[1];
    source_point.z = source_cstate->values[2];

    const auto * target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
    target_point.x = target_cstate->values[0];
    target_point.y = target_cstate->values[1];
    target_point.z = target_cstate->values[2];

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

    const auto * source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
    source_point.x = source_cstate->values[0];
    source_point.y = source_cstate->values[1];
    source_point.z = source_cstate->values[2];

    const auto * target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
    target_point.x = target_cstate->values[0];
    target_point.y = target_cstate->values[1];
    target_point.z = target_cstate->values[2];

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

void ompl::control::AITStarKin::visualizePath(
  const std::shared_ptr<PathControl> & path,
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

  for (size_t i = 1; i < path->getStateCount(); i++) {

    auto u = path->getState(i - 1);
    auto v = path->getState(i);

    geometry_msgs::msg::Point source_point, target_point;

    const auto * source_cstate = u->as<ompl::base::RealVectorStateSpace::StateType>();
    source_point.x = source_cstate->values[0];
    source_point.y = source_cstate->values[1];
    source_point.z = source_cstate->values[2];

    const auto * target_cstate = v->as<ompl::base::RealVectorStateSpace::StateType>();
    target_point.x = target_cstate->values[0];
    target_point.y = target_cstate->values[1];
    target_point.z = target_cstate->values[2];

    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(color);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = ns + "costs";
    text.id = i - 1;   //g[u].id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.lifetime = rclcpp::Duration::from_seconds(0);
    text.text = std::to_string(i - 1);
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
