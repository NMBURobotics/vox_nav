// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_planning/native_planners/KinoPlanner.hpp"

ompl::control::KinoPlanner::KinoPlanner(const SpaceInformationPtr& si) : base::Planner(si, "KinoPlanner")
{
  // set planner specs
  specs_.approximateSolutions = true;
  specs_.optimizingPaths = true;
  specs_.multithreaded = false;
  specs_.canReportIntermediateSolutions = true;
  specs_.approximateSolutions = false;

  siC_ = si.get();

  // declare configuration parameters, the user can use these parameters to tune the planner
  // Batch size for each thread, we generate this many samples in each thread at each iteration
  declareParam<int>("batch_size", this, &KinoPlanner::setBatchSize, &KinoPlanner::getBatchSize, "10:100:1000");
}

ompl::control::KinoPlanner::~KinoPlanner()
{
  freeMemory();
}

void ompl::control::KinoPlanner::setup()
{
  base::Planner::setup();

  // If we are not in Eucledian space, then set the max edge distance to 1/10th of the maximum extent of the state space
  if (si_->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR)
  {
    vertexes_mean_ = Eigen::VectorXd::Zero(si_->getStateSpace()->getDimension());
    vertexes_covariance_ =
        Eigen::MatrixXd::Zero(si_->getStateSpace()->getDimension(), si_->getStateSpace()->getDimension());
  }
  else if (si_->getStateSpace()->getType() == base::STATE_SPACE_SE3)
  {
    // only positions are considered
    vertexes_mean_ = Eigen::VectorXd::Zero(3);
    vertexes_covariance_ = Eigen::MatrixXd::Zero(3, 3);
  }
  else if (si_->getStateSpace()->getType() == base::STATE_SPACE_SE2)
  {
    // only positions are considered
    vertexes_mean_ = Eigen::VectorXd::Zero(2);
    vertexes_covariance_ = Eigen::MatrixXd::Zero(2, 2);
  }
  else if (si_->getStateSpace()->getType() == base::STATE_SPACE_REEDS_SHEPP)
  {
    // only positions are considered
    vertexes_mean_ = Eigen::VectorXd::Zero(2);
    vertexes_covariance_ = Eigen::MatrixXd::Zero(2, 2);
  }
  else
  {
    OMPL_ERROR("%s: State space type is not supported", getName().c_str());
  }

  // reset nn structure
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>> this_nn;
  this_nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty*>(this));
  nnControl_ = this_nn;

  nnControl_->setDistanceFunction(
      [this](const VertexProperty* a, const VertexProperty* b) { return distanceFunction(a, b); });

  // set a default optimization objective if one is not set
  if (pdef_)
  {
    if (pdef_->hasOptimizationObjective())
    {
      opt_ = pdef_->getOptimizationObjective();
    }
    else
    {
      OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
      opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
      pdef_->setOptimizationObjective(opt_);
    }
  }

  // set   samplers
  if (!controlSampler_)
  {
    controlSampler_ = siC_->allocControlSampler();
  }

  // RVIZ VISUALIZATIONS, this is likely to be removed in the future, but for now it is useful
  node_ = std::make_shared<rclcpp::Node>("KinoPlanner_rclcpp_node");
  rgg_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/KinoPlanner/rgg",
                                                                                 rclcpp::SystemDefaultsQoS());
  geometric_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/KinoPlanner/g_plan",
                                                                                      rclcpp::SystemDefaultsQoS());
  first_control_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "vox_nav/KinoPlanner/first_control_rgg", rclcpp::SystemDefaultsQoS());
  second_control_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "vox_nav/KinoPlanner/second_control_rgg", rclcpp::SystemDefaultsQoS());
  control_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/KinoPlanner/c_plan",
                                                                                    rclcpp::SystemDefaultsQoS());
}

void ompl::control::KinoPlanner::clear()
{
  Planner::clear();
  controlSampler_.reset();

  freeMemory();

  // clear the nn for geometric
  nnControl_->clear();
}

void ompl::control::KinoPlanner::freeMemory()
{
}

double ompl::control::KinoPlanner::distanceFunction(const VertexProperty* a, const VertexProperty* b) const
{
  return si_->distance(a->state, b->state);
}

double ompl::control::KinoPlanner::distanceFunction(const base::State* a, const base::State* b) const
{
  return si_->distance(a, b);
}

void ompl::control::KinoPlanner::setBatchSize(int batch_size)
{
  params_.batch_size_ = batch_size;
}

int ompl::control::KinoPlanner::getBatchSize() const
{
  return params_.batch_size_;
}

ompl::base::PlannerStatus ompl::control::KinoPlanner::solve(const base::PlannerTerminationCondition& ptc)
{
  // check if the problem is setup properly
  checkValidity();

  // get the goal node and state
  auto* goal_state = si_->allocState();
  auto* start_state = si_->allocState();
  // get start node and state
  while (const base::State* st = pis_.nextStart())
  {
    si_->copyState(start_state, st);
  }
  while (const base::State* goal = pis_.nextGoal())
  {
    si_->copyState(goal_state, goal);
  }

  if (!si_->getStateValidityChecker()->isValid(start_state))
  {
    OMPL_ERROR("%s: Start state is not valid", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }
  if (!si_->getStateValidityChecker()->isValid(goal_state))
  {
    OMPL_ERROR("%s: Goal state is not valid", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  // allocate the start and goal vertex properties
  startVertex_ = new VertexProperty();
  goalVertex_ = new VertexProperty();
  // Now set the start and goal vertex properties and push them to the nn of all threads
  // NOTE: In gemetric problem setting the we are always solving from start to goal
  startVertex_->state = start_state;
  startVertex_->is_root = true;
  goalVertex_->state = goal_state;

  // the controls in start and goal vertexes are zeros
  startVertex_->control = siC_->allocControl();
  goalVertex_->control = siC_->allocControl();

  // Add the start nnControl
  nnControl_->add(startVertex_);
  // Keep the goal vertex out at the moment, we will check if it is connected to the graph later

  // inform with number of vertexes in nnControl
  OMPL_INFORM("%s: Starting planning with %d vertexes in nnControl", getName().c_str(), nnControl_->size());

  // initially we have no solution and no approximate solution
  bool exact_solution{ false };
  bool approximate_solution{ false };

  // march forward until we have a solution or we run out of time
  while (ptc == false)
  {
    // get all vertexes in nnControl
    std::vector<VertexProperty*> vertexes;
    nnControl_->list(vertexes);

    // Calculate current mean and covariance of the vertexes in nnControl
    Eigen::MatrixXd xy_matrix(2, nnControl_->size());
    int counter{ 0 };
    for (auto&& v : vertexes)
    {
      double x = v->state->as<ompl::base::SE2StateSpace::StateType>()->getX();
      double y = v->state->as<ompl::base::SE2StateSpace::StateType>()->getY();
      xy_matrix(0, counter) = x;
      xy_matrix(1, counter) = y;
      counter++;
    }
    Eigen::VectorXd mean = xy_matrix.rowwise().mean();
    xy_matrix.transposeInPlace();
    Eigen::MatrixXd centered = xy_matrix.rowwise() - mean.colwise().mean();
    Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(xy_matrix.rows() - 1);

    // select a random vertex from nnControl
    VertexProperty* random_vertex = vertexes[rng_.uniformInt(0, vertexes.size() - 1)];
    ompl::base::State* new_state = si_->allocState();

    // allocate a random batch of control samples
    std::vector<ompl::control::Control*> samples;
    samples.reserve(params_.batch_size_);
    for (int i = 0; i < params_.batch_size_; i++)
    {
      samples.push_back(siC_->allocControl());
    }

    // sample the controls
    for (auto&& i : samples)
    {
      controlSampler_->sampleNext(i, random_vertex->control);
    }

    // Attempt to drive from random vertex with the sampled control
    // Do not maoify the random vertex, instead modify the new_state
    for (auto&& i : samples)
    {
      auto duration = siC_->propagateWhileValid(random_vertex->state, i, siC_->getMinControlDuration(), new_state);

      if (nnControl_->size() < 2)
      {
        // Add the state anyway, we need at least two vertexes to compute the mean and covariance
        // of the vertexes in nnControl
        VertexProperty* vertex_property = new VertexProperty();
        vertex_property->state = new_state;
        vertex_property->control = i;
        vertex_property->control_duration = duration;
        nnControl_->add(vertex_property);
        break;
      }

      // Check if the new state expands the nnControl in terms of mean and covariance
      double x = new_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
      double y = new_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
      Eigen::VectorXd new_state_vector(2);
      new_state_vector << x, y;
      double mahalanobis_distance = (new_state_vector - mean).transpose() * cov.inverse() *
                                    (new_state_vector - mean);  // mahalanobis distance of the new state
                                    
    }
  }

  // Add the best path to the solution path
  // clear data structures
  clear();

  // report the solution status
  return { exact_solution, approximate_solution };
}

void ompl::control::KinoPlanner::getPlannerData(base::PlannerData& data) const
{
  Planner::getPlannerData(data);
}

void ompl::control::KinoPlanner::expandGeometricGraph(
    const std::vector<ompl::base::State*>& samples, const base::PlannerTerminationCondition& ptc,
    GraphT& geometric_graph, std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& geometric_nn,
    WeightMap& geometric_weightmap)
{
  // Add batch_size_ number of samples to graphs
  // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
  // But too close vertices will be discarded in order for memory not to sink
  for (auto&& i : samples)
  {
    // if the planner is termination condition is met, break
    if (ptc == true)
    {
      break;
    }

    // If the sampled state is not valid, discard it and continue
    if (!si_->getStateValidityChecker()->isValid(i))
    {
      continue;
    }

    // We will potentially add a new vertex to the graph and nn structure, so allocate a vertex property
    VertexProperty* vertex_property_to_be_added = new VertexProperty();
    vertex_property_to_be_added->state = (i);

    // Get the neighbors of the new vertex with radius_ or k-nearest
    std::vector<ompl::control::KinoPlanner::VertexProperty*> nbh;
    if (params_.use_k_nearest_)
    {
      geometric_nn->nearestK(vertex_property_to_be_added, numNeighbors_, nbh);
    }
    else
    {
      geometric_nn->nearestR(vertex_property_to_be_added, radius_, nbh);
    }

    // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
    // So the further neighbors are discarded
    if (nbh.size() > params_.max_neighbors_)
    {
      nbh.resize(params_.max_neighbors_);
    }

    // Check if the new vertex is too close to an existing vertex
    bool does_vertice_exits{ false };
    for (auto&& nb : nbh)
    {
      double dist = distanceFunction(i, nb->state);
      if (dist < params_.min_dist_between_vertices_)
      {
        // ops, a very close vertex already exists in graph and nn, do not add due to computational burden*/
        does_vertice_exits = true;
      }
    }

    if (!does_vertice_exits)
    {
      // Add the new vertex to the graph and nn structure
      vertex_descriptor vertex_descriptor_to_be_added = boost::add_vertex(geometric_graph);
      vertex_property_to_be_added->id = vertex_descriptor_to_be_added;
      geometric_graph[vertex_descriptor_to_be_added] = *vertex_property_to_be_added;
      geometric_nn->add(vertex_property_to_be_added);
      for (auto&& nb : nbh)
      {
        if (!si_->checkMotion(vertex_property_to_be_added->state, nb->state))
        {
          continue;
        }

        vertex_descriptor u = vertex_descriptor_to_be_added;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(vertex_property_to_be_added->state, nb->state);

        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > params_.max_dist_between_vertices_)
        {
          continue;
        }
        if (boost::edge(u, v, geometric_graph).second ||  // if edge already exists, continue
            boost::edge(v, u, geometric_graph).second)
        {
          continue;
        }
        // Once suitable edges are found, populate them over graphs
        edge_descriptor e;
        bool edge_added;
        boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
        geometric_weightmap[e] = opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
      }
    }
  }
}

void ompl::control::KinoPlanner::ensureGeometricGoalVertexConnectivity(
    VertexProperty* target_vertex_property, GraphT& geometric_graph,
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& geometric_nn, WeightMap& geometric_weightmap)
{
  // get neihbors of the target vertex
  std::vector<ompl::control::KinoPlanner::VertexProperty*> goal_nbh;
  if (params_.use_k_nearest_)
  {
    geometric_nn->nearestK(target_vertex_property, numNeighbors_, goal_nbh);
  }
  else
  {
    geometric_nn->nearestR(target_vertex_property, radius_, goal_nbh);
  }

  // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
  if (goal_nbh.size() > params_.max_neighbors_)
  {
    goal_nbh.resize(params_.max_neighbors_);
  }

  // iterate over the neighbors and add edges to the target vertex if they are valid
  // and satisfy the distance criteria
  for (auto&& nb : goal_nbh)
  {
    if (!si_->checkMotion(nb->state, target_vertex_property->state))
    {
      // Do not add edge to target vertex as it is not valid
      continue;
    }

    vertex_descriptor u = target_vertex_property->id;
    vertex_descriptor v = nb->id;
    double dist = distanceFunction(target_vertex_property->state, nb->state);

    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || (dist > params_.max_dist_between_vertices_))
    {
      continue;
    }
    if (boost::edge(u, v, geometric_graph).second || boost::edge(v, u, geometric_graph).second)
    {
      continue;
    }

    // Connect the target vertex to the neighbor as it is valid and satisfies the distance criterias
    edge_descriptor e;
    bool edge_added;
    boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
    geometric_weightmap[e] = opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
  }
}

void ompl::control::KinoPlanner::expandControlGraph(
    const std::vector<ompl::base::State*>& samples, const ompl::base::State* target_vertex_state,
    const vertex_descriptor& target_vertex_descriptor, const base::PlannerTerminationCondition& ptc,
    const GraphT* connection_control_graph,
    const std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>> connection_control_nn, GraphT& control_graph,
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& control_nn, WeightMap& control_weightmap)
{
  // Every ith sample is goal biasing sampling
  int ith_sample = static_cast<int>(1.0 / params_.goal_bias_);
  int goal_bias_counter{ 0 };

  // Iterate over the samples and try to add them to the control graph
  for (auto&& i : samples)
  {
    // Check if the planner has been terminated
    if (ptc == true)
    {
      break;
    }

    // If the sampled state is not valid, discard it and continue
    if (!si_->getStateValidityChecker()->isValid(i))
    {
      continue;
    }

    VertexProperty* vertex_property = new VertexProperty();
    vertex_property->state = i;

    // Every ith sample is goal biasing
    // So if this is the ith sample, then the sample is the goal state
    if ((goal_bias_counter & ith_sample) == 0)
    {
      auto deep_copy_target_state = si_->allocState();
      si_->copyState(deep_copy_target_state, target_vertex_state);
      vertex_property->state = deep_copy_target_state;
    }
    // increment the goal bias counter
    goal_bias_counter++;

    // Get the nearest neighbors of the sample with numNeighbors_ or radius_
    std::vector<ompl::control::KinoPlanner::VertexProperty*> nbh;
    if (params_.use_k_nearest_)
    {
      control_nn->nearestK(vertex_property, numNeighbors_, nbh);
    }
    else
    {
      control_nn->nearestR(vertex_property, radius_, nbh);
    }

    // if No neighbors found, continue
    if (nbh.size() == 0)
    {
      continue;
    }

    // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
    if (nbh.size() > params_.max_neighbors_)
    {
      nbh.resize(params_.max_neighbors_);
    }

    // Check if sample us unique enough
    bool does_vertice_exits{ false };
    for (auto&& nb : nbh)
    {
      double dist = distanceFunction(i, nb->state);
      if (dist < params_.min_dist_between_vertices_)
      {
        // ops this sample is not unique enough, so do not add it to the graph as it increases the computation time
        does_vertice_exits = true;
      }
    }

    if (!does_vertice_exits)
    {
      // Now this sample is unique enough,  we will try to propogate from it's neighbors to it
      for (auto&& nb : nbh)
      {
        // If the neighbor is goal vertex, do not "propogate" from goal to sample,
        // the ultmate goal is to propogate from sample to goal
        if (nb->id == target_vertex_descriptor)
        {
          continue;
        }

        // if neighbor is blacklisted, do not propogate from it and continue
        if (nb->blacklisted)
        {
          continue;
        }

        // Do not modify original sample
        auto deep_copy_sample_state = si_->allocState();
        si_->copyState(deep_copy_sample_state, i);
        // Attempt to drive from neighbour towards newly added sample
        // modify the sample to latest arrived valid state, is it guaranteed to be valid.
        auto c = siC_->allocControl();
        auto duration = directedControlSampler_->sampleTo(c, nb->state, deep_copy_sample_state);

        // if the duration is less than min_control_duration, then probably it hit the boundary
        // or the sample is invalid, so do not add it to the graph
        if (duration < siC_->getMinControlDuration() || !si_->isValid(deep_copy_sample_state))
        {
          // we cannot proceed free the allocated control and state
          siC_->freeControl(c);
          si_->freeState(deep_copy_sample_state);
          // blacklist the neighbor, this is not %100 correct, but otherwise the graph get filled with
          // useless vertices that are stuck on boundaries
          nb->blacklisted = true;
          continue;
        }

        // As result of sampleTo, the sample is now valid, so add it to the graph and nn structure
        vertex_descriptor arrived_vertex_descriptor = boost::add_vertex(control_graph);
        auto arrived_vertex_property = new VertexProperty();
        arrived_vertex_property->id = arrived_vertex_descriptor;
        arrived_vertex_property->state = deep_copy_sample_state;
        arrived_vertex_property->parent_id = nb->id;
        control_graph[arrived_vertex_descriptor] = *arrived_vertex_property;
        control_graph[arrived_vertex_descriptor].state = deep_copy_sample_state;
        control_graph[arrived_vertex_descriptor].control = c;
        control_graph[arrived_vertex_descriptor].control_duration = duration;
        control_graph[arrived_vertex_descriptor].parent_id = nb->id;

        // lock the nn structure as some other thread might be accessing it
        std::lock_guard<std::mutex> guard(nnMutex_);
        control_nn->add(arrived_vertex_property);

        // Also add edge from neighbor to arrived vertex
        vertex_descriptor u = arrived_vertex_descriptor;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(arrived_vertex_property->state, nb->state);

        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > params_.max_dist_between_vertices_)
        {
          continue;
        }
        if (boost::edge(u, v, control_graph).second || boost::edge(v, u, control_graph).second)
        {
          continue;
        }

        edge_descriptor e;
        bool edge_added;
        boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
        control_weightmap[e] = opt_->motionCost(control_graph[u].state, control_graph[v].state).value();

        // Now check if the arrived vertex connects to any vertex from connection_graph
        VertexProperty* nbh_in_connection_graph = nullptr;
        try
        {
          nbh_in_connection_graph = connection_control_nn->nearest(arrived_vertex_property);
        }
        catch (...)
        {
          OMPL_WARN("%s: No nn available to establish a connection from connection_graph", getName().c_str());
          continue;
        }

        if (nbh_in_connection_graph)
        {
          // We have a neighbor in connection graph, check if it is close enough to connect

          if (distanceFunction(nbh_in_connection_graph, arrived_vertex_property) < params_.min_dist_between_vertices_)
          {
            // arrived vertex is close to enough to it's nn in connection graph
            // establish connection from arrived vertex from all the way to root of connection_graph,
            // ultimately giving us a path from start to goal
            auto connection_vertex = nbh_in_connection_graph->id;
            auto connection_vertex_parent = arrived_vertex_descriptor;

            // spin until we get Get to the root of the connection_graph which is the goal vertex
            while ((*connection_control_graph)[connection_vertex].is_root == false)
            {
              auto vertex_to_be_added = boost::add_vertex(control_graph);
              auto vertex_prop_to_be_added = new VertexProperty();
              vertex_prop_to_be_added->id = vertex_to_be_added;
              vertex_prop_to_be_added->parent_id = connection_vertex_parent;
              vertex_prop_to_be_added->state = si_->allocState();

              // if the state is nullptr, then we have reached the root of the connection_graph
              if ((*connection_control_graph)[connection_vertex].state == nullptr)
              {
                break;
              }

              // Add the connection vertex and all of its parents to the control_graph and nn structure
              si_->copyState(vertex_prop_to_be_added->state, (*connection_control_graph)[connection_vertex].state);
              control_graph[vertex_to_be_added] = *vertex_prop_to_be_added;
              control_nn->add(vertex_prop_to_be_added);

              // Also add edges from the parent to the vertex
              edge_descriptor e;
              bool edge_added;
              boost::tie(e, edge_added) = boost::add_edge(connection_vertex_parent, vertex_to_be_added, control_graph);
              control_weightmap[e] = opt_->motionCost(control_graph[connection_vertex_parent].state,
                                                      control_graph[vertex_to_be_added].state)
                                         .value();

              // NOTE: The path from connection vertex to root of connection graph is reversed
              // We have to deal with this when we are trying to extract the path from the control_graph
              connection_vertex = (*connection_control_graph)[connection_vertex].parent_id;
              connection_vertex_parent = vertex_to_be_added;
            }
          }
        }
      }
    }
  }
}

void ompl::control::KinoPlanner::ensureControlGoalVertexConnectivity(
    const ompl::base::State* target_vertex_state, const vertex_descriptor& target_vertex_descriptor,
    GraphT& control_graph, std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& control_nn,
    WeightMap& control_weightmap, int& status)
{
  // Get all the neighbors of the target vertex
  std::vector<ompl::control::KinoPlanner::VertexProperty*> nbh;
  if (params_.use_k_nearest_)
  {
    control_nn->nearestK(&control_graph[target_vertex_descriptor], 2.0 * numNeighbors_, nbh);
  }
  else
  {
    control_nn->nearestR(&control_graph[target_vertex_descriptor], radius_, nbh);
  }

  // Try to drive from all neighbors to the target vertex
  for (auto&& nb : nbh)
  {
    if (nb->id == target_vertex_descriptor)
    {
      // Do not add edge to self
      continue;
    }

    // Check if the motion is valid
    if (!si_->checkMotion(nb->state, control_graph[target_vertex_descriptor].state))
    {
      continue;
    }

    // Do not modify original sample
    auto deep_copy_sample_state = si_->allocState();
    si_->copyState(deep_copy_sample_state, control_graph[target_vertex_descriptor].state);
    // Attempt to drive towards newly added sample
    // Modify the sample to latest arrived state
    auto c = siC_->allocControl();
    auto duration = directedControlSampler_->sampleTo(c, nb->state, deep_copy_sample_state);

    if (duration < siC_->getMinControlDuration() || !si_->isValid(deep_copy_sample_state))
    {
      // perhaps due to invalidity of the state, we cannot proceed
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
    edge_descriptor e;
    bool edge_added;
    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || dist > params_.max_dist_between_vertices_)
    {
      continue;
    }
    if (boost::edge(u, v, control_graph).second || boost::edge(v, u, control_graph).second)
    {
      continue;
    }
    // Once suitable edges are found, populate them over graphs
    boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
    control_weightmap[e] = opt_->motionCost(control_graph[u].state, control_graph[v].state).value();

    // calculate the distance between the arrived state and the start/goal
    base::Goal* goal = pdef_->getGoal().get();
    auto* goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    double goal_tolerance = goal_s->getThreshold();
    double dist_to_target_approx = distanceFunction(arrived_vertex_property->state, target_vertex_state);
    if (dist_to_target_approx < goal_tolerance)
    {
      // if the distance is less than the radius to target, then add the edge
      // Exact solution
      vertex_descriptor u = arrived_vertex_property->id;
      vertex_descriptor v = target_vertex_descriptor;
      double dist = distanceFunction(control_graph[u].state, control_graph[v].state);
      edge_descriptor e;
      bool edge_added;
      // not to construct edges with self, and if nbh is further than radius_, continue
      if (u == v || dist > params_.max_dist_between_vertices_)
      {
        continue;
      }
      if (boost::edge(u, v, control_graph).second || boost::edge(v, u, control_graph).second)
      {
        continue;
      }
      // Once suitable edges are found, populate them over graphs
      boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
      control_weightmap[e] = opt_->motionCost(control_graph[u].state, control_graph[v].state).value();
      status = base::PlannerStatus::EXACT_SOLUTION;
    }
  }
}

std::size_t ompl::control::KinoPlanner::computeNumberOfSamplesInInformedSet() const
{
  // Loop over all vertices and count the ones in the informed set.
  std::size_t numberOfSamplesInInformedSet{ 0u };
  for (auto vd : boost::make_iterator_range(vertices(graphControlThreads_[0])))
  {
    auto vertex = graphControlThreads_[0][vd].state;

    // Get the best cost to come from any start.
    auto costToCome = opt_->infiniteCost();
    costToCome = opt_->betterCost(costToCome, opt_->motionCostHeuristic(startVertexGeometric_->state, vertex));

    // Get the best cost to go to any goal.
    auto costToGo = opt_->infiniteCost();
    costToGo = opt_->betterCost(costToCome, opt_->motionCostHeuristic(vertex, goalVertexGeometric_->state));

    // If this can possibly improve the current solution, it is in the informed set.
    if (opt_->isCostBetterThan(opt_->combineCosts(costToCome, costToGo), bestControlCost_))
    {
      ++numberOfSamplesInInformedSet;
    }
  }

  return numberOfSamplesInInformedSet;
}

double ompl::control::KinoPlanner::computeConnectionRadius(std::size_t numSamples) const
{
  // Define the dimension as a helper variable.
  auto dimension = static_cast<double>(si_->getStateDimension());

  // Compute the RRT* factor. Taken from AITStar::computeConnectionRadius.
  return params_.rewire_factor_ *
         std::pow(2.0 * (1.0 + 1.0 / dimension) *
                      (rejectionInformedSampler_->getInformedMeasure(bestControlCost_) /
                       unitNBallMeasure(si_->getStateDimension())) *
                      (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
                  1.0 / dimension);
}

std::size_t ompl::control::KinoPlanner::computeNumberOfNeighbors(std::size_t numSamples) const
{
  // Compute the RGG factor. Taken from AITStar::computeNumberOfNeighbors.
  return std::ceil(params_.rewire_factor_ * kRGG_ * std::log(static_cast<double>(numSamples)));
}

void ompl::control::KinoPlanner::populateOmplPathfromVertexPath(const std::list<vertex_descriptor>& vertex_path,
                                                                GraphT& g, WeightMap& weightmap,
                                                                std::shared_ptr<ompl::control::PathControl>& path,
                                                                std::vector<VertexProperty*>& vertexprop_path,
                                                                const bool control) const
{
  path = std::make_shared<PathControl>(si_);
  int index{ 0 };
  vertex_descriptor prev_vertex = vertex_path.front();
  for (auto&& i : vertex_path)
  {
    // Use this opportunity to mark the edges as invalid if they were blacklisted
    if (g[i].blacklisted)
    {
      for (auto ed : boost::make_iterator_range(boost::out_edges(i, g)))
      {
        weightmap[ed] = opt_->infiniteCost().value();
      }
    }
    if (index > 0)
    {
      bool is_edge_valid = si_->checkMotion(g[prev_vertex].state, g[i].state);
      if (!is_edge_valid)
      {
        // Mark the edge as invalid
        auto ed = boost::edge(prev_vertex, i, g);
        weightmap[ed.first] = opt_->infiniteCost().value();
      }
      prev_vertex = i;
    }
    if (g[i].control == nullptr && control)
    {
      // This most likely a start or goal vertex
      // The control has not been allocated for this vertex
      // Allocate a control and set it to zero
      g[i].control = siC_->allocControl();
    }
    if (index == 0)
    {
      path->append(g[i].state);
    }
    else
    {
      if (control)
      {
        path->append(g[i].state, g[i].control, g[i].control_duration * siC_->getPropagationStepSize());
      }
      else
      {
        path->append(g[i].state);
      }
    }
    vertexprop_path[index] = new VertexProperty(g[i]);
    index++;
  }
}

ompl::base::Cost ompl::control::KinoPlanner::computePathCost(std::shared_ptr<ompl::control::PathControl>& path) const
{
  ompl::base::Cost path_cost = opt_->identityCost();
  if (path->getStateCount() == 0)
  {
    return path_cost;
  }
  for (std::size_t i = 0; i < path->getStateCount() - 1; ++i)
  {
    path_cost = opt_->combineCosts(path_cost, opt_->motionCost(path->getState(i), path->getState(i + 1)));
  }
  return path_cost;
}

std::vector<const ompl::base::State*>
ompl::control::KinoPlanner::getConstStatesFromPath(const std::shared_ptr<ompl::control::PathControl>& path)
{
  std::vector<const ompl::base::State*> states;
  for (std::size_t i = 0; i < path->getStateCount(); ++i)
  {
    states.push_back(path->getState(i));
  }
  return states;
}

void ompl::control::KinoPlanner::clearControlGraphs(
    std::vector<WeightMap>& control_weightmaps,
    std::vector<std::pair<vertex_descriptor, vertex_descriptor>>& control_start_goal_descriptors)
{
  // Reset control graphs anyways
  control_start_goal_descriptors.clear();

  for (int i = 0; i < params_.num_threads_; i++)
  {
    graphControlThreads_[i].clear();
    graphControlThreads_[i] = GraphT();
    // free memory for all nns in control threads
    nnControlsThreads_[i]->clear();
    // Add the start and goal vertex to the control graph
    nnControlsThreads_[i]->add(startVerticesControl_[i]);

    vertex_descriptor control_g_root = boost::add_vertex(graphControlThreads_[i]);
    graphControlThreads_[i][control_g_root] = *startVerticesControl_[i];
    graphControlThreads_[i][control_g_root].id = control_g_root;
    graphControlThreads_[i][control_g_root].is_root = true;
    vertex_descriptor control_g_target = boost::add_vertex(graphControlThreads_[i]);
    graphControlThreads_[i][control_g_target] = *goalVerticesControl_[i];
    graphControlThreads_[i][control_g_target].id = control_g_target;

    control_start_goal_descriptors.push_back(
        std::make_pair(graphControlThreads_[i][control_g_root].id, graphControlThreads_[i][control_g_target].id));
  }
  // Reset the weightmap
  control_weightmaps.clear();
  for (auto& graph : graphControlThreads_)
  {
    control_weightmaps.push_back(get(boost::edge_weight, graph));
  }
}

void ompl::control::KinoPlanner::visualizeRGG(
    const GraphT& g, const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const std::string& ns, const std_msgs::msg::ColorRGBA& color, const vertex_descriptor& start_vertex,
    const vertex_descriptor& goal_vertex, const int& state_space_type)
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
  sphere.scale.x = 0.15;
  sphere.scale.y = 0.15;
  sphere.scale.z = 0.15;

  visualization_msgs::msg::MarkerArray marker_array;
  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap
  for (auto vd : boost::make_iterator_range(vertices(g)))
  {
    // Paint the start and goal vertices differently.
    std_msgs::msg::ColorRGBA color_vd = color;
    double is_goal_or_start{ 0.0 };
    if (g[vd].id == start_vertex || g[vd].id == goal_vertex)
    {
      color_vd.b *= 0.5;
    }

    geometry_msgs::msg::Point point;
    if (state_space_type == base::STATE_SPACE_REAL_VECTOR)
    {
      const auto* target_cstate = g[vd].state->as<ompl::base::RealVectorStateSpace::StateType>();
      point.x = target_cstate->values[0];
      point.y = target_cstate->values[1];
      point.z = target_cstate->values[2];
    }
    else
    {
      const auto* target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      point.x = target_xyzv->values[0];
      point.y = target_xyzv->values[1];
      point.z = target_xyzv->values[2];
    }

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
  line_strip.scale.x = 0.01;
  line_strip.scale.y = 0.01;
  line_strip.scale.z = 0.01;
  line_strip.color = color;

  for (auto eit = es.first; eit != es.second; ++eit)
  {
    u = boost::source(*eit, g);
    v = boost::target(*eit, g);

    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == base::STATE_SPACE_REAL_VECTOR)
    {
      const auto* source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto* target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    }
    else
    {
      const auto* source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* source_xyzv = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto* target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      target_point.x = target_xyzv->values[0];
      target_point.y = target_xyzv->values[1];
      target_point.z = target_xyzv->values[2];
    }

    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(color);
  }
  marker_array.markers.push_back(line_strip);

  publisher->publish(marker_array);
}

void ompl::control::KinoPlanner::visualizePath(
    const GraphT& g, const std::list<vertex_descriptor>& path,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher, const std::string& ns,
    const std_msgs::msg::ColorRGBA& color, const int& state_space_type)
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
  line_strip.scale.x = 0.1;
  line_strip.scale.y = 0.1;
  line_strip.scale.z = 0.1;
  line_strip.color = color;

  for (size_t i = 1; i < path.size(); i++)
  {
    auto u = *std::next(path.begin(), i - 1);
    auto v = *std::next(path.begin(), i);
    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == base::STATE_SPACE_REAL_VECTOR)
    {
      const auto* source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto* target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    }
    else
    {
      const auto* source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* source_xyzv = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto* target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      target_point.x = target_xyzv->values[0];
      target_point.y = target_xyzv->values[1];
      target_point.z = target_xyzv->values[2];
    }

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

void ompl::control::KinoPlanner::visualizePath(
    const std::shared_ptr<PathControl>& path,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher, const std::string& ns,
    const std_msgs::msg::ColorRGBA& color, const int& state_space_type)
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
  line_strip.scale.x = 0.1;
  line_strip.scale.y = 0.1;
  line_strip.scale.z = 0.1;
  line_strip.color = color;

  for (size_t i = 1; i < path->getStateCount(); i++)
  {
    auto u = path->getState(i - 1);
    auto v = path->getState(i);

    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == ompl::base::STATE_SPACE_REAL_VECTOR)
    {
      const auto* source_cstate = u->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto* target_cstate = v->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    }
    else
    {
      const auto* source_cstate = u->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* source_xyzv = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto* target_cstate = v->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      target_point.x = target_xyzv->values[0];
      target_point.y = target_xyzv->values[1];
      target_point.z = target_xyzv->values[2];
    }

    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(color);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = rclcpp::Clock().now();
    text.ns = ns + "costs";
    text.id = i - 1;  // g[u].id;
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

std_msgs::msg::ColorRGBA ompl::control::KinoPlanner::getColor(std::string& color)
{
  std_msgs::msg::ColorRGBA color_rgba;
  if (color == "red")
  {
    color_rgba.r = 1.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  else if (color == "green")
  {
    color_rgba.r = 0.0;
    color_rgba.g = 1.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  else if (color == "blue")
  {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  }
  else if (color == "yellow")
  {
    color_rgba.r = 1.0;
    color_rgba.g = 1.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  else if (color == "magenta")
  {
    color_rgba.r = 1.0;
    color_rgba.g = 0.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  }
  else if (color == "cyan")
  {
    color_rgba.r = 0.0;
    color_rgba.g = 1.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  }
  else if (color == "white")
  {
    color_rgba.r = 1.0;
    color_rgba.g = 1.0;
    color_rgba.b = 1.0;
    color_rgba.a = 1.0;
  }
  else if (color == "black")
  {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  else
  {
    color_rgba.r = 0.0;
    color_rgba.g = 0.0;
    color_rgba.b = 0.0;
    color_rgba.a = 1.0;
  }
  return color_rgba;
}
