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

#include "vox_nav_planning/native_planners/InformedSGCP.hpp"

ompl::control::InformedSGCP::InformedSGCP(const SpaceInformationPtr & si)
: base::Planner(si, "InformedSGCP")
{
  // set planner specs
  specs_.approximateSolutions = true;
  specs_.optimizingPaths = true;
  specs_.multithreaded = true;
  specs_.canReportIntermediateSolutions = true;
  specs_.approximateSolutions = false;

  siC_ = si.get();

  // declare configuration parameters, the user can use these parameters to tune the planner

  // Number of threads to use for planning, This needs to be an even number and no less than 2
  declareParam<int>(
    "num_threads", this, &InformedSGCP::setNumThreads, &InformedSGCP::getNumThreads, "1:12:12");

  // Batch size for each thread, we generate this many samples in each thread at each iteration
  declareParam<int>(
    "batch_size", this, &InformedSGCP::setBatchSize, &InformedSGCP::getBatchSize, "10:100:1000");

  // maximum number of neighbors to connect to in the graph for any given vertex
  declareParam<int>(
    "max_neighbors", this, &InformedSGCP::setMaxNeighbors, &InformedSGCP::getMaxNeighbors,
    "5:10:20");

  // minimum distance between vertices in the graph, this is used to regulate the graph computation burden
  declareParam<double>(
    "min_dist_between_vertices", this, &InformedSGCP::setMinDistBetweenVertices,
    &InformedSGCP::getMinDistBetweenVertices, "0.01:0.1:1.0");

  // maximum distance between vertices in the graph, this is used to regulate the large and likely invalid motions
  declareParam<double>(
    "max_dist_between_vertices", this, &InformedSGCP::setMaxDistBetweenVertices,
    &InformedSGCP::getMaxDistBetweenVertices, "0.0:0.0:10.0");

  // If valid sampler is available and the user sets this param to true, use it to sample valid states
  declareParam<bool>(
    "use_valid_sampler", this, &InformedSGCP::setUseValidSampler, &InformedSGCP::getUseValidSampler,
    "0,1");

  // Goal bias, this is the probability of sampling the goal state in control graphs
  // the geometric graph does not use this parameter
  declareParam<double>(
    "goal_bias", this, &InformedSGCP::setGoalBias, &InformedSGCP::getGoalBias, "0.0:0.05:0.2");

  // If the user sets this param to true, use k nearest neighbors to connect to the graph
  declareParam<bool>(
    "use_k_nearest", this, &InformedSGCP::setUseKNearest, &InformedSGCP::getUseKNearest, "0,1");

  // as planner progresses, the cost of the best geometric solution is updated
  addPlannerProgressProperty(
    "geometric_cost DOUBLE", [this]() {return std::to_string(bestGeometricCost_.value());});

  // as planner progresses, the cost of the best control solution is updated
  addPlannerProgressProperty(
    "control_cost DOUBLE", [this]() {return std::to_string(bestControlCost_.value());});

}

ompl::control::InformedSGCP::~InformedSGCP()
{
  freeMemory();
}

void ompl::control::InformedSGCP::setup()
{
  base::Planner::setup();

  // Make sure number of threads is even and no less than 2
  if (params_.num_threads_ < 2 || params_.num_threads_ % 2 != 0) {
    OMPL_WARN(
      "%s: Number of threads must be at least 2, and an even number. Setting to 2",
      getName().c_str());
    this->setNumThreads(2);
  }

  // initialize the graphs for all geometric and control threads
  graphGeometricThreads_ = std::vector<GraphT>(params_.num_threads_, GraphT());
  graphControlThreads_ = std::vector<GraphT>(params_.num_threads_, GraphT());

  // reset nn structure for geometric and control threads
  nnGeometricThreads_.clear();
  for (int i = 0; i < params_.num_threads_; i++) {
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> this_nn;
    this_nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
    nnGeometricThreads_.push_back(this_nn);
  }
  nnControlsThreads_.clear();
  for (int i = 0; i < params_.num_threads_; i++) {
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> this_nn;
    this_nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty *>(this));
    nnControlsThreads_.push_back(this_nn);
  }

  // set distance function for all nn geometric and control threads
  for (auto & nn : nnGeometricThreads_) {
    nn->setDistanceFunction(
      [this](const VertexProperty * a, const VertexProperty * b)
      {
        return distanceFunction(a, b);
      });
  }
  for (auto & nn : nnControlsThreads_) {
    nn->setDistanceFunction(
      [this](const VertexProperty * a, const VertexProperty * b)
      {
        return distanceFunction(a, b);
      });
  }

  // set a default optimization objective if one is not set
  if (pdef_) {
    if (pdef_->hasOptimizationObjective()) {
      opt_ = pdef_->getOptimizationObjective();
    } else {
      OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
      opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
      pdef_->setOptimizationObjective(opt_);
    }
  }

  // set all samplers
  if (!validStateSampler_) {
    validStateSampler_ = si_->allocValidStateSampler();
  }
  if (!rejectionInformedSampler_) {
    rejectionInformedSampler_ = std::make_shared<base::RejectionInfSampler>(
      pdef_,
      std::numeric_limits<double>::infinity());
  }
  if (!directedControlSampler_) {
    directedControlSampler_ = std::make_shared<SimpleDirectedControlSampler>(
      siC_,
      params_.k_number_of_controls_);
  }

  kRGG_ = boost::math::constants::e<double>() +
    (boost::math::constants::e<double>() / si_->getStateDimension());

  // initialize the best geometric and control paths
  bestControlPath_ = std::make_shared<PathControl>(si_);
  bestGeometricPath_ = std::make_shared<PathControl>(si_);

  // RVIZ VISUALIZATIONS, this is likely to be removed in the future, but for now it is useful
  node_ = std::make_shared<rclcpp::Node>("InformedSGCP_rclcpp_node");
  rgg_graph_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/InformedSGCP/rgg", rclcpp::SystemDefaultsQoS());
  geometric_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/InformedSGCP/g_plan", rclcpp::SystemDefaultsQoS());
  control_graph_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/InformedSGCP/control_rgg", rclcpp::SystemDefaultsQoS());
  control_path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/InformedSGCP/c_plan", rclcpp::SystemDefaultsQoS());
}

void ompl::control::InformedSGCP::clear()
{
  Planner::clear();
  validStateSampler_.reset();
  rejectionInformedSampler_.reset();
  radius_ = std::numeric_limits<double>::infinity();
  numNeighbors_ = std::numeric_limits<std::size_t>::max();
  bestControlCost_ = opt_->infiniteCost();
  bestGeometricCost_ = opt_->infiniteCost();

  freeMemory();

  // clear the nn for geometric
  for (auto & nn : nnGeometricThreads_) {
    nn->clear();
  }

  // clear the nn for control
  for (auto & nn : nnControlsThreads_) {
    nn->clear();
  }

  // clear all graphs in geometric threads
  for (auto & graph : graphGeometricThreads_) {
    graph.clear();
    graph = GraphT();
  }

  // clear all graphs in control threads
  for (auto & graph : graphControlThreads_) {
    graph.clear();
    graph = GraphT();
  }
}

void ompl::control::InformedSGCP::freeMemory()
{
}

double ompl::control::InformedSGCP::distanceFunction(
  const VertexProperty * a,
  const VertexProperty * b) const
{
  return si_->distance(a->state, b->state);
}

double ompl::control::InformedSGCP::distanceFunction(
  const base::State * a,
  const base::State * b) const
{
  return si_->distance(a, b);
}

const ompl::control::InformedSGCP::VertexProperty * ompl::control::InformedSGCP::getVertex(
  std::size_t id, int thread_id)
{
  return &graphGeometricThreads_[thread_id][id];
}

ompl::control::InformedSGCP::VertexProperty * ompl::control::InformedSGCP::getVertexMutable(
  std::size_t id, int thread_id)
{
  return &graphGeometricThreads_[thread_id][id];
}

const ompl::control::InformedSGCP::VertexProperty * ompl::control::InformedSGCP::getVertexControls(
  std::size_t id, int thread_id)
{
  return &graphControlThreads_[thread_id][id];
}

void ompl::control::InformedSGCP::setNumThreads(int num_threads)
{
  params_.num_threads_ = num_threads;
}

int ompl::control::InformedSGCP::getNumThreads() const
{
  return params_.num_threads_;
}

void ompl::control::InformedSGCP::setBatchSize(int batch_size)
{
  params_.batch_size_ = batch_size;
}

int ompl::control::InformedSGCP::getBatchSize() const
{
  return params_.batch_size_;
}

void ompl::control::InformedSGCP::setMaxNeighbors(int max_neighbors)
{
  params_.max_neighbors_ = max_neighbors;
}

int ompl::control::InformedSGCP::getMaxNeighbors() const
{
  return params_.max_neighbors_;
}

void ompl::control::InformedSGCP::setMinDistBetweenVertices(double min_dist_between_vertices)
{
  params_.min_dist_between_vertices_ = min_dist_between_vertices;
}

double ompl::control::InformedSGCP::getMinDistBetweenVertices() const
{
  return params_.min_dist_between_vertices_;
}

void ompl::control::InformedSGCP::setMaxDistBetweenVertices(double max_dist_between_vertices)
{
  params_.max_dist_between_vertices_ = max_dist_between_vertices;
}

double ompl::control::InformedSGCP::getMaxDistBetweenVertices() const
{
  return params_.max_dist_between_vertices_;
}

void ompl::control::InformedSGCP::setUseValidSampler(bool use_valid_sampler)
{
  params_.use_valid_sampler_ = use_valid_sampler;
}

bool ompl::control::InformedSGCP::getUseValidSampler() const
{
  return params_.use_valid_sampler_;
}

void ompl::control::InformedSGCP::setKNumberControls(int k_number_of_controls)
{
  params_.k_number_of_controls_ = k_number_of_controls;
}

int ompl::control::InformedSGCP::getKNumberControls() const
{
  return params_.k_number_of_controls_;
}

void ompl::control::InformedSGCP::setGoalBias(double goal_bias)
{
  params_.goal_bias_ = goal_bias;
}

double ompl::control::InformedSGCP::getGoalBias() const
{
  return params_.goal_bias_;
}

void ompl::control::InformedSGCP::setUseKNearest(bool use_k_nearest)
{
  params_.use_k_nearest_ = use_k_nearest;
}

bool ompl::control::InformedSGCP::getUseKNearest() const
{
  return params_.use_k_nearest_;
}

void ompl::control::InformedSGCP::setSolveControlGraph(bool solve_control_graph)
{
  params_.solve_control_graph_ = solve_control_graph;
}

bool ompl::control::InformedSGCP::getSolveControlGraph() const
{
  return params_.solve_control_graph_;
}

ompl::base::PlannerStatus ompl::control::InformedSGCP::solve(
  const base::PlannerTerminationCondition & ptc)
{
  // check if the problem is setup properly
  checkValidity();

  // get the goal node and state
  auto * goal_state = si_->allocState();
  auto * start_state = si_->allocState();
  // get start node and state
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }

  if (!si_->getStateValidityChecker()->isValid(start_state)) {
    OMPL_ERROR("%s: Start state is not valid", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }
  if (!si_->getStateValidityChecker()->isValid(goal_state)) {
    OMPL_ERROR("%s: Goal state is not valid", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  // allocate the start and goal vertex properties
  startVertexGeometric_ = new VertexProperty();
  goalVertexGeometric_ = new VertexProperty();
  // Now set the start and goal vertex properties and push them to the nn of all threads
  // NOTE: In gemetric problem setting the we are always solving from start to goal
  startVertexGeometric_->state = start_state;
  goalVertexGeometric_->state = goal_state;
  for (auto & nn : nnGeometricThreads_) {
    nn->add(goalVertexGeometric_);
    nn->add(startVertexGeometric_);
  }

  // For control problem setting, in even threads we are solving from start to goal
  // and in odd threads we are solving from goal to start
  // Due to highly non-unifrom nature of kinodynamic planning problems,
  // we are not sure which setting (start->goal or goal->start) will be solved first
  // So we are solving both for the both settings
  // Then we will try to establish connection between even thread and odd threads,
  // making the planner bi-directional,
  // we keep on doing this until we find best solution within the time limit.
  for (int i = 0; i < params_.num_threads_; i++) {
    auto this_control_root_vertex = new VertexProperty();
    auto this_control_target_vertex = new VertexProperty();
    if (i % 2 == 0) {
      // even thread, solve from start to goal, i.e., place the start state in the root vertex and goal state in the target vertex
      this_control_root_vertex->state = start_state;
      this_control_target_vertex->state = goal_state;
    } else {
      // odd thread, solve from goal to start, i.e., place the goal state in the root vertex and start state in the target vertex
      this_control_root_vertex->state = goal_state;
      this_control_target_vertex->state = start_state;
    }
    // set the root flag on
    this_control_root_vertex->is_root = true;
    // only push the "root" vertex to the nn
    nnControlsThreads_[i]->add(this_control_root_vertex);

    // push the "root" and "yarget" vertex to the graphs
    startVerticesControl_.push_back(this_control_root_vertex);
    goalVerticesControl_.push_back(this_control_target_vertex);
  }

  // If we are not in Eucledian space, then set the max edge distance to 1/10th of the maximum extent of the state space
  if (si_->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR) {
    params_.max_dist_between_vertices_ = si_->getStateSpace()->getMaximumExtent() / 10.0;
  } else {
    // on other spaces, set the max distance to 2.0
    params_.max_dist_between_vertices_ = 2.0;
  }

  // Add goal and start to geomteric graphs in all threads
  std::vector<std::pair<vertex_descriptor, vertex_descriptor>> geometric_start_goal_descriptors;
  for (auto & graph : graphGeometricThreads_) {
    vertex_descriptor start_vertex_descriptor = boost::add_vertex(graph);
    vertex_descriptor goal_vertex_descriptor = boost::add_vertex(graph);
    startVertexGeometric_->id = start_vertex_descriptor;
    goalVertexGeometric_->id = goal_vertex_descriptor;
    graph[start_vertex_descriptor] = *startVertexGeometric_;
    graph[goal_vertex_descriptor] = *goalVertexGeometric_;
    geometric_start_goal_descriptors.push_back(
      std::make_pair(start_vertex_descriptor, goal_vertex_descriptor));
  }

  // Add start and goal (or root and target) vertexes to control graphs in all threads
  std::vector<std::pair<vertex_descriptor, vertex_descriptor>> control_start_goal_descriptors;
  for (int i = 0; i < params_.num_threads_; i++) {
    vertex_descriptor control_g_root = boost::add_vertex(graphControlThreads_[i]);
    vertex_descriptor control_g_target = boost::add_vertex(graphControlThreads_[i]);
    graphControlThreads_[i][control_g_root] = *startVerticesControl_[i];
    graphControlThreads_[i][control_g_root].id = control_g_root;
    graphControlThreads_[i][control_g_root].is_root = true;
    graphControlThreads_[i][control_g_target] = *goalVerticesControl_[i];
    graphControlThreads_[i][control_g_target].id = control_g_target;
    control_start_goal_descriptors.push_back(
      std::make_pair(
        graphControlThreads_[i][control_g_root].id,
        graphControlThreads_[i][control_g_target].id));
  }

  OMPL_INFORM(
    "%s: Using %u geometric graphs.\n", getName().c_str(), graphGeometricThreads_.size());
  OMPL_INFORM(
    "%s: Using %u control graphs each pair of control threads expands towards each other.\n",
    getName().c_str(), graphControlThreads_.size());

  // Keep a copy of shortest path in each thread, described as a list of vertex ids (vertex_descriptor)
  std::vector<std::list<std::size_t>>
  geometric_shortest_paths(params_.num_threads_, std::list<std::size_t>());
  std::vector<std::list<std::size_t>>
  control_shortest_paths(params_.num_threads_, std::list<std::size_t>());

  // Keep a copy of shortest path in each thread, described as ompl::control::PathControl::SharedPtr
  std::vector<std::shared_ptr<PathControl>>
  geometric_ompl_paths(params_.num_threads_, std::make_shared<PathControl>(si_));
  std::vector<std::shared_ptr<PathControl>>
  control_ompl_paths(params_.num_threads_, std::make_shared<PathControl>(si_));

  // Keep a copy of shortest path in each thread, described as vector of VertexProperty for geometric and control graphs
  std::vector<std::vector<VertexProperty *>> bestGeometricVertex(params_.num_threads_,
    std::vector<VertexProperty *>({}));
  std::vector<std::vector<VertexProperty *>> bestControlVertex(params_.num_threads_,
    std::vector<VertexProperty *>({}));

  // A Boost weightmap for each graph in each geometric thread, used to store the cost of each edge
  std::vector<WeightMap> geometric_weightmaps;
  for (auto & graph : graphGeometricThreads_) {
    geometric_weightmaps.push_back(get(boost::edge_weight, graph));
  }
  // A Boost weightmap for each graph in each geometric thread, used to store the cost of each edge
  std::vector<WeightMap> control_weightmaps;
  for (auto & graph : graphControlThreads_) {
    control_weightmaps.push_back(get(boost::edge_weight, graph));
  }

  // The thread ids needs to be immutable for the lambda function
  std::vector<int> thread_ids;
  for (int t = 0; t < params_.num_threads_; t++) {
    thread_ids.push_back(t);
  }

  // initially we have no solution and no approximate solution
  bool exact_solution{false}; bool approximate_solution{false};

  // march forward until we have a solution or we run out of time
  while (ptc == false) {

    // Keep track of the status of each control thread (UNKNOWN, EXACT_SOLUTION, APPROXIMATE_SOLUTION)
    std::vector<int> control_threads_status(params_.num_threads_,
      ompl::base::PlannerStatus::UNKNOWN);

    // compute the number of neighbors and the connection radius and numNeighbors
    auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();
    numNeighbors_ = computeNumberOfNeighbors(numSamplesInInformedSet /*- 2 goal and start */);
    radius_ = computeConnectionRadius(numSamplesInInformedSet /*- 2 goal and start*/);

    // launch geometric planning threads
    std::vector<std::thread *> geometric_threads(params_.num_threads_);
    for (int t = 0; t < params_.num_threads_; t++) {

      int immutable_t = t;
      // Pass all the variables by reference to the lambda function
      auto & thread_id = thread_ids.at(immutable_t);
      auto & geometric_graph = graphGeometricThreads_.at(thread_id);
      auto & geometric_nn = nnGeometricThreads_.at(thread_id);
      auto & geometric_weightmap = geometric_weightmaps.at(thread_id);
      auto & geometric_shortest_path = geometric_shortest_paths.at(thread_id);
      auto & start_goal_descriptor_pair = geometric_start_goal_descriptors.at(thread_id);

      geometric_threads[thread_id] = new std::thread(
        [this,
        &geometric_graph,
        &geometric_nn,
        &geometric_weightmap,
        &geometric_shortest_path,
        &start_goal_descriptor_pair,
        &thread_id,
        &ptc
        ]  {
          // Generate a batch of samples
          std::vector<ompl::base::State *> samples;
          generateBatchofSamples(params_.batch_size_, params_.use_valid_sampler_, samples);

          // expand the graph with the new samples
          expandGeometricGraph(samples, ptc, geometric_graph, geometric_nn, geometric_weightmap);

          // aggressively try to connect the goal to its nearest neighbors
          ensureGeometricGoalVertexConnectivity(
            &geometric_graph[start_goal_descriptor_pair.second], geometric_graph,
            geometric_nn, geometric_weightmap);

          // First lets compute an heuristic working backward from goal -> start (Adaptive Heuristic)
          // This is done by running A*/Dijkstra backwards from goal to start with no collision checking
          auto heuristic =
          GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
            this, &geometric_graph[start_goal_descriptor_pair.first], false, thread_id);
          geometric_shortest_path =
          computeShortestPath<GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>>(
            geometric_graph, geometric_weightmap, heuristic,
            start_goal_descriptor_pair.second,
            start_goal_descriptor_pair.first, true, false);

          // Now an actual heuristic is computed by running A*/Dijkstra from start -> goal without collision checking
          if (geometric_shortest_path.size() > 0) {
            // precomputed heuristic is available,
            // lets use it for actual path search with collision checking
            auto precomputed_heuristic =
            PrecomputedCostHeuristic<GraphT, GraphEdgeCost>(this, thread_id);
            geometric_shortest_path =
            computeShortestPath<PrecomputedCostHeuristic<GraphT, GraphEdgeCost>>(
              geometric_graph, geometric_weightmap, precomputed_heuristic,
              start_goal_descriptor_pair.first,
              start_goal_descriptor_pair.second, false, true);
          }
        });
    }

    // Launch control planning threads
    if (params_.solve_control_graph_) {
      std::vector<std::thread *> control_threads(params_.num_threads_);
      for (int t = 0; t < params_.num_threads_; t++) {
        int immutable_t = t;
        // Pass all the variables by reference to the lambda function
        auto & thread_id = thread_ids.at(immutable_t);
        auto & control_graph = graphControlThreads_.at(thread_id);
        auto & control_nn = nnControlsThreads_.at(thread_id);
        auto & control_weightmap = control_weightmaps.at(thread_id);
        auto & control_shortest_path = control_shortest_paths.at(thread_id);
        auto & start_goal_descriptor_pair = control_start_goal_descriptors.at(thread_id);
        auto & planner_status = control_threads_status.at(thread_id);

        control_threads[thread_id] = new std::thread(
          [this,
          &start_goal_descriptor_pair,
          &control_graph,
          &control_nn,
          &control_weightmap,
          &control_shortest_path,
          &thread_id,
          &planner_status,
          &ptc
          ]  {
            // Generate a batch of samples
            std::vector<ompl::base::State *> samples;
            generateBatchofSamples(params_.batch_size_, params_.use_valid_sampler_, samples);

            // determine the counterpart graph to connect to for this thread
            // Depending on the thread_id, the graph to connect to is either the previous or next thread's graph
            GraphT * connection_graph = nullptr;
            std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> connection_nn = nullptr;
            if (thread_id % 2 == 0) {
              // connect current thread's graph to next thread's graph
              connection_graph = &graphControlThreads_.at(thread_id + 1);
              connection_nn = nnControlsThreads_.at(thread_id + 1);
            } else {
              // connect current thread's graph to previous thread's graph
              connection_graph = &graphControlThreads_.at(thread_id - 1);
              connection_nn = nnControlsThreads_.at(thread_id - 1);
            }

            // expand the graph with the new samples while trying to connect to the counterpart graph (connection_graph and connection_nn)
            expandControlGraph(
              samples,                                                // samples to expand the graph with
              control_graph[start_goal_descriptor_pair.second].state, // target state
              start_goal_descriptor_pair.second,                      // target vertex_descriptor
              ptc,                                                    // termination condition
              connection_graph,                                       // graph to connect to
              connection_nn,                                          // nearest neighbor of graph to connect to
              control_graph,                                          // current graph that is expanding towards target
              control_nn,                                             // current graph nearest neighbor expanding towards target
              control_weightmap                                       // current graph weightmap
            );

            // aggressively try to connect the goal to its nearest neighbors, by "driving" the neighbours towards the goal state
            ensureControlGoalVertexConnectivity(
              control_graph[start_goal_descriptor_pair.second].state,   // target state
              start_goal_descriptor_pair.second,                        // target vertex
              control_graph,                                            // current graph that is expanding towards target
              control_nn,                                               // current graph nearest neighbor
              control_weightmap,                                        // current graph weightmap
              planner_status
            );

            // See if we can connect the start to the goal by A* search
            auto control_forward_heuristic =
            GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>(
              this, &control_graph[start_goal_descriptor_pair.second], true, thread_id);
            control_shortest_path =
            computeShortestPath<GenericDistanceHeuristic<GraphT, VertexProperty, GraphEdgeCost>>(
              control_graph,
              control_weightmap,
              control_forward_heuristic,
              start_goal_descriptor_pair.first,
              start_goal_descriptor_pair.second,
              false, false);
          });
      }

      // Let the control threads finish and join to the main thread
      for (auto & thread : control_threads) {
        thread->join();
        delete thread;
      }

      // Populate the OMPL control paths from vertexes found by A* in the control threads
      for (size_t i = 0; i < control_shortest_paths.size(); i++) {
        bestControlVertex[i].resize(control_shortest_paths[i].size());
        populateOmplPathfromVertexPath(
          control_shortest_paths[i],
          graphControlThreads_[i],
          control_weightmaps[i],
          control_ompl_paths[i],
          bestControlVertex[i],
          true
        );
      }
    }

    // Let the threads finish
    for (auto & thread : geometric_threads) {
      thread->join();
      delete thread;
    }

    // Populate the OMPL geometric paths from vertexes found by A* in the geometric threads
    for (size_t i = 0; i < geometric_shortest_paths.size(); i++) {
      bestGeometricVertex[i].resize(geometric_shortest_paths[i].size());
      populateOmplPathfromVertexPath(
        geometric_shortest_paths[i], graphGeometricThreads_[i], geometric_weightmaps[i],
        geometric_ompl_paths[i], bestGeometricVertex[i], false);
    }

    // Determine best geometric path found by all threads
    auto best_geometric_path(std::make_shared<PathControl>(si_));
    auto best_geometric_path_cost = opt_->infiniteCost();
    auto start_goal_l2_distance = opt_->motionCost(start_state, goal_state);
    int best_geometric_path_index = 0; int geometric_counter = 0;

    for (auto && curr_path : geometric_ompl_paths) {
      auto path_cost = computePathCost(curr_path);
      if (!opt_->isCostBetterThan(path_cost, start_goal_l2_distance)) {
        // Aight, this path seems legit
        if (opt_->isCostBetterThan(path_cost, best_geometric_path_cost)) {
          best_geometric_path = curr_path;
          best_geometric_path_index = geometric_counter;
          best_geometric_path_cost = path_cost;
        }
      }
      geometric_counter++;
    }

    // Determine best control path found by current threads
    auto best_control_path(std::make_shared<PathControl>(si_));
    auto best_control_path_cost = opt_->infiniteCost();
    int control_counter = 0;

    for (auto && curr_path : control_ompl_paths) {
      auto path_cost = computePathCost(curr_path);
      if (!opt_->isCostBetterThan(path_cost, start_goal_l2_distance)) {
        // Aight, this path seems legit
        if (opt_->isCostBetterThan(path_cost, best_control_path_cost)) {
          best_control_path = curr_path;
          bestControlPathIndex_ = control_counter;
          best_control_path_cost = path_cost;
        }
      }
      control_counter++;
    }

    // only for visualization, keep a copy of the best geometric and control graphs
    // Keep a copy before we change graphGeometricThreads_ and graphControlThreads_
    auto best_geometric_graph = graphGeometricThreads_[best_geometric_path_index];
    auto best_control_graph = graphControlThreads_[bestControlPathIndex_];

    // If the cost is less than L2 norm of start and goal, this is likely an useless one.
    // make sure the current cost is not less than L2 norm of start and goal
    if (opt_->isCostBetterThan(start_goal_l2_distance, best_geometric_path_cost)) {
      // This is a valid solution
      // Now we need to check if it is better than the previous one
      if (opt_->isCostBetterThan(best_geometric_path_cost, bestGeometricCost_)) {
        // This is a better solution, update the best cost and path
        bestGeometricCost_ = best_geometric_path_cost;
        bestGeometricPath_ = best_geometric_path;
      }
    }

    // If the cost is less than L2 norm of start and goal, this is likely an useless one.
    // make sure the current cost is not less than L2 norm of start and goal
    if (opt_->isCostBetterThan(
        start_goal_l2_distance,
        best_control_path_cost) &&
      params_.solve_control_graph_)
    {

      // This is a valid solution
      // Now we need to check if it is better than the previous one
      if (opt_->isCostBetterThan(best_control_path_cost, bestControlCost_)) {
        // This is a better solution, update the best cost and path

        // Lets VALIDATE the control path
        bool valid = true;
        for (auto && state : best_control_path->getStates()) {
          if (!si_->isValid(state)) {
            valid = false;
            break;
          }
        }

        if (valid) {
          int temp = ompl::base::PlannerStatus::UNKNOWN;

          if (control_threads_status[bestControlPathIndex_] ==
            ompl::base::PlannerStatus::EXACT_SOLUTION)
          {
            approximate_solution = false;
            exact_solution = true;
            OMPL_INFORM(
              "%s: Found Exact solution with %.2f cost", getName().c_str(),
              best_control_path_cost.value());
            temp = base::PlannerStatus::EXACT_SOLUTION;
            bestControlCost_ = best_control_path_cost;
            bestControlPath_ = best_control_path;
          }

          if (control_threads_status[bestControlPathIndex_] ==
            ompl::base::PlannerStatus::APPROXIMATE_SOLUTION &&
            currentBestSolutionStatus_ == ompl::base::PlannerStatus::EXACT_SOLUTION)
          {
            OMPL_INFORM(
              "%s: Previously Found an Exact solution with %.2f cost but control thread returned approximate solution now, we will skip this solution and keep exact solution.",
              getName().c_str(), bestControlCost_.value());
            temp = base::PlannerStatus::EXACT_SOLUTION;
          } else if (control_threads_status[bestControlPathIndex_] ==
            base::PlannerStatus::APPROXIMATE_SOLUTION &&
            (currentBestSolutionStatus_ == base::PlannerStatus::APPROXIMATE_SOLUTION ||
            currentBestSolutionStatus_ == base::PlannerStatus::UNKNOWN))
          {
            approximate_solution = true;
            exact_solution = true;
            OMPL_INFORM(
              "%s: Found Approximate solution with %.2f cost", getName().c_str(),
              best_control_path_cost.value());
            temp = base::PlannerStatus::APPROXIMATE_SOLUTION;
            bestControlCost_ = best_control_path_cost;
            bestControlPath_ = best_control_path;
            if (static_cast<bool>(Planner::pdef_->getIntermediateSolutionCallback())) {
              pdef_->getIntermediateSolutionCallback()(
                this, getConstStatesFromPath(bestControlPath_), bestControlCost_);
            }
          }
          currentBestSolutionStatus_ = temp;
        }

        if (currentBestSolutionStatus_ != base::PlannerStatus::UNKNOWN) {

          // print number of vertex
          OMPL_INFORM(
            "%s: Best Control Graph has %d vertices and ",
            getName().c_str(), bestControlVertex[bestControlPathIndex_].size());
          // Reset control graphs anyways
          //clearControlGraphs(control_weightmaps, control_start_goal_descriptors);

        }
      } else if (control_threads_status[bestControlPathIndex_] ==
        ompl::base::PlannerStatus::EXACT_SOLUTION)
      {
        // This is not a better solution, but if it is exact solution, clear graphs and populate with solution
        // print number of vertex
        OMPL_INFORM(
          "%s: Another solution was found with cost %.2f but it did not improve previous cost %.2f so we will not update the best control path. ",
          getName().c_str(), best_control_path_cost.value(), bestControlCost_);
        // Reset control graphs anyways
        //clearControlGraphs(control_weightmaps, control_start_goal_descriptors);

      }
    }

    // Visualize the best geometric and control paths in rviz
    std::string red("red");
    std::string green("green");
    std::string blue("blue");

    // geometric path
    visualizePath(
      best_geometric_graph,
      geometric_shortest_paths[best_geometric_path_index],
      geometric_path_pub_,
      "g",
      getColor(blue),
      si_->getStateSpace()->getType());

    // best control path
    visualizePath(
      bestControlPath_,
      control_path_pub_,
      "c",
      getColor(red),
      si_->getStateSpace()->getType());

    visualizeRGG(
      best_geometric_graph,
      rgg_graph_pub_,
      "g",
      getColor(green),
      geometric_start_goal_descriptors[best_geometric_path_index].first,
      geometric_start_goal_descriptors[best_geometric_path_index].second,
      si_->getStateSpace()->getType());

    visualizeRGG(
      best_control_graph,
      control_graph_pub_,
      "c",
      getColor(red),
      control_start_goal_descriptors[bestControlPathIndex_].first,
      control_start_goal_descriptors[bestControlPathIndex_].second,
      si_->getStateSpace()->getType());

  }

  // Add the best path to the solution path
  if (params_.solve_control_graph_) {
    pdef_->addSolutionPath(bestControlPath_, approximate_solution, 0.0, getName());
  } else {
    pdef_->addSolutionPath(bestGeometricPath_, approximate_solution, 0.0, getName());
  }

  // clear data structures
  clear();

  return {exact_solution, approximate_solution};
}

void ompl::control::InformedSGCP::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);
}

void ompl::control::InformedSGCP::generateBatchofSamples(
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
      // sample from the valid sampler
      validStateSampler_->sample(samples.back());
    } else {
      do{
        // Sample in the informed set after an control solution has been found.
        // Otherwise, sample uniformly in the state space.
        ompl::base::Cost min_cost = opt_->infiniteCost();
        if (opt_->isCostBetterThan(bestControlCost_, opt_->infiniteCost())) {
          auto euc_cost = opt_->motionCost(
            startVertexGeometric_->state,
            goalVertexGeometric_->state);
          if (opt_->isCostBetterThan(euc_cost, bestControlCost_)) {
            min_cost = bestControlCost_;
          }
        }
        rejectionInformedSampler_->sampleUniform(
          samples.back(),
          min_cost);
      } while (!si_->getStateValidityChecker()->isValid(samples.back()));
    }
  } while (samples.size() < batch_size); // Keep sampling until batch_size is reached
}

void ompl::control::InformedSGCP::expandGeometricGraph(
  const std::vector<ompl::base::State *> & samples,
  const base::PlannerTerminationCondition & ptc,
  GraphT & geometric_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
  WeightMap & geometric_weightmap)
{
  // Add batch_size_ number of samples to graphs
  // Create edges to construct an RGG, the vertices closer than radius_ will construct an edge
  // But too close vertices will be discarded in order for memory not to sink
  for (auto && i : samples) {

    // if the planner is termination condition is met, break
    if (ptc == true) {
      break;
    }

    // If the sampled state is not valid, discard it and continue
    if (!si_->getStateValidityChecker()->isValid(i)) {
      continue;
    }

    // We will potentially add a new vertex to the graph and nn structure, so allocate a vertex property
    VertexProperty * vertex_property_to_be_added = new VertexProperty();
    vertex_property_to_be_added->state = (i);

    // Get the neighbors of the new vertex with radius_ or k-nearest
    std::vector<ompl::control::InformedSGCP::VertexProperty *> nbh;
    if (params_.use_k_nearest_) {
      geometric_nn->nearestK(vertex_property_to_be_added, numNeighbors_, nbh);
    } else {
      geometric_nn->nearestR(vertex_property_to_be_added, radius_, nbh);
    }

    // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
    // So the further neighbors are discarded
    if (nbh.size() > params_.max_neighbors_) {
      nbh.resize(params_.max_neighbors_);
    }

    // Check if the new vertex is too close to an existing vertex
    bool does_vertice_exits{false};
    for (auto && nb : nbh) {
      double dist = distanceFunction(i, nb->state);
      if (dist < params_.min_dist_between_vertices_) {
        // ops, a very close vertex already exists in graph and nn, do not add due to computational burden*/
        does_vertice_exits = true;
      }
    }

    if (!does_vertice_exits) {

      // Add the new vertex to the graph and nn structure
      vertex_descriptor vertex_descriptor_to_be_added = boost::add_vertex(geometric_graph);
      vertex_property_to_be_added->id = vertex_descriptor_to_be_added;
      geometric_graph[vertex_descriptor_to_be_added] = *vertex_property_to_be_added;
      geometric_nn->add(vertex_property_to_be_added);
      for (auto && nb : nbh) {
        if (!si_->checkMotion(vertex_property_to_be_added->state, nb->state)) {
          continue;
        }

        vertex_descriptor u = vertex_descriptor_to_be_added;
        vertex_descriptor v = nb->id;
        double dist = distanceFunction(
          vertex_property_to_be_added->state,
          nb->state);

        // not to construct edges with self, and if nbh is further than radius_, continue
        if (u == v || dist > params_.max_dist_between_vertices_) {
          continue;
        }
        if (boost::edge(u, v, geometric_graph).second || // if edge already exists, continue
          boost::edge(v, u, geometric_graph).second)
        {
          continue;
        }
        // Once suitable edges are found, populate them over graphs
        edge_descriptor e; bool edge_added;
        boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
        geometric_weightmap[e] =
          opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
      }
    }
  }
}

void ompl::control::InformedSGCP::ensureGeometricGoalVertexConnectivity(
  VertexProperty * target_vertex_property,
  GraphT & geometric_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
  WeightMap & geometric_weightmap)
{

  // get neihbors of the target vertex
  std::vector<ompl::control::InformedSGCP::VertexProperty *> goal_nbh;
  if (params_.use_k_nearest_) {
    geometric_nn->nearestK(target_vertex_property, numNeighbors_, goal_nbh);
  } else {
    geometric_nn->nearestR(target_vertex_property, radius_, goal_nbh);
  }

  // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
  if (goal_nbh.size() > params_.max_neighbors_) {
    goal_nbh.resize(params_.max_neighbors_);
  }

  // iterate over the neighbors and add edges to the target vertex if they are valid
  // and satisfy the distance criteria
  for (auto && nb : goal_nbh) {

    if (!si_->checkMotion(nb->state, target_vertex_property->state)) {
      // Do not add edge to target vertex as it is not valid
      continue;
    }

    vertex_descriptor u = target_vertex_property->id;
    vertex_descriptor v = nb->id;
    double dist = distanceFunction(target_vertex_property->state, nb->state);

    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || (dist > params_.max_dist_between_vertices_)) {
      continue;
    }
    if (boost::edge(u, v, geometric_graph).second || boost::edge(v, u, geometric_graph).second) {
      continue;
    }

    // Connect the target vertex to the neighbor as it is valid and satisfies the distance criterias
    edge_descriptor e; bool edge_added;
    boost::tie(e, edge_added) = boost::add_edge(u, v, geometric_graph);
    geometric_weightmap[e] =
      opt_->motionCost(geometric_graph[u].state, geometric_graph[v].state).value();
  }
}

void ompl::control::InformedSGCP::expandControlGraph(
  const std::vector<ompl::base::State *> & samples,
  const ompl::base::State * target_vertex_state,
  const vertex_descriptor & target_vertex_descriptor,
  const base::PlannerTerminationCondition & ptc,
  const GraphT * connection_control_graph,
  const std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> connection_control_nn,
  GraphT & control_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
  WeightMap & control_weightmap)
{
  // Every ith sample is goal biasing sampling
  int ith_sample = static_cast<int>(1.0 / params_.goal_bias_);
  int goal_bias_counter{0};

  // Iterate over the samples and try to add them to the control graph
  for (auto && i : samples) {

    // Check if the planner has been terminated
    if (ptc == true) {
      break;
    }

    // If the sampled state is not valid, discard it and continue
    if (!si_->getStateValidityChecker()->isValid(i)) {
      continue;
    }

    VertexProperty * vertex_property = new VertexProperty();
    vertex_property->state = i;

    // Every ith sample is goal biasing
    // So if this is the ith sample, then the sample is the goal state
    if ((goal_bias_counter & ith_sample) == 0) {
      auto deep_copy_target_state = si_->allocState();
      si_->copyState(deep_copy_target_state, target_vertex_state);
      vertex_property->state = deep_copy_target_state;
    }
    // increment the goal bias counter
    goal_bias_counter++;

    // Get the nearest neighbors of the sample with numNeighbors_ or radius_
    std::vector<ompl::control::InformedSGCP::VertexProperty *> nbh;
    if (params_.use_k_nearest_) {
      control_nn->nearestK(vertex_property, numNeighbors_, nbh);
    } else {
      control_nn->nearestR(vertex_property, radius_, nbh);
    }

    // if No neighbors found, continue
    if (nbh.size() == 0) {
      continue;
    }

    // Clip the number of neighbors to max_neighbors_, The nearest neighbors are sorted by distance
    if (nbh.size() > params_.max_neighbors_) {
      nbh.resize(params_.max_neighbors_);
    }

    // Check if sample us unique enough
    bool does_vertice_exits{false};
    for (auto && nb : nbh) {
      double dist = distanceFunction(i, nb->state);
      if (dist < params_.min_dist_between_vertices_) {
        // ops this sample is not unique enough, so do not add it to the graph as it increases the computation time
        does_vertice_exits = true;
      }
    }

    if (!does_vertice_exits) {

      // Now this sample is unique enough,  we will try to propogate from it's neighbors to it
      for (auto && nb : nbh) {

        // If the neighbor is goal vertex, do not "propogate" from goal to sample,
        // the ultmate goal is to propogate from sample to goal
        if (nb->id == target_vertex_descriptor) {
          continue;
        }

        // if neighbor is blacklisted, do not propogate from it and continue
        if (nb->blacklisted) {
          continue;
        }

        // Do not modify original sample
        auto deep_copy_sample_state = si_->allocState();
        si_->copyState(deep_copy_sample_state, i);
        // Attempt to drive from neighbour towards newly added sample
        // modify the sample to latest arrived valid state, is it guaranteed to be valid.
        auto c = siC_->allocControl();
        auto duration = directedControlSampler_->sampleTo(
          c,
          nb->state,
          deep_copy_sample_state);

        // if the duration is less than min_control_duration, then probably it hit the boundary
        // or the sample is invalid, so do not add it to the graph
        if (duration < siC_->getMinControlDuration() ||
          !si_->isValid(deep_copy_sample_state))
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
        if (u == v || dist > params_.max_dist_between_vertices_) {
          continue;
        }
        if (boost::edge(u, v, control_graph).second ||
          boost::edge(v, u, control_graph).second)
        {
          continue;
        }

        edge_descriptor e; bool edge_added;
        boost::tie(e, edge_added) = boost::add_edge(u, v, control_graph);
        control_weightmap[e] =
          opt_->motionCost(control_graph[u].state, control_graph[v].state).value();

        // Now check if the arrived vertex connects to any vertex from connection_graph
        VertexProperty * nbh_in_connection_graph = nullptr;
        try {
          nbh_in_connection_graph = connection_control_nn->nearest(arrived_vertex_property);
        } catch (...) {
          OMPL_WARN(
            "%s: No nn available to establish a connection from connection_graph",
            getName().c_str());
          continue;
        }

        if (nbh_in_connection_graph) {

          // We have a neighbor in connection graph, check if it is close enough to connect

          if (distanceFunction(
              nbh_in_connection_graph,
              arrived_vertex_property) < params_.min_dist_between_vertices_)
          {
            // arrived vertex is close to enough to it's nn in connection graph
            // establish connection from arrived vertex from all the way to root of connection_graph,
            // ultimately giving us a path from start to goal
            auto connection_vertex = nbh_in_connection_graph->id;
            auto connection_vertex_parent = arrived_vertex_descriptor;

            // spin until we get Get to the root of the connection_graph which is the goal vertex
            while ((*connection_control_graph)[connection_vertex].is_root == false) {

              auto vertex_to_be_added = boost::add_vertex(control_graph);
              auto vertex_prop_to_be_added = new VertexProperty();
              vertex_prop_to_be_added->id = vertex_to_be_added;
              vertex_prop_to_be_added->parent_id = connection_vertex_parent;
              vertex_prop_to_be_added->state = si_->allocState();

              // if the state is nullptr, then we have reached the root of the connection_graph
              if ((*connection_control_graph)[connection_vertex].state == nullptr) {
                break;
              }

              // Add the connection vertex and all of its parents to the control_graph and nn structure
              si_->copyState(
                vertex_prop_to_be_added->state,
                (*connection_control_graph)[connection_vertex].state);
              control_graph[vertex_to_be_added] = *vertex_prop_to_be_added;
              control_nn->add(vertex_prop_to_be_added);

              // Also add edges from the parent to the vertex
              edge_descriptor e; bool edge_added;
              boost::tie(e, edge_added) =
                boost::add_edge(connection_vertex_parent, vertex_to_be_added, control_graph);
              control_weightmap[e] = opt_->motionCost(
                control_graph[connection_vertex_parent].state,
                control_graph[vertex_to_be_added].state).value();

              // NOTE: The path from connection vertex to root of connection graph is reversed
              // We have to deal with this when we are trying to extract the path from the control_graph
              connection_vertex =
                (*connection_control_graph)[connection_vertex].parent_id;
              connection_vertex_parent = vertex_to_be_added;
            }
          }
        }
      }
    }
  }
}

void ompl::control::InformedSGCP::ensureControlGoalVertexConnectivity(
  const ompl::base::State * target_vertex_state,
  const vertex_descriptor & target_vertex_descriptor,
  GraphT & control_graph,
  std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
  WeightMap & control_weightmap,
  int & status)
{
  // Get all the neighbors of the target vertex
  std::vector<ompl::control::InformedSGCP::VertexProperty *> nbh;
  if (params_.use_k_nearest_) {
    control_nn->nearestK(&control_graph[target_vertex_descriptor], 2.0 * numNeighbors_, nbh);
  } else {
    control_nn->nearestR(&control_graph[target_vertex_descriptor], radius_, nbh);
  }

  // Try to drive from all neighbors to the target vertex
  for (auto && nb : nbh) {

    if (nb->id == target_vertex_descriptor) {
      // Do not add edge to self
      continue;
    }

    // Check if the motion is valid
    if (!si_->checkMotion(nb->state, control_graph[target_vertex_descriptor].state)) {
      continue;
    }

    // Do not modify original sample
    auto deep_copy_sample_state = si_->allocState();
    si_->copyState(deep_copy_sample_state, control_graph[target_vertex_descriptor].state);
    // Attempt to drive towards newly added sample
    // Modify the sample to latest arrived state
    auto c = siC_->allocControl();
    auto duration = directedControlSampler_->sampleTo(
      c,
      nb->state,
      deep_copy_sample_state);

    if (duration < siC_->getMinControlDuration() || !si_->isValid(deep_copy_sample_state)) {
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
    edge_descriptor e; bool edge_added;
    // not to construct edges with self, and if nbh is further than radius_, continue
    if (u == v || dist > params_.max_dist_between_vertices_) {
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

    // calculate the distance between the arrived state and the start/goal
    base::Goal * goal = pdef_->getGoal().get();
    auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    double goal_tolerance = goal_s->getThreshold();
    double dist_to_target_approx = distanceFunction(
      arrived_vertex_property->state, target_vertex_state);
    if (dist_to_target_approx < goal_tolerance) {
      // if the distance is less than the radius to target, then add the edge
      // Exact solution
      vertex_descriptor u = arrived_vertex_property->id;
      vertex_descriptor v = target_vertex_descriptor;
      double dist = distanceFunction(control_graph[u].state, control_graph[v].state);
      edge_descriptor e; bool edge_added;
      // not to construct edges with self, and if nbh is further than radius_, continue
      if (u == v || dist > params_.max_dist_between_vertices_) {
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
      status = base::PlannerStatus::EXACT_SOLUTION;
    }
  }
}

std::size_t ompl::control::InformedSGCP::computeNumberOfSamplesInInformedSet() const
{
  // Loop over all vertices and count the ones in the informed set.
  std::size_t numberOfSamplesInInformedSet{0u};
  for (auto vd : boost::make_iterator_range(vertices(graphControlThreads_[0]))) {

    auto vertex = graphControlThreads_[0][vd].state;

    // Get the best cost to come from any start.
    auto costToCome = opt_->infiniteCost();
    costToCome = opt_->betterCost(
      costToCome, opt_->motionCostHeuristic(startVertexGeometric_->state, vertex));

    // Get the best cost to go to any goal.
    auto costToGo = opt_->infiniteCost();
    costToGo = opt_->betterCost(
      costToCome, opt_->motionCostHeuristic(vertex, goalVertexGeometric_->state));

    // If this can possibly improve the current solution, it is in the informed set.
    if (opt_->isCostBetterThan(
        opt_->combineCosts(costToCome, costToGo),
        bestControlCost_))
    {
      ++numberOfSamplesInInformedSet;
    }
  }

  return numberOfSamplesInInformedSet;
}

double ompl::control::InformedSGCP::computeConnectionRadius(std::size_t numSamples) const
{
  // Define the dimension as a helper variable.
  auto dimension = static_cast<double>(si_->getStateDimension());

  // Compute the RRT* factor. Taken from AITStar::computeConnectionRadius.
  return
    params_.rewire_factor_ * std::pow(
    2.0 * (1.0 + 1.0 / dimension) *
    (rejectionInformedSampler_->getInformedMeasure(bestControlCost_) /
    unitNBallMeasure(si_->getStateDimension())) *
    (std::log(static_cast<double>(numSamples)) / static_cast<double>(numSamples)),
    1.0 / dimension);
}

std::size_t ompl::control::InformedSGCP::computeNumberOfNeighbors(std::size_t numSamples) const
{
  // Compute the RGG factor. Taken from AITStar::computeNumberOfNeighbors.
  return std::ceil(params_.rewire_factor_ * kRGG_ * std::log(static_cast<double>(numSamples)));
}

void ompl::control::InformedSGCP::populateOmplPathfromVertexPath(
  const std::list<vertex_descriptor> & vertex_path,
  GraphT & g,
  WeightMap & weightmap,
  std::shared_ptr<ompl::control::PathControl> & path,
  std::vector<VertexProperty *> & vertexprop_path,
  const bool control
) const
{
  path = std::make_shared<PathControl>(si_);
  int index{0};
  vertex_descriptor prev_vertex = vertex_path.front();
  for (auto && i : vertex_path) {
    // Use this opportunity to mark the edges as invalid if they were blacklisted
    if (g[i].blacklisted) {
      for (auto ed : boost::make_iterator_range(boost::out_edges(i, g))) {
        weightmap[ed] = opt_->infiniteCost().value();
      }
    }
    if (index > 0) {
      bool is_edge_valid = si_->checkMotion(g[prev_vertex].state, g[i].state);
      if (!is_edge_valid) {
        // Mark the edge as invalid
        auto ed = boost::edge(prev_vertex, i, g);
        weightmap[ed.first] = opt_->infiniteCost().value();
      }
      prev_vertex = i;
    }
    if (g[i].control == nullptr && control) {
      // This most likely a start or goal vertex
      // The control has not been allocated for this vertex
      // Allocate a control and set it to zero
      g[i].control = siC_->allocControl();
    }
    if (index == 0) {
      path->append(g[i].state);
    } else {
      if (control) {
        path->append(
          g[i].state, g[i].control,
          g[i].control_duration * siC_->getPropagationStepSize());
      } else {
        path->append(g[i].state);
      }
    }
    vertexprop_path[index] = new VertexProperty(g[i]);
    index++;
  }
}

ompl::base::Cost ompl::control::InformedSGCP::computePathCost(
  std::shared_ptr<ompl::control::PathControl> & path)  const
{
  ompl::base::Cost path_cost = opt_->identityCost();
  if (path->getStateCount() == 0) {
    return path_cost;
  }
  for (std::size_t i = 0; i < path->getStateCount() - 1; ++i) {
    path_cost = opt_->combineCosts(
      path_cost,
      opt_->motionCost(path->getState(i), path->getState(i + 1)));
  }
  return path_cost;
}

std::vector<const ompl::base::State *> ompl::control::InformedSGCP::getConstStatesFromPath(
  const std::shared_ptr<ompl::control::PathControl> & path)
{
  std::vector<const ompl::base::State *> states;
  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    states.push_back(path->getState(i));
  }
  return states;
}

void ompl::control::InformedSGCP::clearControlGraphs(
  std::vector<WeightMap> & control_weightmaps,
  std::vector<std::pair<vertex_descriptor, vertex_descriptor>> & control_start_goal_descriptors
)
{
  // Reset control graphs anyways
  control_start_goal_descriptors.clear();

  for (int i = 0; i < params_.num_threads_; i++) {
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
      std::make_pair(
        graphControlThreads_[i][control_g_root].id,
        graphControlThreads_[i][control_g_target].id));
  }
  // Reset the weightmap
  control_weightmaps.clear();
  for (auto & graph : graphControlThreads_) {
    control_weightmaps.push_back(get(boost::edge_weight, graph));
  }
}

void ompl::control::InformedSGCP::visualizeRGG(
  const GraphT & g,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
  const vertex_descriptor & start_vertex,
  const vertex_descriptor & goal_vertex,
  const int & state_space_type)
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
  for (auto vd : boost::make_iterator_range(vertices(g))) {
    // Paint the start and goal vertices differently.
    std_msgs::msg::ColorRGBA color_vd = color;
    double is_goal_or_start{0.0};
    if (g[vd].id == start_vertex || g[vd].id == goal_vertex) {
      color_vd.b *= 0.5;
    }

    geometry_msgs::msg::Point point;
    if (state_space_type == base::STATE_SPACE_REAL_VECTOR) {
      const auto * target_cstate = g[vd].state->as<ompl::base::RealVectorStateSpace::StateType>();
      point.x = target_cstate->values[0];
      point.y = target_cstate->values[1];
      point.z = target_cstate->values[2];
    } else {
      const auto * target_cstate = g[vd].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * target_xyzv =
        target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
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

  for (auto eit = es.first; eit != es.second; ++eit) {
    u = boost::source(*eit, g);
    v = boost::target(*eit, g);

    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == base::STATE_SPACE_REAL_VECTOR) {
      const auto * source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto * target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    } else {
      const auto * source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * source_xyzv =
        source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto * target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * target_xyzv =
        target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
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

void ompl::control::InformedSGCP::visualizePath(
  const GraphT & g,
  const std::list<vertex_descriptor> & path,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
  const int & state_space_type
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
  line_strip.scale.x = 0.1;
  line_strip.scale.y = 0.1;
  line_strip.scale.z = 0.1;
  line_strip.color = color;

  for (size_t i = 1; i < path.size(); i++) {
    auto u = *std::next(path.begin(), i - 1);
    auto v = *std::next(path.begin(), i);
    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == base::STATE_SPACE_REAL_VECTOR) {
      const auto * source_cstate = g[u].state->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto * target_cstate = g[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    } else {
      const auto * source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * source_xyzv =
        source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto * target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * target_xyzv =
        target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
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

void ompl::control::InformedSGCP::visualizePath(
  const std::shared_ptr<PathControl> & path,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & color,
  const int & state_space_type
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
  line_strip.scale.x = 0.1;
  line_strip.scale.y = 0.1;
  line_strip.scale.z = 0.1;
  line_strip.color = color;

  for (size_t i = 1; i < path->getStateCount(); i++) {

    auto u = path->getState(i - 1);
    auto v = path->getState(i);

    geometry_msgs::msg::Point source_point, target_point;

    if (state_space_type == ompl::base::STATE_SPACE_REAL_VECTOR) {
      const auto * source_cstate = u->as<ompl::base::RealVectorStateSpace::StateType>();
      source_point.x = source_cstate->values[0];
      source_point.y = source_cstate->values[1];
      source_point.z = source_cstate->values[2];

      const auto * target_cstate = v->as<ompl::base::RealVectorStateSpace::StateType>();
      target_point.x = target_cstate->values[0];
      target_point.y = target_cstate->values[1];
      target_point.z = target_cstate->values[2];
    } else {
      const auto * source_cstate = u->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * source_xyzv =
        source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      source_point.x = source_xyzv->values[0];
      source_point.y = source_xyzv->values[1];
      source_point.z = source_xyzv->values[2];

      const auto * target_cstate = v->as<ompl::base::ElevationStateSpace::StateType>();
      const auto * target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto * target_xyzv =
        target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
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

std_msgs::msg::ColorRGBA ompl::control::InformedSGCP::getColor(std::string & color)
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
