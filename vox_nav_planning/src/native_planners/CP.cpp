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

#include "vox_nav_planning/native_planners/CP.hpp"

ompl::control::CP::CP(const SpaceInformationPtr& si) : base::Planner(si, "CP")
{
  // set planner specs
  specs_.approximateSolutions = true;
  specs_.optimizingPaths = true;
  specs_.multithreaded = false;
  specs_.canReportIntermediateSolutions = true;
  specs_.approximateSolutions = true;

  siC_ = si.get();

  // declare configuration parameters, the user can use these parameters to tune the planner

  // Number of threads to use for planning, This needs to be an even number and no less than 2
  declareParam<int>("num_threads", this, &CP::setNumThreads, &CP::getNumThreads, "1:12:12");

  // Batch size for each thread, we generate this many samples in each thread at each iteration
  declareParam<int>("min_number_of_branches_to_extend", this, &CP::setMinNumberOfBranchesToExtend,
                    &CP::getMinNumberOfBranchesToExtend, "1:5:10");

  // Maximum number of samples to generate in each thread at each iteration
  declareParam<int>("max_number_of_branches_to_extend", this, &CP::setMaxNumberOfBranchesToExtend,
                    &CP::getMaxNumberOfBranchesToExtend, "10:12:20");

  // Maximum number of frontier nodes to keep in each thread
  declareParam<int>("max_number_of_frontier_nodes", this, &CP::setMaxNumberOfFrontierNodes,
                    &CP::getMaxNumberOfFrontierNodes, "10:50:100");

  // Minimum distance between nodes in the graph
  declareParam<double>("min_distance_between_nodes", this, &CP::setMinDistanceBetweenNodes,
                       &CP::getMinDistanceBetweenNodes, "0.1:0.1:2.0");

  // k parameter for preferring nodes with low number of branches
  declareParam<double>("k_prefer_nodes_with_low_branches", this, &CP::setKPreferNodesWithLowBranches,
                       &CP::getKPreferNodesWithLowBranches, "0.0:1.0:5.0");

  // k parameter for preferring nodes with high cost
  declareParam<double>("k_prefer_nodes_with_high_cost", this, &CP::setKPreferNodesWithHighCost,
                       &CP::getKPreferNodesWithHighCost, "0.0:5.0:5.0");

  // Number of neighbors to consider for density calculation
  declareParam<int>("num_of_neighbors_to_consider_for_density", this, &CP::setNumOfNeighborsToConsiderForDensity,
                    &CP::getNumOfNeighborsToConsiderForDensity, "1:20:25");

  // as planner progresses, the cost of the best control solution is updated
  addPlannerProgressProperty("control_cost DOUBLE", [this]() { return std::to_string(bestControlCost_.value()); });
}

ompl::control::CP::~CP()
{
  freeMemory();
}

void ompl::control::CP::setup()
{
  base::Planner::setup();

  // Make sure number of threads is even and no less than 2
  if (params_.num_threads_ < 2 || params_.num_threads_ % 2 != 0)
  {
    OMPL_WARN("%s: Number of threads must be at least 2, and an even number. Setting to 2", getName().c_str());
    this->setNumThreads(2);
  }

  nnControlsThreads_.clear();
  for (int i = 0; i < params_.num_threads_; i++)
  {
    std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>> this_nn;
    this_nn.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty*>(this));

    nnControlsThreads_.push_back(this_nn);
  }

  // set distance function for all nn geometric and control threads
  for (auto& nn : nnControlsThreads_)
  {
    nn->setDistanceFunction(
        [this](const VertexProperty* a, const VertexProperty* b) { return distanceFunction(a, b); });
  }

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

  // set all samplers
  if (!controlSampler_)
  {
    controlSampler_ = siC_->allocControlSampler();
  }

  // initialize the best geometric and control paths
  bestControlPath_ = std::make_shared<PathControl>(si_);

  // initialize the best control cost
  bestControlCost_ = opt_->infiniteCost();

  // RVIZ VISUALIZATIONS, this is likely to be removed in the future, but for now it is useful
  node_ = std::make_shared<rclcpp::Node>("CP_rclcpp_node");
  rgg_graph_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/CP/rgg", rclcpp::SystemDefaultsQoS());
  geometric_path_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/CP/g_plan", rclcpp::SystemDefaultsQoS());
  first_control_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "vox_nav/CP/first_control_rgg", rclcpp::SystemDefaultsQoS());
  second_control_graph_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "vox_nav/CP/second_control_rgg", rclcpp::SystemDefaultsQoS());
  control_path_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("vox_nav/CP/c_plan", rclcpp::SystemDefaultsQoS());
}

void ompl::control::CP::clear()
{
  Planner::clear();
  bestControlCost_ = opt_->infiniteCost();

  freeMemory();

  // clear the nn for control
  for (auto& nn : nnControlsThreads_)
  {
    nn->clear();
  }
}

void ompl::control::CP::freeMemory()
{
}

double ompl::control::CP::distanceFunction(const VertexProperty* a, const VertexProperty* b) const
{
  return si_->distance(a->state, b->state);
}

double ompl::control::CP::distanceFunction(const base::State* a, const base::State* b) const
{
  return si_->distance(a, b);
}

void ompl::control::CP::setNumThreads(int num_threads)
{
  params_.num_threads_ = num_threads;
}

int ompl::control::CP::getNumThreads() const
{
  return params_.num_threads_;
}

void ompl::control::CP::setMinNumberOfBranchesToExtend(int x)
{
  params_.min_number_of_branches_to_extend_ = x;
}

int ompl::control::CP::getMinNumberOfBranchesToExtend() const
{
  return params_.min_number_of_branches_to_extend_;
}

void ompl::control::CP::setMaxNumberOfBranchesToExtend(int x)
{
  params_.max_number_of_branches_to_extend_ = x;
}

int ompl::control::CP::getMaxNumberOfBranchesToExtend() const
{
  return params_.max_number_of_branches_to_extend_;
}

void ompl::control::CP::setMaxNumberOfFrontierNodes(int x)
{
  params_.max_number_of_frontier_nodes_ = x;
}

int ompl::control::CP::getMaxNumberOfFrontierNodes() const
{
  return params_.max_number_of_frontier_nodes_;
}

void ompl::control::CP::setMinDistanceBetweenNodes(double x)
{
  params_.min_distance_between_nodes_ = x;
}

double ompl::control::CP::getMinDistanceBetweenNodes() const
{
  return params_.min_distance_between_nodes_;
}

void ompl::control::CP::setKPreferNodesWithLowBranches(double x)
{
  params_.k_prefer_nodes_with_low_branches_ = x;
}

double ompl::control::CP::getKPreferNodesWithLowBranches() const
{
  return params_.k_prefer_nodes_with_low_branches_;
}

void ompl::control::CP::setKPreferNodesWithHighCost(double x)
{
  params_.k_prefer_nodes_with_high_cost_ = x;
}

double ompl::control::CP::getKPreferNodesWithHighCost() const
{
  return params_.k_prefer_nodes_with_high_cost_;
}

void ompl::control::CP::setNumOfNeighborsToConsiderForDensity(int x)
{
  params_.num_of_neighbors_to_consider_for_density_ = x;
}

int ompl::control::CP::getNumOfNeighborsToConsiderForDensity() const
{
  return params_.num_of_neighbors_to_consider_for_density_;
}

ompl::base::PlannerStatus ompl::control::CP::solve(const base::PlannerTerminationCondition& ptc)
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

  // For control problem setting, in even threads we are solving from start to goal
  // and in odd threads we are solving from goal to start
  // Due to highly non-unifrom nature of kinodynamic planning problems,
  // we are not sure which setting (start->goal or goal->start) will be solved first
  // So we are solving both for the both settings
  // Then we will try to establish connection between even thread and odd threads,
  // making the planner bi-directional,
  // we keep on doing this until we find best solution within the time limit.
  for (int i = 0; i < params_.num_threads_; i++)
  {
    auto this_control_root_vertex = new VertexProperty();
    auto this_control_target_vertex = new VertexProperty();
    if (i % 2 == 0)
    {
      // even thread, solve from start to goal, i.e., place the start state in the root vertex and goal state in the
      // target vertex
      this_control_root_vertex->state = start_state;
      this_control_target_vertex->state = goal_state;
    }
    else
    {
      // odd thread, solve from goal to start, i.e., place the goal state in the root vertex and start state in the
      // target vertex
      this_control_root_vertex->state = goal_state;
      this_control_target_vertex->state = start_state;
    }

    this_control_root_vertex->belongs_to_solution = true;
    this_control_target_vertex->belongs_to_solution = true;

    // set the root flag on
    this_control_root_vertex->is_root = true;
    this_control_root_vertex->cost = opt_->identityCost();
    // empty control
    this_control_root_vertex->control = siC_->allocControl();
    // only push the "root" vertex to the nn
    nnControlsThreads_[i]->add(this_control_root_vertex);

    // push the "root" and "target" vertex to the graphs
    startVerticesControl_.push_back(this_control_root_vertex);
    goalVerticesControl_.push_back(this_control_target_vertex);
  }

  OMPL_INFORM("%s: Using %u control graphs each pair of control threads expands towards each other.\n",
              getName().c_str(), params_.num_threads_);

  std::vector<std::shared_ptr<PathControl>> control_ompl_paths(params_.num_threads_,
                                                               std::make_shared<PathControl>(si_));

  std::vector<std::vector<VertexProperty*>> control_paths_vertices(params_.num_threads_,
                                                                   std::vector<VertexProperty*>());

  std::vector<bool*> should_stop_exploration_flags(params_.num_threads_, new bool(false));

  // The thread ids needs to be immutable for the lambda function
  std::vector<int> thread_ids;
  for (int t = 0; t < params_.num_threads_; t++)
  {
    thread_ids.push_back(t);
  }

  // initially we have no solution and no approximate solution
  bool exact_solution{ false };
  bool approximate_solution{ false };

  // march forward until we have a solution or we run out of time
  while (ptc == false)
  {
    // Launch control planning threads
    std::vector<std::thread*> control_threads(params_.num_threads_);
    for (int t = 0; t < params_.num_threads_; t++)
    {
      int immutable_t = t;
      // Pass all the variables by reference to the lambda function
      auto& thread_id = thread_ids.at(immutable_t);
      auto& control_nn = nnControlsThreads_.at(thread_id);
      auto& control_shortest_path = control_ompl_paths.at(thread_id);
      auto& target_property = goalVerticesControl_.at(thread_id);
      auto& this_control_path_vertices = control_paths_vertices.at(thread_id);
      auto should_stop_exploration = should_stop_exploration_flags.at(thread_id);

      control_threads[thread_id] = new std::thread([this, &target_property, &control_nn, &control_shortest_path,
                                                    &thread_id, &ptc, &exact_solution, &approximate_solution,
                                                    &this_control_path_vertices, &should_stop_exploration] {
        std::vector<VertexProperty*> frontier_nodes;
        // Extend frontier nodes with random controls
        // Add random branch no in the range of min_number_of_branches_to_extend_ and max_number_of_branches_to_extend_
        int random_branch_no =
            rng_.uniformInt(params_.min_number_of_branches_to_extend_, params_.max_number_of_branches_to_extend_);
        selectExplorativeFrontiers(params_.max_number_of_frontier_nodes_, control_nn, frontier_nodes);
        extendFrontiers(frontier_nodes, random_branch_no, control_nn, ptc, target_property, control_shortest_path,
                        this_control_path_vertices, exact_solution, should_stop_exploration, bestControlPath_);
      });
    }

    // Let the control threads finish and join to the main thread
    for (auto& thread : control_threads)
    {
      // Wait for the thread to finish
      thread->join();
      delete thread;
    }

    auto start_goal_l2_distance = opt_->motionCost(start_state, goal_state);

    // Determine best control path found by current threads
    int control_counter = 0;

    bool should_update_nn{ false };

    for (auto&& curr_path : control_ompl_paths)
    {
      auto path_cost = computePathCost(curr_path);
      if (!opt_->isCostBetterThan(path_cost, start_goal_l2_distance))
      {
        // Aight, this path seems legit
        if (opt_->isCostBetterThan(path_cost, bestControlCost_))
        {
          OMPL_INFORM("%s: Found a better control path with cost %.2f over previous cost %.2f", getName().c_str(),
                      path_cost.value(), bestControlCost_.value());

          bestControlPath_ = curr_path;
          bestControlPathIndex_ = control_counter;
          bestControlCost_ = path_cost;
          bestControlPathNN_ = nnControlsThreads_[bestControlPathIndex_];

          should_update_nn = true;

          // We have a solution
          approximate_solution = false;
          exact_solution = true;
        }
      }
      control_counter++;
    }

    // if all of the NN are telling that we should stop exploration, then we should stop exploration and update the NN
    // structures
    bool all_nn_are_telling_to_stop_exploration{ true };
    for (auto&& should_stop_exploration : should_stop_exploration_flags)
    {
      if (!*should_stop_exploration)
      {
        all_nn_are_telling_to_stop_exploration = false;
      }
    }

    if (all_nn_are_telling_to_stop_exploration)
    {
      should_update_nn = true;
      // set all the should_stop_exploration flags to false again
      for (auto&& should_stop_exploration : should_stop_exploration_flags)
      {
        *should_stop_exploration = false;
      }
    }

    // Update the NN structures
    if (bestControlPath_->getStateCount() > 0 && should_update_nn)
    {
      // fill this_nn with the best control path NN structure
      std::vector<VertexProperty*> vertices = control_paths_vertices[bestControlPathIndex_];

      // Choose the random state from the frist half of the best control path
      // This is to avoid the case where the random state is in the second half of the best control path
      int random_state_index = rng_.uniformInt(0, bestControlPath_->getStateCount() / 2 - 1);

      for (int t = 0; t < params_.num_threads_; t++)
      {
        // equate this NN structure with the best bestControlPathIndex_ but create a copy
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>> this_nn;
        this_nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<VertexProperty*>(this));

        this_nn->setDistanceFunction(
            [this](const VertexProperty* a, const VertexProperty* b) { return distanceFunction(a, b); });

        // also equate the shortest paths with the best bestControlPathIndex_
        control_ompl_paths[t] = control_ompl_paths[bestControlPathIndex_];
        control_paths_vertices[t] = control_paths_vertices[bestControlPathIndex_];
        // also equate the root and target vertices with the best bestControlPathIndex_
        startVerticesControl_[t] = startVerticesControl_[bestControlPathIndex_];
        goalVerticesControl_[t] = goalVerticesControl_[bestControlPathIndex_];

        std::vector<VertexProperty*> solution_vertices;
        for (auto&& vertex : vertices)
        {
          if (!vertex->belongs_to_solution)
          {
            continue;
          }
          solution_vertices.push_back(vertex);
        }

        // Now pick a random vertex from the solution vertices and add it to the new NN structure
        // make sure the random_state_index is not out of bounds
        if (random_state_index > solution_vertices.size() - 1)
        {
          OMPL_WARN("%s: Random state index is out of bounds, setting it to the last index", getName().c_str());
          OMPL_WARN("%s: Random state index is %d but the solution vertices size is %d", getName().c_str(),
                    random_state_index, solution_vertices.size());
          random_state_index = solution_vertices.size() - 1;
        }

        // from start up to the random state index, add the vertices to the new NN structure and black list them
        for (int i = 0; i < random_state_index; i++)
        {
          auto vertex = vertices[i];
          auto* new_vertex = new VertexProperty();
          new_vertex->state = vertex->state;
          new_vertex->control = vertex->control;
          new_vertex->control_duration = vertex->control_duration;
          new_vertex->cost = vertex->cost;
          new_vertex->parent = vertex->parent;
          new_vertex->is_root = vertex->is_root;
          new_vertex->belongs_to_solution = vertex->belongs_to_solution;
          new_vertex->blacklisted = true;
          this_nn->add(new_vertex);
        }

        // Now add the random state to the new NN structure and do not black list it
        auto vertex = vertices[random_state_index];
        auto* new_vertex = new VertexProperty();
        new_vertex->state = vertex->state;
        new_vertex->control = vertex->control;
        new_vertex->control_duration = vertex->control_duration;
        new_vertex->cost = vertex->cost;
        new_vertex->parent = vertex->parent;
        new_vertex->is_root = vertex->is_root;
        new_vertex->belongs_to_solution = vertex->belongs_to_solution;
        new_vertex->blacklisted = false;
        this_nn->add(new_vertex);

        nnControlsThreads_[t] = this_nn;
      }
    }

    // only for visualization, keep a copy of the best control graphs
    // Keep a copy before we change and graphControlThreads_
    auto best_control_nn_structure = nnControlsThreads_[bestControlPathIndex_];

    int counterpart_index{ 0 };
    if (bestControlPathIndex_ % 2 == 0)
    {
      counterpart_index = bestControlPathIndex_ + 1;
    }
    else
    {
      counterpart_index = bestControlPathIndex_ - 1;
    }
    auto best_control_nn_structure_counterpart = nnControlsThreads_[counterpart_index];

    // Visualize the best geometric and control paths in rviz
    std::string red("red");
    std::string green("green");
    std::string blue("blue");

    // best control path

    visualizePath(bestControlPath_, control_path_pub_, "c", getColor(red), si_->getStateSpace()->getType());

    visualizeRGG(best_control_nn_structure, first_control_graph_pub_, "c", getColor(red),
                 si_->getStateSpace()->getType());

    visualizeRGG(best_control_nn_structure_counterpart, second_control_graph_pub_, "c", getColor(blue),
                 si_->getStateSpace()->getType());

    if (static_cast<bool>(Planner::pdef_->getIntermediateSolutionCallback()))
    {
      pdef_->getIntermediateSolutionCallback()(this, getConstStatesFromPath(bestControlPath_), bestControlCost_);
    }
  }

  // Add the best path to the solution path
  OMPL_INFORM("%s: Best Control path has %d vertices and ", getName().c_str(), bestControlPath_->getStateCount());
  pdef_->addSolutionPath(bestControlPath_, approximate_solution, 0.0, getName());

  // clear data structures
  clear();

  // report the solution status
  return { exact_solution, approximate_solution };
}

void ompl::control::CP::getPlannerData(base::PlannerData& data) const
{
  Planner::getPlannerData(data);
}

void ompl::control::CP::selectExplorativeFrontiers(
    int max_number, std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& nn_structure,
    std::vector<VertexProperty*>& frontier_nodes)
{
  nn_structure->list(frontier_nodes);

  // remove the blacklisted nodes
  frontier_nodes.erase(std::remove_if(frontier_nodes.begin(), frontier_nodes.end(),
                                      [](const VertexProperty* a) { return a->blacklisted; }),
                       frontier_nodes.end());

  // Get number of neighbors within radius r for the minimum branch nodes
  std::vector<ompl::base::Cost> costs;
  std::vector<int> num_branches;

  for (auto&& frontier_node : frontier_nodes)
  {
    std::vector<VertexProperty*> knn;
    costs.push_back(frontier_node->cost);
    num_branches.push_back(frontier_node->branches.size());
  }

  // based on the number of neighbors, costs and number of branches create best frontier nodes
  std::vector<double> best_frontier_candidate_scores;
  for (size_t i = 0; i < frontier_nodes.size(); i++)
  {
    double score = params_.k_prefer_nodes_with_low_branches_ * (1.0 / (num_branches[i] + 0.01)) +  // NOLINT
                   params_.k_prefer_nodes_with_high_cost_ * costs[i].value();
    best_frontier_candidate_scores.push_back(score);
  }

  // Step 1: Create a vector of indices
  std::vector<size_t> indices(frontier_nodes.size());
  std::iota(indices.begin(), indices.end(), 0);  // Fill with 0, 1, 2, ...

  // Step 2: Sort the indices based on corresponding values in num_neighbors
  std::sort(indices.begin(), indices.end(), [&best_frontier_candidate_scores](double i1, double i2) {
    return best_frontier_candidate_scores[i1] > best_frontier_candidate_scores[i2];
  });

  // Step 3: Reorder frontier_nodes according to the sorted indices
  std::vector<VertexProperty*> sorted_frontier_nodes(frontier_nodes.size());
  for (size_t i = 0; i < indices.size(); ++i)
  {
    sorted_frontier_nodes[i] = frontier_nodes[indices[i]];
  }

  // Now sorted_frontier_nodes is sorted according to the number of neighbors
  frontier_nodes = std::move(sorted_frontier_nodes);

  // clip the number of frontier nodes to max_number
  if (frontier_nodes.size() > 3 * max_number)
  {
    frontier_nodes.resize(3 * max_number);
  }

  // calculate neihborhood density for each frontier node, prefer nodes with lower density
  std::vector<double> neighborhood_densities;
  for (auto&& frontier_node : frontier_nodes)
  {
    std::vector<VertexProperty*> knn;
    nn_structure->nearestK(frontier_node, params_.num_of_neighbors_to_consider_for_density_, knn);
    double dist = 0.0;
    for (auto&& neighbor : knn)
    {
      dist += distanceFunction(frontier_node, neighbor);
    }
    neighborhood_densities.push_back(dist);  // NOLINT
  }

  // Step 1: Create a vector of indices
  std::vector<size_t> indices2(frontier_nodes.size());
  std::iota(indices2.begin(), indices2.end(), 0);  // Fill with 0, 1, 2, ...

  // Step 2: Sort the indices based on corresponding values in num_neighbors
  std::sort(indices2.begin(), indices2.end(), [&neighborhood_densities](double i1, double i2) {
    return neighborhood_densities[i1] > neighborhood_densities[i2];
  });

  // Step 3: Reorder frontier_nodes according to the sorted indices
  std::vector<VertexProperty*> sorted_frontier_nodes2(frontier_nodes.size());
  for (size_t i = 0; i < indices2.size(); ++i)
  {
    sorted_frontier_nodes2[i] = frontier_nodes[indices2[i]];
  }

  // Now sorted_frontier_nodes is sorted according to the number of neighbors
  frontier_nodes = std::move(sorted_frontier_nodes2);

  // clip the number of frontier nodes to max_number
  if (frontier_nodes.size() > max_number)
  {
    frontier_nodes.resize(max_number);
  }
}

void ompl::control::CP::extendFrontiers(std::vector<VertexProperty*>& frontier_nodes, int num_branch_to_extend,
                                        std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& nn_structure,
                                        const base::PlannerTerminationCondition& ptc, VertexProperty* target_property,
                                        std::shared_ptr<PathControl>& path,
                                        std::vector<VertexProperty*>& control_paths_vertices,
                                        const bool exact_solution_found, bool* should_stop_exploration,
                                        const std::shared_ptr<PathControl>& current_best_path)
{
  // If all the new branches ended up with higher cost than the current best path, then we should stop exploration
  bool all_branches_are_worse_than_current_best_path{ true };

  base::Goal* goal = pdef_->getGoal().get();
  double tolerance = goal->as<base::GoalRegion>()->getThreshold();

  // Now lets expand the frontier nodes with random controls and add them to the graph
  for (auto&& frontier_node : frontier_nodes)
  {
    // Add max number of branches to the frontier nodes
    for (size_t i = 0; i < num_branch_to_extend; i++)
    {
      // Check if the planner has been terminated
      if (ptc == true)
      {
        return;
      }

      // allocate a control
      Control* control = siC_->allocControl();
      // allocate a new state
      auto* new_state = si_->allocState();

      int control_duration = 0;

      controlSampler_->sampleNext(control, frontier_node->control);
      control_duration = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
      // propagate the control from the frontier node
      // this will generate a new state
      control_duration = siC_->propagateWhileValid(frontier_node->state, control, control_duration, new_state);

      // Create a new vertex.
      VertexProperty* vertex_property = new VertexProperty();
      vertex_property->state = new_state;

      // check that if the nearestK search to this node is too close skip as it is likely to be a duplicate
      std::vector<VertexProperty*> knn;
      nn_structure->nearestK(vertex_property, 1, knn);
      double goal_dist = distanceFunction(vertex_property, target_property);

      if (knn.size() > 0)
      {
        double dist = distanceFunction(vertex_property, knn[0]);
        if (dist < params_.min_distance_between_nodes_ && goal_dist > tolerance)
        {
          continue;
        }
      }

      // if the number of steps taken is less than the random step size, then the propagation failed blacklist the new
      // node because states propogated from this new node will also fail most likely
      if (control_duration < siC_->getMinControlDuration())
      {
        vertex_property->blacklisted = true;
      }

      // the parent of the new node is the frontier node
      vertex_property->parent = frontier_node;

      vertex_property->control_duration = control_duration;

      // do add accumulated cost to the new node
      vertex_property->cost =
          opt_->combineCosts(frontier_node->cost, opt_->motionCost(frontier_node->state, new_state));

      // check if the cost of the new node is better than the current best path
      // this is relevant if there is an exact solution
      if (opt_->isCostBetterThan(vertex_property->cost, computePathCost(current_best_path)))
      {
        all_branches_are_worse_than_current_best_path = false;
      }

      frontier_node->branches.push_back(vertex_property);

      // add the new node to the graph
      nn_structure->add(vertex_property);

      // check if the new node is close to the goal
      // if it is, connect it to the goal

      if (goal_dist < tolerance)
      {
        // proceed only if the cost is better than tshe current best cost
        auto current_best_cost = computePathCost(current_best_path);

        if (!opt_->isCostBetterThan(vertex_property->cost, current_best_cost))
        {
          continue;
        }

        // back track the path from the new node to the root node and add it to the path
        VertexProperty* current_vertex = vertex_property;
        control_paths_vertices.clear();
        while (current_vertex != nullptr)
        {
          current_vertex->belongs_to_solution = true;
          control_paths_vertices.push_back(current_vertex);
          current_vertex = current_vertex->parent;
        }
        std::reverse(control_paths_vertices.begin(), control_paths_vertices.end());

        // reset the path from previous successful iteration
        path = std::make_shared<PathControl>(si_);

        for (auto&& vertex : control_paths_vertices)
        {
          // if control is not allocated for this vertex, allocate it
          if (vertex->control == nullptr)
          {
            vertex->control = siC_->allocControl();
          }

          path->append(vertex->state, vertex->control, vertex->control_duration * siC_->getPropagationStepSize());
        }

        // add the goal state to the path
        path->append(target_property->state);
      }
    }
  }
  if (all_branches_are_worse_than_current_best_path)
  {
    *should_stop_exploration = true;
  }
}

ompl::base::Cost ompl::control::CP::computePathCost(std::shared_ptr<ompl::control::PathControl>& path) const
{
  ompl::base::Cost path_cost = opt_->infiniteCost();
  if (path->getStateCount() == 0)
  {
    return path_cost;
  }
  path_cost = opt_->identityCost();
  for (std::size_t i = 0; i < path->getStateCount() - 1; ++i)
  {
    path_cost = opt_->combineCosts(path_cost, opt_->motionCost(path->getState(i), path->getState(i + 1)));
  }
  return path_cost;
}

ompl::base::Cost ompl::control::CP::computePathCost(const std::shared_ptr<ompl::control::PathControl>& path) const
{
  ompl::base::Cost path_cost = opt_->infiniteCost();
  if (path->getStateCount() == 0)
  {
    return path_cost;
  }
  path_cost = opt_->identityCost();
  for (std::size_t i = 0; i < path->getStateCount() - 1; ++i)
  {
    path_cost = opt_->combineCosts(path_cost, opt_->motionCost(path->getState(i), path->getState(i + 1)));
  }
  return path_cost;
}

std::vector<const ompl::base::State*>
ompl::control::CP::getConstStatesFromPath(const std::shared_ptr<ompl::control::PathControl>& path)
{
  std::vector<const ompl::base::State*> states;
  for (std::size_t i = 0; i < path->getStateCount(); ++i)
  {
    states.push_back(path->getState(i));
  }
  return states;
}

void ompl::control::CP::visualizeRGG(
    const std::shared_ptr<ompl::NearestNeighbors<VertexProperty*>>& nn_structure,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher, const std::string& ns,
    const std_msgs::msg::ColorRGBA& color, const int& state_space_type)
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

  visualization_msgs::msg::Marker spheres;
  spheres.header.frame_id = "map";
  spheres.header.stamp = rclcpp::Clock().now();
  spheres.ns = ns + "rgg_vertex";
  spheres.id = 0;
  spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::msg::Marker::ADD;
  spheres.scale.x = 0.15;
  spheres.scale.y = 0.15;
  spheres.scale.z = 0.15;

  int edge_index = 0;
  visualization_msgs::msg::Marker line_stripes;
  line_stripes.header.frame_id = "map";
  line_stripes.ns = ns + "rgg_edges";
  line_stripes.id = edge_index;
  line_stripes.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_stripes.action = visualization_msgs::msg::Marker::ADD;
  line_stripes.lifetime = rclcpp::Duration::from_seconds(0);
  line_stripes.header.stamp = rclcpp::Clock().now();
  line_stripes.scale.x = 0.01;
  line_stripes.scale.y = 0.01;
  line_stripes.scale.z = 0.01;
  line_stripes.color = color;

  visualization_msgs::msg::MarkerArray marker_array;

  // iterate through all the vertices in the graph
  std::vector<VertexProperty*> vertices;
  nn_structure->list(vertices);

  for (auto vd : vertices)
  {
    // Paint the start and goal vertices differently.
    std_msgs::msg::ColorRGBA color_vd = color;
    double is_goal_or_start{ 0.0 };

    geometry_msgs::msg::Point point;
    if (state_space_type == base::STATE_SPACE_REAL_VECTOR)
    {
      const auto* target_cstate = vd->state->as<ompl::base::RealVectorStateSpace::StateType>();
      point.x = target_cstate->values[0];
      point.y = target_cstate->values[1];
      point.z = target_cstate->values[2];
    }
    else
    {
      const auto* target_cstate = vd->state->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      point.x = target_xyzv->values[0];
      point.y = target_xyzv->values[1];
      point.z = target_xyzv->values[2];
    }

    spheres.points.push_back(point);
    spheres.colors.push_back(color);

    // draw lines from current vd to all its branches
    for (auto b : vd->branches)
    {
      geometry_msgs::msg::Point source_point, target_point;
      if (state_space_type == base::STATE_SPACE_REAL_VECTOR)
      {
        const auto* source_cstate = vd->state->as<ompl::base::RealVectorStateSpace::StateType>();
        source_point.x = source_cstate->values[0];
        source_point.y = source_cstate->values[1];
        source_point.z = source_cstate->values[2];

        const auto* target_cstate = b->state->as<ompl::base::RealVectorStateSpace::StateType>();
        target_point.x = target_cstate->values[0];
        target_point.y = target_cstate->values[1];
        target_point.z = target_cstate->values[2];
      }
      else
      {
        const auto* source_cstate = vd->state->as<ompl::base::ElevationStateSpace::StateType>();
        const auto* source_so2 = source_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto* source_xyzv = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        source_point.x = source_xyzv->values[0];
        source_point.y = source_xyzv->values[1];
        source_point.z = source_xyzv->values[2];

        const auto* target_cstate = b->state->as<ompl::base::ElevationStateSpace::StateType>();
        const auto* target_so2 = target_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto* target_xyzv = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        target_point.x = target_xyzv->values[0];
        target_point.y = target_xyzv->values[1];
        target_point.z = target_xyzv->values[2];
      }

      line_stripes.points.push_back(source_point);
      line_stripes.colors.push_back(color);
      line_stripes.points.push_back(target_point);
      line_stripes.colors.push_back(color);
    }
  }

  marker_array.markers.push_back(spheres);
  marker_array.markers.push_back(line_stripes);

  publisher->publish(marker_array);
}

void ompl::control::CP::visualizePath(
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

std_msgs::msg::ColorRGBA ompl::control::CP::getColor(std::string& color)
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
