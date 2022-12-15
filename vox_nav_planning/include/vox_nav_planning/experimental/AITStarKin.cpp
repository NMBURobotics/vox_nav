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
  while (const base::State * goal = pis_.nextGoal()) {
    si_->copyState(goal_state, goal);
  }

  // get start node and state,push  the node inton nn_ as well
  auto * start_start = si_->allocState();
  VertexProperty * start_vertex = new VertexProperty();

  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_start, st);
    start_vertex->state = start_start;
    nn_->add(start_vertex);
  }

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n", getName().c_str(),
    nn_->size());

  std::vector<ompl::base::State *> samples = generateBatchofSamples(100, true);

  GraphT g;
  WeightMap weightmap = get(boost::edge_weight, g);

  for (auto && i : samples) {
    vertex_descriptor this_vertex_descriptor = boost::add_vertex(g);
    g[this_vertex_descriptor].state = (i);
    g[this_vertex_descriptor].state_label = (reinterpret_cast<std::uintptr_t>(i));
    g[this_vertex_descriptor].id = (this_vertex_descriptor);
    VertexProperty * this_vertex_property = new VertexProperty();
    this_vertex_property->state = i;
    this_vertex_property->state_label = (reinterpret_cast<std::uintptr_t>(i));
    this_vertex_property->id = (this_vertex_descriptor);
    nn_->add(this_vertex_property);
  }

  std::vector<VertexProperty *> vertices_in_nn;
  if (nn_) {
    nn_->list(vertices_in_nn);
  }

  double radius = 0.3;
  for (auto && i : vertices_in_nn) {
    std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;
    nn_->nearestR(i, radius, nbh);

    for (auto && nb : nbh) {
      vertex_descriptor u = i->id;
      vertex_descriptor v = nb->id;
      edge_descriptor e; bool edge_added;
      boost::tie(e, edge_added) = boost::add_edge(u, v, g);
    }

  }

  for (auto vd : boost::make_iterator_range(vertices(g))) {
    std::cout << "Vertex descriptor #" << vd
              << " state:" << g[vd].state
              << "\n";
  }

  //Write graph to file
  std::ofstream f("/home/atas/test.dot");
  boost::dynamic_properties dp;
  boost::write_graphviz_dp(f, g, dp);
  f.close();

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

std::vector<ompl::base::State *> & ompl::control::AITStarKin::generateBatchofSamples(
  int batch_size,
  bool use_valid_sampler)
{
  std::vector<ompl::base::State *> samples;
  samples.reserve(batch_size);

  do{

    // Create a new vertex.
    samples.emplace_back(si_->allocState());
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

  // Update the batch id.
  return samples;
}
