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

  std::vector<ompl::base::State *> samples;
  generateBatchofSamples(100, true, samples);

  OMPL_INFORM(
    "%s: We have %d samples\n", getName().c_str(), samples.size());

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

  OMPL_INFORM(
    "%s: Filled G and NN %d \n", getName().c_str(), samples.size());

  std::vector<VertexProperty *> vertices_in_nn;
  if (nn_) {
    nn_->list(vertices_in_nn);
  }

  OMPL_INFORM(
    "%s: aFilled G and NN %d \n", getName().c_str(), samples.size());

  double radius = 0.5;
  for (auto && i : vertices_in_nn) {
    std::vector<ompl::control::AITStarKin::VertexProperty *> nbh;

    nn_->nearestR(i, radius, nbh);

    for (auto && nb : nbh) {
      vertex_descriptor u = i->id;
      vertex_descriptor v = nb->id;
      edge_descriptor e; bool edge_added;

      if (u == v) {
        continue;
      }
      boost::tie(e, edge_added) = boost::add_edge(u, v, g);
    }
  }


  visualization_msgs::msg::MarkerArray marker_array;
  // To make a graph of the supervoxel adjacency,
  // we need to iterate through the supervoxel adjacency multimap

  int index = 0;

  for (auto vd : boost::make_iterator_range(vertices(g))) {
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
    sphere.ns = "rgg";
    sphere.id = index;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position = point;
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;
    sphere.color.a = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;
    marker_array.markers.push_back(sphere);
    index++;
  }

  auto es = boost::edges(g);

  vertex_descriptor u, v;
  for (auto eit = es.first; eit != es.second; ++eit) {

    std::cout << boost::source(*eit, g) << ' ' << boost::target(*eit, g) << std::endl;

    u = boost::source(*eit, g);
    v = boost::target(*eit, g);
    geometry_msgs::msg::Point source_point, target_point;

    const auto * source_cstate = g[u].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * source_se2 = source_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * source_z = source_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    source_point.x = source_se2->getX();
    source_point.y = source_se2->getY();
    source_point.z = source_z->values[0];

    std::cout << source_point.x << ' ' << std::endl;
    std::cout << source_point.y << ' ' << std::endl;

    const auto * target_cstate = g[v].state->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * target_se2 = target_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * target_z = target_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    target_point.x = target_se2->getX();
    target_point.y = target_se2->getY();
    target_point.z = target_z->values[0];

    std::cout << target_point.x << ' ' << std::endl;
    std::cout << target_point.y << ' ' << std::endl;

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.ns = "rgg";
    line_strip.id = index;
    line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.lifetime = rclcpp::Duration::from_seconds(0);
    line_strip.header.stamp = rclcpp::Clock().now();
    line_strip.scale.x = 0.1;
    line_strip.scale.x = 0.1;
    line_strip.scale.y = 0.25;
    line_strip.scale.z = 0.25;
    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;

    std_msgs::msg::ColorRGBA yellow_color;
    yellow_color.r = 1.0;
    yellow_color.g = 1.0;
    yellow_color.a = 1.0;

    line_strip.points.push_back(source_point);
    line_strip.colors.push_back(yellow_color);
    line_strip.points.push_back(target_point);
    line_strip.colors.push_back(yellow_color);

    marker_array.markers.push_back(line_strip);

    index++;
  }

  rrt_nodes_pub_->publish(marker_array);

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
