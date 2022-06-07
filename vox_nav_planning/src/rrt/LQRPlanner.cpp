#include "vox_nav_planning/rrt/LQRPlanner.hpp"


ompl::control::LQRPlanner::LQRPlanner(const SpaceInformationPtr & si)
: base::Planner(si, "LQRPlanner")
{
  specs_.approximateSolutions = true;
}

ompl::control::LQRPlanner::~LQRPlanner()
{
  freeMemory();
}

void ompl::control::LQRPlanner::setup()
{
  base::Planner::setup();

  // ros2 node to publish rrt nodes
  node_ = std::make_shared<rclcpp::Node>("LQRPlanner_rclcpp_node");

  rrt_nodes_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vox_nav/rrtstar/nodes", rclcpp::SystemDefaultsQoS());
}

void ompl::control::LQRPlanner::clear()
{
  Planner::clear();
  freeMemory();
}

void ompl::control::LQRPlanner::freeMemory()
{
}

ompl::base::PlannerStatus ompl::control::LQRPlanner::solve(
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
  auto * start_state = si_->allocState();
  while (const base::State * st = pis_.nextStart()) {
    si_->copyState(start_state, st);
  }

  if (!start_state) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  OMPL_INFORM("%s: Starting planning with", getName().c_str());

  std::vector<base::State *> resulting_path;

  while (ptc == false) {

    compute_LQR_plan(start_state, goal_state, resulting_path);
    if (resulting_path.size() > 2) {break; /*Found a solution with at least some pose*/}

  }

  bool solved = false;
  bool approximate = false;

  if (resulting_path.size() > 2) {
    solved = true;

    /* set the solution path */
    auto path(std::make_shared<PathControl>(si_));
    for (auto i : resulting_path) {
      if (i) {
        path->append(i);
      }
    }
    solved = true;
    pdef_->addSolutionPath(path, approximate, 0.0 /*approxdif*/, getName());
    OMPL_INFORM("Found solution with %u poses", resulting_path.size());
  } else {
    OMPL_WARN("%s: Failed to cretae a plan after", getName().c_str());
  }
  clear();
  return {solved, approximate};
}

void ompl::control::LQRPlanner::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);
}
