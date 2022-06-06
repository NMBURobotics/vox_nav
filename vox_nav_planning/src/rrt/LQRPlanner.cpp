#include "vox_nav_planning/rrt/LQRPlanner.hpp"


ompl::control::LQRPlanner::LQRPlanner(const SpaceInformationPtr & si)
: base::Planner(si, "LQRPlanner")
{
  specs_.approximateSolutions = true;
  siC_ = si.get();
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
  sampler_.reset();
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


  unsigned iterations = 0;
  std::vector<base::State *> resulting_path;
  resulting_path.push_back(start_state);

  OMPL_INFORM(
    "%s: Starting planning with %u states already in datastructure\n",
    getName().c_str(), resulting_path.size());

  const auto * start_cstate = start_state->as<ompl::base::ElevationStateSpace::StateType>();
  const auto * start_se2 = start_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
  const auto * start_z = start_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  const auto * goal_cstate = goal_state->as<ompl::base::ElevationStateSpace::StateType>();
  const auto * goal_se2 = goal_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
  const auto * goal_z = goal_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  double v_r = 1.5;
  double L = 0.8;

  Eigen::MatrixXd A(2, 2);
  A << 0, -v_r,
    0, 0;

  Eigen::VectorXd B(2);
  B(0) = 0;
  B(1) = -v_r / L;
  B = B.transpose();

  Eigen::MatrixXd Q(2, 2);
  double q1 = 1;
  double q2 = 10;
  double r = 1;
  Q(0, 0) = q1;
  Q(1, 1) = q2;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = r;

  double time = 0.0;

  while (ptc == false) {

    auto * latest_cstate = resulting_path.back()->as<ompl::base::ElevationStateSpace::StateType>();
    const auto * latest_se2 = latest_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    const auto * latest_z = latest_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    double xc = latest_se2->getX();
    double yc = latest_se2->getY();
    double thetac = latest_se2->getYaw();
    double zc = latest_z->values[0];

    double theta_r = std::atan2(
      (yc - start_se2->getY()),
      (xc - start_se2->getX()));

    Eigen::MatrixXd T(3, 3);
    T <<
      -std::cos(theta_r), -std::sin(theta_r), 0,
      std::sin(theta_r), -std::cos(theta_r), 0,
      0, 0, 1;

    Eigen::VectorXd e(3);
    e(0) = xc - goal_se2->getX();
    e(1) = yc - goal_se2->getY();
    e(2) = thetac - theta_r;

    auto Te_dynamics = T * e.transpose();

    Eigen::VectorXd X(2);
    X(0) = Te_dynamics(1);
    X(1) = Te_dynamics(0);

    auto res = lqr_control(A, B, X);
    Eigen::VectorXd U(2);
    U(0) = std::get<0>(res);
    U(1) = std::get<1>(res);

    X = A * X.transpose() + B * U.transpose();

    auto d = std::sqrt(
      std::pow(goal_se2->getX() - xc, 2) +
      std::pow(goal_se2->getY() - yc, 2));

    if (d < 0.2) {
      U(0)  = 0;
      break;
    }

    auto * this_state = si_->allocState();
    auto * this_cstate = start_state->as<ompl::base::ElevationStateSpace::StateType>();

    this_cstate->setSE2(
      xc + U(0) * std::cos(thetac),
      yc + U(0) * std::sin(thetac),
      thetac + (U(0) * std::tan(U(1)) / L)
    );
    this_cstate->setZ(zc);
    resulting_path.push_back(latest_cstate);

    // TIME STEP INCREASE
    time += dt_;

    iterations++;

    // MAX TIME REACHED
    if (time >= max_time_) {break;}
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
    OMPL_WARN("%s: Failed to cretae a plan after %u iterations", getName().c_str(), iterations);
  }

  clear();

  return {solved, approximate};
}

void ompl::control::LQRPlanner::getPlannerData(base::PlannerData & data) const
{
  Planner::getPlannerData(data);
}
