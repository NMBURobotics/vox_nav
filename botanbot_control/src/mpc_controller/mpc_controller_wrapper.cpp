// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#include <nav_msgs/msg/path.hpp>
#include <botanbot_control/mpc_controller/mpc_controller_wrapper.hpp>


using namespace std;

namespace botanbot_control
{
namespace mpc_controller
{

MPCWrapper::MPCWrapper(rclcpp::Node::SharedPtr parent)
{
  node_ = parent;

  cmd_vel_publisher_ =
    node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/global", rclcpp::SystemDefaultsQoS(),
    std::bind(&MPCWrapper::globalOdometryCallback, this, std::placeholders::_1));

  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MPCWrapper::timerCallback, this));

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::PoseStamped curr_robot_pose;

  while (!tf_buffer_->canTransform("base_link", "map", rclcpp::Time(0))) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(node_->get_logger(), "Waiting for Transforms to be available!");
  }

  pose.header.frame_id = "map";
  pose.header.stamp.nanosec = node_->now().nanoseconds();
  pose.pose.position.x = -1.0;
  pose.pose.position.y = -1.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -2.0;
  pose.pose.position.y = -2.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -3.0;
  pose.pose.position.y = -3.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -4.0;
  pose.pose.position.y = -4.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -5.0;
  pose.pose.position.y = -5.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -6.0;
  pose.pose.position.y = -6.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -7.0;
  pose.pose.position.y = -7.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -8.0;
  pose.pose.position.y = -8.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -9.0;
  pose.pose.position.y = -9.0;
  ref_traj.poses.push_back(pose);
  pose.pose.position.x = -10.0;
  pose.pose.position.y = -10.0;
  ref_traj.poses.push_back(pose);
  previous_time_ = node_->now();
}

MPCWrapper::~MPCWrapper()
{
}

void MPCWrapper::globalOdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  latest_recived_odom_ = *msg;
}

void MPCWrapper::timerCallback()
{
  rclcpp::Duration transfrom_tolerance(std::chrono::seconds(1));

  nav_msgs::msg::Path ref_traj_base_link;
  for (auto && path_pose_map : ref_traj.poses) {
    geometry_msgs::msg::PoseStamped path_pose_base_link;
    path_pose_map.header.frame_id = "map";
    path_pose_map.header.stamp.nanosec = node_->now().nanoseconds();
    path_pose_base_link.header.frame_id = "base_link";
    path_pose_base_link.header.stamp.nanosec = node_->now().nanoseconds();
    botanbot_utilities::transformPose(
      tf_buffer_,
      "base_link", path_pose_map, path_pose_base_link,
      transfrom_tolerance);
    ref_traj_base_link.poses.push_back(path_pose_base_link);
  }

  geometry_msgs::msg::PoseStamped curr_steering, curr_robot_pose;
  if (!botanbot_utilities::getCurrentPose(
      curr_steering, *tf_buffer_, "base_link", "front_left_wheel", 0.1))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current curr_steering is not available.");
  }
  if (!botanbot_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", 0.1))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
  }

  tf2::Quaternion q;
  tf2::fromMsg(curr_steering.pose.orientation, q);
  double roll, pitch, steering_angle, psi;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, steering_angle);

  double dt = node_->now().seconds() - previous_time_.seconds();
  // reject if step is too long
  if (dt > 0.5) {
    previous_time_ = node_->now();
    RCLCPP_WARN(node_->get_logger(), "A large step found but gonna igore it ");
    dt = node_->now().seconds() - previous_time_.seconds();
  }

  double target_speed = 1.0;
  double current_speed = latest_recived_odom_.twist.twist.linear.x;
  double current_steering = (steering_angle - 1.5706);


  std::vector<double> x_ref, y_ref, psi_ref, v_ref;
  for (auto && i : ref_traj_base_link.poses) {
    x_ref.push_back(i.pose.position.x);
    y_ref.push_back(i.pose.position.y);

    tf2::fromMsg(i.pose.orientation, q);
    tf2::Matrix3x3 mpp(q);
    double r, p, ref_psi;
    mpp.getRPY(roll, pitch, ref_psi);
    psi_ref.push_back(ref_psi);
    v_ref.push_back(1.0);
  }

  mpc_.updateInitialCondition(
    0,
    0,
    0,
    1.0);
  mpc_.updateReference(x_ref, y_ref, psi_ref, v_ref);
  mpc_.updatePreviousInput(previous_control_.first, previous_control_.second);

  SolutionResult res = mpc_.solve();

  twist.linear.x += res.control_input.first * (dt);
  double kMAX_SPEED = 1.0;

  if (twist.linear.x > kMAX_SPEED) {
    twist.linear.x = kMAX_SPEED;
  } else if (twist.linear.x < -kMAX_SPEED) {
    twist.linear.x = -kMAX_SPEED;
  }
  // twist.angular.z = twist.linear.x * res.control_input.second / 1.32;
  twist.angular.z = res.control_input.second;

  cmd_vel_publisher_->publish(twist);

  previous_time_ = node_->now();
  previous_control_ = res.control_input;

  std::cout << "acceleration cmd " << res.control_input.first << std::endl;
  std::cout << "steering angle " << res.control_input.second << std::endl;
  std::cout << "Solver took ms: " << res.solve_time_ms << std::endl;
}

void MPCWrapper::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> &
  /*const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &*/)
{

}

void MPCWrapper::solve()
{

}
geometry_msgs::msg::TwistStamped MPCWrapper::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  return geometry_msgs::msg::TwistStamped();
}

void MPCWrapper::setPlan(const nav_msgs::msg::Path & path)
{

}

void MPCWrapper::setSpeedLimit(const double & speed_limit)
{

}


}  // namespace mpc_controller

}  // namespace botanbot_control


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mpc");
  botanbot_control::mpc_controller::MPCWrapper mpc(node);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
