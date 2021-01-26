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
#include <botanbot_control/mpc_controller/mpc_controller.hpp>


using namespace std;

namespace botanbot_control
{
namespace mpc_controller
{

MPCController::MPCController()
{
  vector<vector<double>> control_output;

  nav_msgs::msg::Odometry robot_odom;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;

  pose.pose.position.x = 1.0;
  path.poses.push_back(pose);
  pose.pose.position.x = 2.0;
  path.poses.push_back(pose);
  pose.pose.position.x = 3.0;
  path.poses.push_back(pose);
  pose.pose.position.x = 4.0;
  path.poses.push_back(pose);

  // j[1] is the data JSON object
  vector<double> ptsx;
  vector<double> ptsy;
  for (auto && path_pose : path.poses) {
    ptsx.push_back(path_pose.pose.position.x);
    ptsy.push_back(path_pose.pose.position.y);

    double px = path_pose.pose.position.x;
    double py = path_pose.pose.position.x;
    double psi = 0;
    double v = robot_odom.twist.twist.linear.x;

    // simulate latency compensation
    const double latency = 0.1;
    px = px + v * cos(psi) * latency;
    py = py + v * sin(psi) * latency;

    //converting to car's local coordinate system
    Eigen::VectorXd xvals(ptsx.size());
    Eigen::VectorXd yvals(ptsx.size());
    Eigen::MatrixXd translation(2, 2);
    translation << cos(-psi), -sin(-psi),
      sin(-psi), cos(-psi);
    Eigen::VectorXd pnt(2);
    Eigen::VectorXd local_pnt(2);

    for (int i = 0; i < ptsx.size(); i++) {
      // convert to vehicle coordinates
      pnt << ptsx[i] - px, ptsy[i] - py;
      local_pnt = translation * pnt;
      xvals[i] = local_pnt[0];
      yvals[i] = local_pnt[1];
      // std::cout <<"i: "<< i<< "lcl: " << local_pnt[0] <<", "<< local_pnt[1] << std::endl;
    }

    auto coeffs = polyfit(xvals, yvals, 3);

    // acado setting
    // because current pos is in local coordinate, x = y = psi = 0
    vector<double> cur_state = {0, 0, v * MPH2MS, 0, 0};

    double ref_v = 20;     // m/s
    if (flg_init == false) {
      printf("-------  initialized the acado ------- \n");
      control_output = init_acado();
      flg_init = true;
    }
    vector<double> predicted_states = motion_prediction(cur_state, control_output);
    vector<double> ref_states = calculate_ref_states(coeffs, ref_v);
    control_output = run_mpc_acado(predicted_states, ref_states, control_output);

    printf(
      "steer value: %lf,   throttle : %lf \n",
      -control_output[1][0], control_output[0][0]);

    // Display the MPC predicted trajectory
    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;

    // .. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Green line
    for (int i = 0; i < predicted_states.size(); i++) {
      if (i % NY == 0) {
        mpc_x_vals.push_back(predicted_states[i]);
      } else if (i % NY == 1) {
        mpc_y_vals.push_back(predicted_states[i]);
      }
    }

    // Display the waypoints/reference line
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Yellow line
    for (int x = 0; x < 100; x = x + 5) {
      double y = polyeval(coeffs, x);
      next_x_vals.push_back(x);
      next_y_vals.push_back(y);
    }

  }


}

MPCController::~MPCController()
{
}

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> &
  /*const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &*/)
{

}

void MPCController::solve()
{

}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  return geometry_msgs::msg::TwistStamped();
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{

}

void MPCController::setSpeedLimit(const double & speed_limit)
{

}


}  // namespace mpc_controller

}  // namespace botanbot_control


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mpc");
  botanbot_control::mpc_controller::MPCController mpc;
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
