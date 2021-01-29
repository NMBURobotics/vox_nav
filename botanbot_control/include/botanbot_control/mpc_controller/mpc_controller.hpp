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


#include <rclcpp/rclcpp.hpp>
#include <botanbot_control/controller_core.hpp>
#include <botanbot_control/mpc_controller/acado.hpp>
#include <botanbot_utilities/tf_helpers.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace botanbot_control
{
namespace mpc_controller
{
class MPCController : public botanbot_control::ControllerCore
{
public:
  MPCController(rclcpp::Node::SharedPtr parent);
  ~MPCController();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> &
    /*const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &*/) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setSpeedLimit(const double & speed_limit) override;

  void solve();


// global variable

  bool flg_init = false;


// Evaluate a polynomial.
  double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


  void  globalOdometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
* @brief periodically called function to publish octomap and its pointcloud data
*
*/
  void timerCallback();

private:
  // RCLCPP node
  rclcpp::Node::SharedPtr node_;
  // ROS2 oublisher to publish velocity commands , for maual robot jogging
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  nav_msgs::msg::Odometry latest_recived_odom_;

  // Used to creted a periodic callback function IOT publish transfrom/octomap/cloud etc.
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path ref_traj;

  // j[1] is the data JSON object
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  Eigen::VectorXd xvals_base_link;
  Eigen::VectorXd yvals_base_link;

  vector<vector<double>> control_output;

  geometry_msgs::msg::Twist twist;


  rclcpp::Time previous_time_;


};


}  // namespace mpc_controller

}  // namespace botanbot_control
