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

#include <botanbot_control/controller_core.hpp>

namespace botanbot_control
{
namespace mpc_controller
{
class MPCController : public botanbot_control::ControllerCore
{
public:
  MPCController();
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

  geometry_msgs::msg::TwistStamped  interpolate(double dt);

private:
};

MPCController::MPCController()
{
}

MPCController::~MPCController()
{
}


}  // namespace mpc_controller

}  // namespace botanbot_control
