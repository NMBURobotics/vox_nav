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

#include <botanbot_control/mpc_controller/acado.hpp>


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


// global variable

  bool flg_init = false;

// For converting back and forth between radians and degrees.
  constexpr double pi() {return M_PI;}
  double deg2rad(double x) {return x * pi() / 180;}
  double rad2deg(double x) {return x * 180 / pi();}
#define MPH2MS 0.44704

// Evaluate a polynomial.
  double polyeval(Eigen::VectorXd coeffs, double x)
  {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
  {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }

private:
};


}  // namespace mpc_controller

}  // namespace botanbot_control
