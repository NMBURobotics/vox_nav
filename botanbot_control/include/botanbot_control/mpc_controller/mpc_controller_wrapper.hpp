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
#include <botanbot_utilities/tf_helpers.hpp>
#include <botanbot_control/mpc_controller/mpc_controller.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

namespace botanbot_control
{
namespace mpc_controller
{
class MPCWrapper : public botanbot_control::ControllerCore
{
public:
  MPCWrapper(rclcpp::Node::SharedPtr parent);
  ~MPCWrapper();

  /**
   * @brief
   *
   * @param name
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> &) override;

  /**
   * @brief Set the Plan object
   *
   * @param path
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief
   *
   * @param pose
   * @param velocity
   * @return geometry_msgs::msg::TwistStamped
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  /**
   * @brief Create a Test Traj object
   *
   * @return nav_msgs::msg::Path
   */
  nav_msgs::msg::Path createTestTraj();

  /**
   * @brief
   *
   */
  void solve();

  /**
   * @brief
   *
   * @param reference_traj
   * @param curr_robot_pose
   * @return int
   */
  int calculate_nearest_state_index(
    nav_msgs::msg::Path reference_traj,
    geometry_msgs::msg::PoseStamped curr_robot_pose);

  /**
   * @brief
   *
   * @param ref_traj
   * @param curr_robot_pose
   * @return std::vector<std::vector<double>>
   */
  std::vector<std::vector<double>> intrpolateTraj(
    const nav_msgs::msg::Path ref_traj,
    geometry_msgs::msg::PoseStamped curr_robot_pose);

  /**
   * @brief
   *
   * @param inValue1
   * @param inValue2
   * @param inPercent
   * @return constexpr double
   */
  constexpr double interp(double inValue1, double inValue2, double inPercent) noexcept
  {
    return inValue1 * (1.0 - inPercent) + inValue2 * inPercent;
  }

  /**
   * @brief
   *
   * @tparam dtype
   * @param inX
   * @param inXp
   * @param inFp
   * @return std::vector<dtype>
   */
  template<typename dtype>
  std::vector<dtype> interp(
    std::vector<dtype> & inX, std::vector<dtype> & inXp,
    std::vector<dtype> & inFp)
  {
    // do some error checking first
    if (inXp.size() != inFp.size()) {
      std::cerr << ("inXp and inFp need to be the same size().") << std::endl;
    }
    //sort the input inXp and inFp data
    std::sort(inXp.begin(), inXp.end());
    std::sort(inFp.begin(), inFp.end());
    std::sort(inX.begin(), inX.end());
    std::vector<dtype> returnArray(inX.size(), 1.0);
    int currXpIdx = 0;
    int currXidx = 0;
    while (currXidx < inX.size()) {

      if (inXp[currXpIdx] <= inX[currXidx] &&
        inX[currXidx] <= inXp[currXpIdx + 1])
      {
        const double percent = static_cast<double>(inX[currXidx] - inXp[currXpIdx]) /
          static_cast<double>(inXp[currXpIdx + 1] - inXp[currXpIdx]);
        returnArray[currXidx++] = interp(
          inFp[currXpIdx], inFp[currXpIdx + 1],
          percent);
      } else {
        ++currXpIdx;
      }
    }
    return returnArray;
  }

  void publihTestTraj();

private:
  // RCLCPP node
  rclcpp::Node::SharedPtr node_;

  // ROS2 oublisher to publish velocity commands , for maual robot jogging
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path reference_traj_;
  geometry_msgs::msg::Twist twist_;
  rclcpp::Time previous_time_;
  MPCController mpc_controller_;
  SolutionResult::control_input_t previous_control_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;


};


}  // namespace mpc_controller

}  // namespace botanbot_control
