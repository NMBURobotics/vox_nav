// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#include <vox_nav_misc/stick_imu_to_inertial_frame.hpp>

namespace vox_nav_misc
{

stick_imu_to_inertial_frame::stick_imu_to_inertial_frame() : rclcpp::Node("stick_imu_to_inertial_frame_rclcpp_node")
{
  // init tf buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&stick_imu_to_inertial_frame::imuCallback, this, std::placeholders::_1));

  // inform user the node has started
  RCLCPP_INFO(get_logger(), "stick_imu_to_inertial_frame has started.");
}

stick_imu_to_inertial_frame::~stick_imu_to_inertial_frame()
{
  // inform user the node has shutdown
  RCLCPP_INFO(get_logger(), "stick_imu_to_inertial_frame has shutdown.");
}

void stick_imu_to_inertial_frame::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // get the transfrom from map -> base_link
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  // get uaw from transform_stamped
  tf2::Quaternion q;
  tf2::fromMsg(transform_stamped.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // get roll and pitch from imu
  tf2::Quaternion q_imu;
  tf2::fromMsg(msg->orientation, q_imu);
  double roll_imu, pitch_imu, yaw_imu;
  tf2::Matrix3x3(q_imu).getRPY(roll_imu, pitch_imu, yaw_imu);

  // construct the correct quaternion from the roll and pitch of the imu and the yaw from the transform
  tf2::Quaternion q_corrected;
  q_corrected.setRPY(roll_imu, pitch_imu, 0);

  // construct the corrected imu message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = msg->header;
  imu_msg.orientation = tf2::toMsg(q_corrected);
  imu_msg.angular_velocity = msg->angular_velocity;
  imu_msg.linear_acceleration = msg->linear_acceleration;

  // publish the corrected imu message
  corrected_imu_pub_->publish(imu_msg);
}

}  // namespace vox_nav_misc

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vox_nav_misc::stick_imu_to_inertial_frame>());
  rclcpp::shutdown();
  return 0;
}
