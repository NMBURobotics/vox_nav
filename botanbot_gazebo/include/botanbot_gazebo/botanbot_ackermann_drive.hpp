// Copyright 2019 Open Source Robotics Foundation, Inc.
// Modifications copyright (C) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef BOTANBOT_GAZEBO__BOTANBOT_ACKERMANN_DRIVE_HPP_
#define BOTANBOT_GAZEBO__BOTANBOT_ACKERMANN_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
class BotanbotAckermannDrivePrivate;

/// A ackermann drive plugin for car like robots. Subscribes to geometry_msgs/twist

/**
  Example Usage:
  \code{.xml}
        <plugin name='botanbot_ackermann_drive' filename='libbotanbot_ackermann_drive.so'>
            <ros>
                <namespace/>
                <remapping>cmd_vel:=cmd_vel</remapping>
                <remapping>odom:=odom</remapping>
                <remapping>distance:=distance</remapping>
            </ros>
            <update_rate>100.0</update_rate>
            <front_left_joint>front_left_combined_joint</front_left_joint>
            <front_right_joint>front_right_combined_joint</front_right_joint>
            <rear_left_joint>rear_left_joint</rear_left_joint>
            <rear_right_joint>rear_right_joint</rear_right_joint>
            <left_steering_joint>front_left_combined_joint</left_steering_joint>
            <right_steering_joint>front_right_combined_joint</right_steering_joint>
            <steering_wheel_joint>steering_joint</steering_wheel_joint>
            <max_steer>0.6458</max_steer>
            <max_steering_angle>7.85</max_steering_angle>
            <max_speed>20</max_speed>
            <left_steering_pid_gain>1500 0 1</left_steering_pid_gain>
            <left_steering_i_range>0 0</left_steering_i_range>
            <right_steering_pid_gain>1500 0 1</right_steering_pid_gain>
            <right_steering_i_range>0 0</right_steering_i_range>
            <linear_velocity_pid_gain>1000 0 1</linear_velocity_pid_gain>
            <linear_velocity_i_range>0 0</linear_velocity_i_range>
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
            <publish_distance>true</publish_distance>
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_link</robot_base_frame>
        </plugin>
  \endcode
*/
class BotanbotAckermannDrive : public gazebo::ModelPlugin
{
public:
  /// Constructor
  BotanbotAckermannDrive();

  /// Destructor
  ~BotanbotAckermannDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<BotanbotAckermannDrivePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // BOTANBOT_GAZEBO__BOTANBOT_ACKERMANN_DRIVE_HPP_
