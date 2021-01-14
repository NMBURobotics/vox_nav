//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef BOTANBOT_GAZEBO__GAZEBO_ROS_IMU_HPP_
#define BOTANBOT_GAZEBO__GAZEBO_ROS_IMU_HPP_

#include <mutex>
#include <string>

#include "gazebo-11/gazebo/common/Plugin.hh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_model.hpp"

namespace gazebo
{
class GazeboRosIMU : public ModelPlugin
{
public:
  /// \brief Constructor
  GazeboRosIMU();

  /// \brief Destructor
  virtual ~GazeboRosIMU();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world_;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link_;

  /// \brief pointer to ros node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  /// \brief ros message
  sensor_msgs::msg::Imu imuMsg_;

  /// \brief store link name
  std::string link_name_;

  /// \brief frame id
  std::string frame_id_;

  /// \brief topic name
  std::string topic_;

  ignition::math::Pose3d offset_;

  /// \brief Sensor models
  SensorModel3 accelModel_;
  SensorModel3 rateModel_;
  SensorModel yawModel_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  boost::mutex lock;

  ignition::math::Quaterniond orientation_;
  ignition::math::Vector3d velocity_;
  ignition::math::Vector3d accel_;
  ignition::math::Vector3d rate_;
  ignition::math::Vector3d gravity_;

  /// \brief for setting ROS name space
  std::string namespace_;

  // UpdateTimer updateTimer;
  gazebo::event::ConnectionPtr updateConnection_;
  gazebo::common::Time last_update_time_;
};
}  // namespace gazebo

#endif  // BOTANBOT_GAZEBO__GAZEBO_ROS_IMU_HPP_
