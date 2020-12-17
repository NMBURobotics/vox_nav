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

#include "botanbot_gazebo/gazebo_ros_imu.hpp"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"


namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  node_ = rclcpp::Node::make_shared("GazeboGPSNode");

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  } else {
    namespace_.clear();
  }

  if (_sdf->HasElement("bodyName")) {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link_ = _model->GetLink(link_name_);
  } else {
    link_ = _model->GetLink();
    link_name_ = link_->GetName();
  }

  // assert that the body by link_name_ exists
  if (!link_) {
    RCLCPP_ERROR(
      node_->get_logger(), "GazeboRosIMU plugin error: bodyName: %s does not exist\n",
      link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link_name_;
  topic_ = "imu/data";

  if (_sdf->HasElement("frameId")) {
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();
  }

  if (_sdf->HasElement("topicName")) {
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  }

  accelModel_.Load(_sdf, "accel");
  rateModel_.Load(_sdf, "rate");
  yawModel_.Load(_sdf, "yaw");

  // also use old configuration variables from gazebo_ros_imu
  if (_sdf->HasElement("gaussianNoise")) {
    double gaussianNoise;
    if (_sdf->GetElement("gaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
      accelModel_.gaussian_noise = gaussianNoise;
      rateModel_.gaussian_noise = gaussianNoise;
    }
  }

  if (_sdf->HasElement("xyzOffset")) {
    this->offset_.Pos() = _sdf->Get<ignition::math::Vector3d>("xyzOffset");
  } else {
    RCLCPP_INFO(node_->get_logger(), "imu plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }

  if (_sdf->HasElement("rpyOffset")) {
    this->offset_.Rot() = _sdf->Get<ignition::math::Quaterniond>("rpyOffset");
  } else {
    RCLCPP_INFO(node_->get_logger(), "imu plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(0, 0, 0);

  }

  imuMsg_.angular_velocity_covariance[0] = rateModel_.gaussian_noise.X() *
    rateModel_.gaussian_noise.X();
  imuMsg_.angular_velocity_covariance[4] = rateModel_.gaussian_noise.Y() *
    rateModel_.gaussian_noise.Y();
  imuMsg_.angular_velocity_covariance[8] = rateModel_.gaussian_noise.Z() *
    rateModel_.gaussian_noise.Z();
  imuMsg_.linear_acceleration_covariance[0] = accelModel_.gaussian_noise.X() *
    accelModel_.gaussian_noise.X();
  imuMsg_.linear_acceleration_covariance[4] = accelModel_.gaussian_noise.Y() *
    accelModel_.gaussian_noise.Y();
  imuMsg_.linear_acceleration_covariance[8] = accelModel_.gaussian_noise.Z() *
    accelModel_.gaussian_noise.Z();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
    return;
  }


  // if topic name specified as empty, do not publish (then what is this plugin good for?)
  if (!topic_.empty()) {
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_, 10);
  }

  Reset();
  last_update_time_ = world_->SimTime();

  this->updateConnection_ =
    event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosIMU::Update, this));
}

void GazeboRosIMU::Reset()
{
  orientation_ = ignition::math::Quaterniond();
  velocity_ = 0.0;
  accel_ = 0.0;
  accelModel_.reset();
  rateModel_.reset();
  yawModel_.reset();
  last_update_time_ = world_->SimTime();
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::Update()
{

  common::Time cur_time = world_->SimTime();

  double dt = (world_->SimTime() - last_update_time_).Double();

  //boost::mutex::scoped_lock scoped_lock(lock);


  ignition::math::Pose3d pose = link_->WorldPose();
  // ignition::math::Vector3d pos = pose.pos + this->offset_.pos;
  ignition::math::Quaterniond rot = this->offset_.Rot() * pose.Rot();

  rot.Normalize();

  gravity_ = world_->Gravity();
  double gravity_length = gravity_.Length();

  ignition::math::Vector3d temp = link_->WorldLinearVel(); // get velocity in world frame

  if (dt > 0.0) {accel_ = rot.RotateVectorReverse((temp - velocity_) / dt - gravity_);}
  velocity_ = temp;

  // calculate angular velocity from delta quaternion
  // note: link->GetRelativeAngularVel() sometimes return nan?
  // rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
  ignition::math::Quaterniond delta = this->orientation_.Inverse() * rot;

  this->orientation_ = rot;
  if (dt > 0.0) {
    rate_ = this->offset_.Rot().Inverse() *
      (2.0 *
      acos(
        std::max(
          std::min(delta.W(), 1.0),
          -1.0)) * ignition::math::Vector3d(delta.X(), delta.Y(), delta.Z()).Normalize() / dt);

  }

  // update sensor models
  accel_ = accelModel_(accel_, dt);
  rate_ = rateModel_(rate_, dt);
  yawModel_.update(dt);


  // apply accelerometer and yaw drift error to orientation (pseudo AHRS)
  ignition::math::Vector3d accelDrift = pose.Rot().RotateVector(accelModel_.getCurrentBias());
  double yawError = yawModel_.getCurrentBias();
  ignition::math::Quaterniond orientationError(
    ignition::math::Quaterniond(cos(yawError / 2), 0.0, 0.0, sin(yawError / 2)) *                                         // yaw error
    ignition::math::Quaterniond(
      1.0, 0.5 * accelDrift.Y() / gravity_length,
      0.5 * -accelDrift.X() / gravity_length, 0.0)                                                                        // roll and pitch error
  );

  orientationError.Normalize();
  rot = orientationError * rot;

  // copy data into pose message
  imuMsg_.header.frame_id = frame_id_;
  imuMsg_.header.stamp.sec = cur_time.sec;
  imuMsg_.header.stamp.nanosec = cur_time.nsec;


  imuMsg_.orientation.x = rot.X();
  imuMsg_.orientation.y = rot.Y();
  imuMsg_.orientation.z = rot.Z();
  imuMsg_.orientation.w = rot.W();

  imuMsg_.angular_velocity.x = rate_.X();
  imuMsg_.angular_velocity.y = rate_.Y();
  imuMsg_.angular_velocity.z = rate_.Z();

  imuMsg_.linear_acceleration.x = accel_.X();
  imuMsg_.linear_acceleration.y = accel_.Y();
  imuMsg_.linear_acceleration.z = accel_.Z();


  // fill in covariance matrix
  imuMsg_.orientation_covariance[8] = yawModel_.gaussian_noise * yawModel_.gaussian_noise;
  if (gravity_length > 0.0) {
    imuMsg_.orientation_covariance[0] = accelModel_.gaussian_noise.X() *
      accelModel_.gaussian_noise.X() / (gravity_length * gravity_length);
    imuMsg_.orientation_covariance[4] = accelModel_.gaussian_noise.Y() *
      accelModel_.gaussian_noise.Y() / (gravity_length * gravity_length);

  } else {
    imuMsg_.orientation_covariance[0] = -1;
    imuMsg_.orientation_covariance[4] = -1;
  }

  // publish to ros
  imu_publisher_->publish(imuMsg_);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

} // namespace gazebo
