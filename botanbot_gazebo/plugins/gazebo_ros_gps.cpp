//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// Modification Copyright (c) 2020, Fetullah Atas
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

#include "gazebo_ros_gps.hpp"
#include "gazebo/physics/physics.hh"

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0 / 298.257223563;
static const double excentrity2 = 2 * flattening - flattening * flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE = 0.0;

namespace gazebo
{

GazeboRosGps::GazeboRosGps()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  RCLCPP_ERROR(
    node_->get_logger(), "Shutting down Gazebo GPS node bye. ",
    link_name_.c_str());
}

// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();
  node_ = rclcpp::Node::make_shared("GazeboGPSNode");

  // load parameters
  if (!_sdf->HasElement("robotNamespace")) {
    namespace_.clear();
  } else {
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  }

  if (!_sdf->HasElement("bodyName")) {
    link_ = _model->GetLink();
    link_name_ = link_->GetName();
  } else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link_ = _model->GetLink(link_name_);
  }

  if (!link_) {
    RCLCPP_ERROR(
      node_->get_logger(), "GazeboRosGps plugin error: bodyName: %s does not exist\n",
      link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "gps";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_ = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_ = DEFAULT_REFERENCE_HEADING * M_PI / 180.0;
  reference_altitude_ = DEFAULT_REFERENCE_ALTITUDE;

  fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix_.status.service = 0;

  if (_sdf->HasElement("frameId")) {
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();
  }

  if (_sdf->HasElement("topicName")) {
    fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  }

  if (_sdf->HasElement("velocityTopicName")) {
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();
  }

  if (_sdf->HasElement("referenceLatitude")) {
    _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);
  }

  if (_sdf->HasElement("referenceLongitude")) {
    _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
  }

  if (_sdf->HasElement("referenceHeading")) {
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_)) {
      reference_heading_ *= M_PI / 180.0;
    }
  }

  if (_sdf->HasElement("referenceAltitude")) {
    _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
  }

  if (_sdf->HasElement("status")) {
    int status = fix_.status.status;
    if (_sdf->GetElement("status")->GetValue()->Get(status)) {
      fix_.status.status = static_cast<sensor_msgs::msg::NavSatStatus::_status_type>(status);
    }
  }

  if (_sdf->HasElement("service")) {
    unsigned int service = fix_.status.service;
    if (_sdf->GetElement("service")->GetValue()->Get(service)) {
      fix_.status.service = static_cast<sensor_msgs::msg::NavSatStatus::_service_type>(service);
    }
  }

  fix_.header.frame_id = frame_id_;
  velocity_.header.frame_id = frame_id_;

  position_error_model_.Load(_sdf);
  velocity_error_model_.Load(_sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 /
    (1.0 - excentrity2 * sin(reference_latitude_ * M_PI / 180.0) *
    sin(reference_latitude_ * M_PI / 180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_ = prime_vertical_radius * cos(reference_latitude_ * M_PI / 180.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  last_update_time_ = world_->SimTime();

  fix_publisher_ =
    node_->create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
  velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    velocity_topic_,
    10);
  Reset();

  this->updateConnection_ =
    event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosGps::OnUpdate, this));

}

void GazeboRosGps::Reset()
{
  last_update_time_ = world_->SimTime();
}

/*void GazeboRosGps::dynamicReconfigureCallback(GNSSConfig & config, uint32_t level)
{
  using sensor_msgs::msg::NavSatStatus;
  if (level == 1) {
    if (!config.STATUS_FIX) {
      fix_.status.status = NavSatStatus::STATUS_NO_FIX;
    } else {
      fix_.status.status = (config.STATUS_SBAS_FIX ? NavSatStatus::STATUS_SBAS_FIX : 0) |
        (config.STATUS_GBAS_FIX ? NavSatStatus::STATUS_GBAS_FIX : 0);
    }
    fix_.status.service = (config.SERVICE_GPS ? NavSatStatus::SERVICE_GPS : 0) |
      (config.SERVICE_GLONASS ? NavSatStatus::SERVICE_GLONASS : 0) |
      (config.SERVICE_COMPASS ? NavSatStatus::SERVICE_COMPASS : 0) |
      (config.SERVICE_GALILEO ? NavSatStatus::SERVICE_GALILEO : 0);
  } else {
    config.STATUS_FIX = (fix_.status.status != NavSatStatus::STATUS_NO_FIX);
    config.STATUS_SBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_SBAS_FIX);
    config.STATUS_GBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_GBAS_FIX);
    config.SERVICE_GPS = (fix_.status.service & NavSatStatus::SERVICE_GPS);
    config.SERVICE_GLONASS = (fix_.status.service & NavSatStatus::SERVICE_GLONASS);
    config.SERVICE_COMPASS = (fix_.status.service & NavSatStatus::SERVICE_COMPASS);
    config.SERVICE_GALILEO = (fix_.status.service & NavSatStatus::SERVICE_GALILEO);
  }
}*/

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::OnUpdate()
{
  common::Time sim_time = world_->SimTime();
  double dt = (world_->SimTime() - last_update_time_).Double();

  ignition::math::Pose3d pose = link_->WorldPose();

  ignition::math::Vector3d velocity = velocity_error_model_(link_->WorldLinearVel(), dt);
  ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);

  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  position_error_model_.setCurrentDrift(
    position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

  fix_.header.stamp = node_->now();
  velocity_.header.stamp = fix_.header.stamp;

  fix_.latitude = reference_latitude_ +
    ( cos(reference_heading_) * position.X() + sin(reference_heading_) * position.Y()) /
    radius_north_ * 180.0 / M_PI;
  fix_.longitude = reference_longitude_ -
    (-sin(reference_heading_) * position.X() + cos(reference_heading_) * position.Y()) /
    radius_east_ * 180.0 / M_PI;
  fix_.altitude = reference_altitude_ + position.Z();
  velocity_.vector.x = cos(reference_heading_) * velocity.X() + sin(reference_heading_) *
    velocity.Y();
  velocity_.vector.y = -sin(reference_heading_) * velocity.X() + cos(reference_heading_) *
    velocity.Y();
  velocity_.vector.z = velocity.Z();

  fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.X() * position_error_model_.drift.X() +
    position_error_model_.gaussian_noise.X() * position_error_model_.gaussian_noise.X();
  fix_.position_covariance[4] = position_error_model_.drift.Y() * position_error_model_.drift.Y() +
    position_error_model_.gaussian_noise.Y() * position_error_model_.gaussian_noise.Y();
  fix_.position_covariance[8] = position_error_model_.drift.Z() * position_error_model_.drift.Z() +
    position_error_model_.gaussian_noise.Z() * position_error_model_.gaussian_noise.Z();

  fix_publisher_->publish(fix_);
  velocity_publisher_->publish(velocity_);
  last_update_time_ = world_->SimTime();
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

}  // namespace gazebo
