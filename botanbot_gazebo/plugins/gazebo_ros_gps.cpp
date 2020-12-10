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

#include <hector_gazebo_plugins/gazebo_ros_gps.h>
#include <gazebo/physics/physics.hh>

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
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_position_.reset();
  dynamic_reconfigure_server_velocity_.reset();
  dynamic_reconfigure_server_status_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace")) {
    namespace_.clear();
  } else {
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  }

  if (!_sdf->HasElement("bodyName")) {
    link = _model->GetLink();
    link_name_ = link->GetName();
  } else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link) {
    ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_ = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_ = DEFAULT_REFERENCE_HEADING * M_PI / 180.0;
  reference_altitude_ = DEFAULT_REFERENCE_ALTITUDE;

  fix_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
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
      fix_.status.status = static_cast<sensor_msgs::NavSatStatus::_status_type>(status);
    }
  }

  if (_sdf->HasElement("service")) {
    unsigned int service = fix_.status.service;
    if (_sdf->GetElement("service")->GetValue()->Get(service)) {
      fix_.status.service = static_cast<sensor_msgs::NavSatStatus::_service_type>(service);
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
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
        "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

  // setup dynamic_reconfigure servers
  dynamic_reconfigure_server_position_.reset(
    new dynamic_reconfigure::Server<SensorModelConfig>(
      ros
      ::NodeHandle(*node_handle_, fix_topic_ + "/position")));
  dynamic_reconfigure_server_velocity_.reset(
    new dynamic_reconfigure::Server<SensorModelConfig>(
      ros
      ::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
  dynamic_reconfigure_server_status_.reset(
    new dynamic_reconfigure::Server<GNSSConfig>(
      ros::
      NodeHandle(*node_handle_, fix_topic_ + "/status")));
  dynamic_reconfigure_server_position_->setCallback(
    boost::bind(
      &SensorModel3::
      dynamicReconfigureCallback, &position_error_model_, _1, _2));
  dynamic_reconfigure_server_velocity_->setCallback(
    boost::bind(
      &SensorModel3::
      dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
  dynamic_reconfigure_server_status_->setCallback(
    boost::bind(
      &GazeboRosGps::
      dynamicReconfigureCallback, this, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}

void GazeboRosGps::dynamicReconfigureCallback(GazeboRosGps::GNSSConfig & config, uint32_t level)
{
  using sensor_msgs::NavSatStatus;
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
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::Update()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();

  ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
  ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);
#else
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();

  gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
  gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);
#endif

  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  position_error_model_.setCurrentDrift(
    position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = fix_.header.stamp;

#if (GAZEBO_MAJOR_VERSION >= 8)
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

  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.X() * position_error_model_.drift.X() +
    position_error_model_.gaussian_noise.X() * position_error_model_.gaussian_noise.X();
  fix_.position_covariance[4] = position_error_model_.drift.Y() * position_error_model_.drift.Y() +
    position_error_model_.gaussian_noise.Y() * position_error_model_.gaussian_noise.Y();
  fix_.position_covariance[8] = position_error_model_.drift.Z() * position_error_model_.drift.Z() +
    position_error_model_.gaussian_noise.Z() * position_error_model_.gaussian_noise.Z();
#else
  fix_.latitude = reference_latitude_ +
    ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) /
    radius_north_ * 180.0 / M_PI;
  fix_.longitude = reference_longitude_ -
    (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) /
    radius_east_ * 180.0 / M_PI;
  fix_.altitude = reference_altitude_ + position.z;
  velocity_.vector.x = cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
  velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
  velocity_.vector.z = velocity.z;

  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.x * position_error_model_.drift.x +
    position_error_model_.gaussian_noise.x * position_error_model_.gaussian_noise.x;
  fix_.position_covariance[4] = position_error_model_.drift.y * position_error_model_.drift.y +
    position_error_model_.gaussian_noise.y * position_error_model_.gaussian_noise.y;
  fix_.position_covariance[8] = position_error_model_.drift.z * position_error_model_.drift.z +
    position_error_model_.gaussian_noise.z * position_error_model_.gaussian_noise.z;
#endif

  fix_publisher_.publish(fix_);
  velocity_publisher_.publish(velocity_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
