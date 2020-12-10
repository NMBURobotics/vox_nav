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

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H

#include "gazebo/common/Plugin.hh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_model.hpp"

/**
 * @brief
 *
 */
namespace gazebo
{

/**
 * @brief
 *
 */
class GazeboRosGps : public ModelPlugin
{
public:
  /**
  * @brief Construct a new Gazebo Ros Gps object
  *
  */
  GazeboRosGps();

  /**
   * @brief Destroy the Gazebo Ros Gps object
   *
   */
  virtual ~GazeboRosGps();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

  typedef hector_gazebo_plugins::GNSSConfig GNSSConfig;

  void dynamicReconfigureCallback(GNSSConfig & config, uint32_t level);

private:
  physics::WorldPtr world_;

  physics::LinkPtr link_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Publisher> fix_publisher_;
  std::shared_ptr<rclcpp::Publisher> velocity_publisher_;

  sensor_msgs::msg::NavSatFix fix_;
  geometry_msgs::msg::Vector3Stamped velocity_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string fix_topic_;
  std::string velocity_topic_;

  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  double reference_altitude_;

  double radius_north_;
  double radius_east_;

  SensorModel3 position_error_model_;
  SensorModel3 velocity_error_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

  boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig>>
  dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
  boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig>> dynamic_reconfigure_server_status_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
