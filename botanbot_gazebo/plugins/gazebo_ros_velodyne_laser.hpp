/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  Modification Copyright (c) 2020, Fetullah Atas.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_VELODYNE_LASER_HPP_
#define BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_VELODYNE_LASER_HPP_


#include <sdf/Param.hh>
#include <ignition/math/Angle.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_ros/node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

namespace gazebo
{

class GazeboRosVelodyneLaser : public SensorPlugin
{
  struct ScanPattern
  {
    struct Sample
    {
      Sample() = default;

      Sample(double _index, ignition::math::Angle _angle)
      : index(_index), angle(_angle)
      {
      }

      double index{0.};
      ignition::math::Angle angle{0.0};
    };
    std::vector<Sample> horizontal_samples;
    std::vector<Sample> vertical_samples;
  };

  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor

public:
  GazeboRosVelodyneLaser();

  /// \brief Destructor

public:
  ~GazeboRosVelodyneLaser();

  /// \brief Load the plugin
  /// \param take in SDF root element

public:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Load scan pattern
  /// \param _parent Parent (gpu) ray sensor
  /// \param _sdf Root sensor plugin SDF element

private:
  void LoadScanPattern(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Sample angle interval evenly
  /// \param _min_angle Angle interval lower bound
  /// \param _max_angle Angle interval upper bound
  /// \param _sample_count Amount of samples to take

private:
  std::vector<ScanPattern::Sample> SampleAngleIntervalEvenly(
    ignition::math::Angle _min_angle,
    ignition::math::Angle _max_angle,
    int _sample_count);

  /// \brief Sample angle interval as configured.
  /// \param _sdf Root SDF element holding configuration
  /// \param _min_angle Angle interval lower bound
  /// \param _max_angle Angle interval upper bound
  /// \param _sample_count Amount of samples to take

private:
  std::vector<ScanPattern::Sample> SampleAngleInterval(
    sdf::ElementPtr _sdf, ignition::math::Angle _min_angle,
    ignition::math::Angle _max_angle, int _sample_count);

  /// \brief Subscribe on-demand

private:
  void ConnectCb();

  /// \brief The parent ray sensor

private:
  sensors::SensorPtr parent_sensor_;

private:
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief ROS publisher

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  /// \brief topic name

private:
  std::string topic_name_;

  /// \brief frame transform name, should match link name

private:
  std::string frame_name_;

  /// \brief the intensity beneath which points will be filtered

private:
  double min_intensity_;

  /// \brief the expected ray casting pattern for this laser.

private:
  ScanPattern scan_pattern_;

  /// \brief Minimum range to publish

private:
  double min_range_;

  /// \brief Maximum range to publish

private:
  double max_range_;

  /// \brief Gaussian noise

private:
  double gaussian_noise_;

  /// \brief Gaussian noise generator

private:
  static double gaussianKernel(double mu, double sigma)
  {
    // using Box-Muller transform to generate two independent
    // standard normally distributed normal variables
    // see wikipedia
    double U = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    // normalized uniform random variable
    double V = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    // normalized uniform random variable
    return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
  }

  /// \brief For setting ROS name space

private:
  std::string robot_namespace_;

  // Subscribe to gazebo laserscan

private:
  gazebo::transport::NodePtr gazebo_node_;

private:
  gazebo::transport::SubscriberPtr sub_;

private:
  void OnScan(const ConstLaserScanStampedPtr & _msg);
};

}  // namespace gazebo

#endif  // BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_VELODYNE_LASER_HPP_
