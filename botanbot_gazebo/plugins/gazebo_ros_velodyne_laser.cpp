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

#include "gazebo_ros_velodyne_laser.hpp"

#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <algorithm>
#include <iterator>
#include <sstream>
#include <vector>
#include <string>
#include <limits>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser()
: min_range_(0), max_range_(0), gaussian_noise_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
}

std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample>
GazeboRosVelodyneLaser::SampleAngleIntervalEvenly(
  ignition::math::Angle _min_angle,
  ignition::math::Angle _max_angle,
  int _sample_count)
{
  std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample> samples;
  samples.reserve(_sample_count);
  if (_sample_count > 1) {
    ignition::math::Angle angle_step = (_max_angle - _min_angle) / (_sample_count - 1);
    for (double i = 0; i < _sample_count; i += 1.) {
      samples.emplace_back(i, angle_step * i + _min_angle);
    }
  } else {
    samples.emplace_back(0.0, 0.0);
  }
  return samples;
}

std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample>
GazeboRosVelodyneLaser::SampleAngleInterval(
  sdf::ElementPtr _sdf,
  ignition::math::Angle _min_angle,
  ignition::math::Angle _max_angle,
  int _sample_count)
{
  using ScanPattern = GazeboRosVelodyneLaser::ScanPattern;
  const ignition::math::Angle kEpsilonAngle{1e-6};
  if (_sdf->HasElement("angles")) {
    std::vector<ScanPattern::Sample> samples;
    samples.reserve(_sample_count);
    std::istringstream iss(_sdf->GetElement("angles")->Get<std::string>());
    for (auto it = std::istream_iterator<double>(iss); it != std::istream_iterator<double>();
      ++it)
    {
      ignition::math::Angle angle(*it);
      if (angle > _max_angle + kEpsilonAngle || angle < _min_angle - kEpsilonAngle) {
        RCLCPP_WARN(
          ros_node_->get_logger(),
          "Velodyne laser plugin given a %s scan angle out of bounds: "
          "%.16g (%.16g deg) < %.16g rad (%.16g deg) < %.16g (%.16g deg), ignoring setting.",
          _sdf->GetName().c_str(), _min_angle.Radian(), _min_angle.Degree(),
          angle.Radian(), angle.Degree(), _max_angle.Radian(), _max_angle.Degree());
        return SampleAngleIntervalEvenly(_min_angle, _max_angle, _sample_count);
      }
      angle = angle >= _min_angle ? (angle <= _max_angle ? angle : _max_angle) : _min_angle;
      double index = (_sample_count - 1) * (angle - _min_angle).Radian() /
        (_max_angle - _min_angle).Radian();
      index = index >= 0 ? (index <= (_sample_count - 1) ? index : (_sample_count - 1)) : 0;
      samples.emplace_back(index, angle);
    }
    samples.shrink_to_fit();
    return samples;
  }
  RCLCPP_WARN(
    ros_node_->get_logger(),
    "Velodyne laser plugin not given any %s scan pattern, ignoring setting.",
    _sdf->GetName().c_str());
  return SampleAngleIntervalEvenly(_min_angle, _max_angle, _sample_count);
}

void GazeboRosVelodyneLaser::LoadScanPattern(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensors::GpuRaySensorPtr parent_gpu_ray_sensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(
    _parent);
  if (parent_gpu_ray_sensor) {
    if (_sdf->HasElement("scan")) {
      sdf::ElementPtr scan_elem = _sdf->GetElement("scan");
      if (scan_elem->HasElement("horizontal")) {
        scan_pattern_.horizontal_samples = SampleAngleInterval(
          scan_elem->GetElement("horizontal"), parent_gpu_ray_sensor->AngleMin(),
          parent_gpu_ray_sensor->AngleMax(), parent_gpu_ray_sensor->RangeCount());
      } else {
        scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
          parent_gpu_ray_sensor->AngleMin(),
          parent_gpu_ray_sensor->AngleMax(),
          parent_gpu_ray_sensor->RangeCount());
      }
      if (scan_elem->HasElement("vertical")) {
        scan_pattern_.vertical_samples = SampleAngleInterval(
          scan_elem->GetElement("vertical"), parent_gpu_ray_sensor->VerticalAngleMin(),
          parent_gpu_ray_sensor->VerticalAngleMax(), parent_gpu_ray_sensor->VerticalRangeCount());
      } else {
        scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
          parent_gpu_ray_sensor->VerticalAngleMin(), parent_gpu_ray_sensor->VerticalAngleMax(),
          parent_gpu_ray_sensor->VerticalRangeCount());
      }
    } else {
      scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
        parent_gpu_ray_sensor->VerticalAngleMin(),
        parent_gpu_ray_sensor->VerticalAngleMax(),
        parent_gpu_ray_sensor->VerticalRangeCount());
      scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
        parent_gpu_ray_sensor->AngleMin(), parent_gpu_ray_sensor->AngleMax(),
        parent_gpu_ray_sensor->RangeCount());
    }
    return;
  }

  sensors::RaySensorPtr parent_ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (parent_ray_sensor) {
    if (_sdf->HasElement("scan")) {
      sdf::ElementPtr scan_elem = _sdf->GetElement("scan");
      if (scan_elem->HasElement("horizontal")) {
        scan_pattern_.horizontal_samples = SampleAngleInterval(
          scan_elem->GetElement("horizontal"), parent_ray_sensor->AngleMin(),
          parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
      } else {
        scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
          parent_ray_sensor->AngleMin(),
          parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
      }
      if (scan_elem->HasElement("vertical")) {
        scan_pattern_.vertical_samples = SampleAngleInterval(
          scan_elem->GetElement("vertical"), parent_ray_sensor->VerticalAngleMin(),
          parent_ray_sensor->VerticalAngleMax(), parent_ray_sensor->VerticalRangeCount());
      } else {
        scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
          parent_ray_sensor->VerticalAngleMin(), parent_ray_sensor->VerticalAngleMax(),
          parent_ray_sensor->VerticalRangeCount());
      }
    } else {
      scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
        parent_ray_sensor->VerticalAngleMin(),
        parent_ray_sensor->VerticalAngleMax(),
        parent_ray_sensor->VerticalRangeCount());
      scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
        parent_ray_sensor->AngleMin(), parent_ray_sensor->AngleMax(),
        parent_ray_sensor->RangeCount());
    }

    // Avoid interpolation for CPU based ray sensor if rays can be relocated.
    physics::MultiRayShapePtr laser = parent_ray_sensor->LaserShape();
    if (laser->GetScanResolution() == 1. && laser->GetVerticalScanResolution() == 1. &&
      scan_pattern_.horizontal_samples.size() ==
      static_cast<size_t>(parent_ray_sensor->RangeCount()) &&
      scan_pattern_.vertical_samples.size() ==
      static_cast<size_t>(parent_ray_sensor->VerticalRangeCount()))
    {
      const double ray_min_range = laser->GetMinRange();
      const double ray_max_range = laser->GetMaxRange();
      const ignition::math::Pose3d pose = parent_ray_sensor->Pose();
      for (size_t j = 0; j < scan_pattern_.vertical_samples.size(); ++j) {
        for (size_t i = 0; i < scan_pattern_.horizontal_samples.size(); ++i) {
          const ignition::math::Quaterniond ray_orientation =
            ignition::math::Quaterniond::EulerToQuaternion(
            ignition::math::Vector3d(
              0.0, -scan_pattern_.vertical_samples[j].angle.Radian(),
              scan_pattern_.horizontal_samples[i].angle.Radian()));
          const ignition::math::Vector3d ray_axis =
            pose.Rot() * ray_orientation * ignition::math::Vector3d::UnitX;
          const size_t ray_index = i + j * scan_pattern_.horizontal_samples.size();
          laser->Ray(ray_index)->SetPoints(
            ray_axis * ray_min_range + pose.Pos(),
            ray_axis * ray_max_range + pose.Pos());
          scan_pattern_.horizontal_samples[i].index = i;
          scan_pattern_.vertical_samples[j].index = j;
        }
      }
    }
    return;
  }

  gzthrow(
    "GazeboRosVelodyneLaser controller requires either"
    " a Ray Sensor or a GPU Ray Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading GazeboRosVelodyneLaser\n";

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  // Create node handle
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get the parent sensor
  parent_sensor_ = _parent;

  robot_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  if (!_sdf->HasElement("frameName")) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "Velodyne laser plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  } else {
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("min_range")) {
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Velodyne laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
  }

  if (!_sdf->HasElement("max_range")) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "Velodyne laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
  }

  min_intensity_ = std::numeric_limits<double>::lowest();
  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Velodyne laser plugin missing <min_intensity>, defaults to no clipping");
  } else {
    min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
  }

  LoadScanPattern(parent_sensor_, _sdf);

  if (!_sdf->HasElement("topicName")) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "Velodyne laser plugin missing <topicName>, defaults to /points");
    topic_name_ = "/points";
  } else {
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("gaussianNoise")) {
    RCLCPP_INFO(
      ros_node_->get_logger(), "Velodyne laser plugin missing <gaussianNoise>, defaults to 0.0");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
  }

  if (topic_name_ != "") {
    pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_name_, 10);
  }

  //  ros subscribers present.
  sub_ = gazebo_node_->Subscribe(parent_sensor_->Topic(), &GazeboRosVelodyneLaser::OnScan, this);

  RCLCPP_INFO(ros_node_->get_logger(), "Velodyne %slaser plugin ready");
  gzdbg << "GazeboRosVelodyneLaser LOADED\n";
}

void GazeboRosVelodyneLaser::OnScan(ConstLaserScanStampedPtr & _msg)
{
  const double maxRange = _msg->scan().range_max();
  const double minRange = _msg->scan().range_min();

  const int rangeCount = _msg->scan().count();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);
  const double MIN_INTENSITY = min_intensity_;

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp.sec = _msg->time().sec();
  msg.header.stamp.nanosec = _msg->time().nsec();
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(
    scan_pattern_.horizontal_samples.size() *
    scan_pattern_.vertical_samples.size() *
    POINT_STEP);

  size_t j = 0;
  uint8_t * ptr = msg.data.data();
  using ScanPattern = GazeboRosVelodyneLaser::ScanPattern;
  for (const ScanPattern::Sample & vsample : scan_pattern_.vertical_samples) {
    for (const ScanPattern::Sample & hsample : scan_pattern_.horizontal_samples) {
      double range = INFINITY;
      double intensity = 0.;

      // Interpolate sample range and intensity if need be.
      if (std::rint(hsample.index) == hsample.index &&
        std::rint(vsample.index) == vsample.index)
      {
        int rindex = hsample.index + vsample.index * rangeCount;
        range = _msg->scan().ranges(rindex);
        intensity = _msg->scan().intensities(rindex);
      } else if (std::rint(hsample.index) == hsample.index) {
        int hindex = hsample.index;
        int vindex0 = std::floor(vsample.index);
        int vindex1 = std::ceil(vsample.index);

        int rindex0 = hindex + vindex0 * rangeCount;
        int rindex1 = hindex + vindex1 * rangeCount;

        double range0 = _msg->scan().ranges(rindex0);
        double range1 = _msg->scan().ranges(rindex1);

        range = (vindex1 - vsample.index) * range0 + (vsample.index - vindex0) * range1;

        intensity = (_msg->scan().intensities(rindex0) +
          _msg->scan().intensities(rindex1)) / 2;
      } else if (std::rint(vsample.index) == vsample.index) {
        int vindex = vsample.index;
        int hindex0 = std::floor(hsample.index);
        int hindex1 = std::ceil(hsample.index);

        int rindex0 = hindex0 + vindex * rangeCount;
        int rindex1 = hindex1 + vindex * rangeCount;

        double range0 = _msg->scan().ranges(rindex0);
        double range1 = _msg->scan().ranges(rindex1);

        range = ((hindex1 - hsample.index) * range0 +
          (hsample.index - hindex0) * range1);

        intensity = (_msg->scan().intensities(rindex0) +
          _msg->scan().intensities(rindex1)) / 2;
      } else {
        int hindex0 = std::floor(hsample.index);
        int hindex1 = std::ceil(hsample.index);
        int vindex0 = std::floor(vsample.index);
        int vindex1 = std::ceil(vsample.index);

        int rindex00 = hindex0 + vindex0 * rangeCount;
        int rindex01 = hindex0 + vindex1 * rangeCount;
        int rindex10 = hindex1 + vindex0 * rangeCount;
        int rindex11 = hindex1 + vindex1 * rangeCount;

        double range00 = _msg->scan().ranges(rindex00);
        double range01 = _msg->scan().ranges(rindex01);
        double range10 = _msg->scan().ranges(rindex10);
        double range11 = _msg->scan().ranges(rindex11);

        range =
          ((vindex1 - vsample.index) *
          ((hindex1 - hsample.index) * range00 + (hsample.index - hindex0) * range10) +
          (vsample.index - vindex0) *
          ((hindex1 - hsample.index) * range01 + (hsample.index - hindex0) * range11));

        intensity = (_msg->scan().intensities(rindex00) +
          _msg->scan().intensities(rindex01) +
          _msg->scan().intensities(rindex10) +
          _msg->scan().intensities(rindex11)) / 4;
      }
      // Ignore points that lay outside range bands or optionally, beneath a
      // minimum intensity level.
      if ((MIN_RANGE >= range) || (range >= MAX_RANGE) || (intensity < MIN_INTENSITY)) {
        continue;
      }

      // Noise
      if (gaussian_noise_ != 0.0) {
        range += gaussianKernel(0, gaussian_noise_);
      }

      // Get angles of ray to get xyz for point
      double yAngle = hsample.angle.Radian();
      double pAngle = vsample.angle.Radian();

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < range) && (range < MAX_RANGE)) {
        *((float *)(ptr + 0)) = range * cos(pAngle) * cos(yAngle);
        *((float *)(ptr + 4)) = range * cos(pAngle) * sin(yAngle);
        *((float *)(ptr + 8)) = range * sin(pAngle);
        *((float *)(ptr + 16)) = intensity;
        *((uint16_t *)(ptr + 20)) = j;  // ring
        ptr += POINT_STEP;
      }
    }
    j += 1;
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step);  // Shrink to actual size
  // Publish output
  pub_->publish(msg);
}

}  // namespace gazebo
