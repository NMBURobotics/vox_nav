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

#ifndef BOTANBOT_GUI__CONTROL_PLUGIN_HPP_
#define BOTANBOT_GUI__CONTROL_PLUGIN_HPP_


// QT headers
#include <QtCore/QTimer>
#include <QtConcurrent/QtConcurrent>
// STL headers
#include <vector>
// ros headers
#include "rqt_gui_cpp/plugin.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "botanbot_gui/robot_controller.hpp"
#include "ui_control_plugin.h"

namespace botanbot_gui
{
/**
 * @brief A RQT instance, to control and visualize robot states conviniently
 *
 */
class ControlPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  ControlPlugin();

  ~ControlPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);

  virtual void shutdownPlugin();

  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;

  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

protected slots:
  /**
   * @brief stops current motion of robot, cancels any present goal
   *
   */
  virtual void onStopButtonClick();

  /**
   * @brief drives robot to predefined charging station
   *
   */
  virtual void onToChargeStationButtonClick();

  /**
   * @brief takes robot to pose , the pose values are recived from GUI spin boxes
   *
   */
  virtual void onToSpecificPoseButtonClick();

  /**
   * @brief sends robot along predefined waypoints, the points can be reconfigured in the yaml file of chiconbot_waypoint_follower_client
   *
   */
  virtual void onAlongPredefinedWaypointsButtonClick();

  /**
 * @brief Cancel all all active goals
 *
 */
  virtual void onCancelAllGoals();

  /**
   * @brief launches Gazebo simulation with chiconybot, only gazebo simulation is started nothing else
   *
   */
  virtual void onGazeboStandaloneButtonClick();

  /**
   * @brief Start full gazebo simulation and navigation stack
   *
   */
  virtual void onGazeboNavigationFullButtonClick();

  /**
   * @brief starts only RVIZ standalone
   *
   */
  virtual void onRvizButtonClick();

  /**
   * @brief Terminates all ACTIVE ROS2 nodes
   *
   */
  virtual void onTerminateAllButtonClick();

  /**
   * @brief manually jogs robot with GUI sliders
   *
   */
  virtual void teleoperation();

  /**
   * @brief displays current robot states(x,y,theta) in GUI , with using of LCDNumber of QT
   *
   */
  virtual void updateRobotStates();

protected:
  // GUI designed in QT Designer, automatically created from .ui file
  Ui::ControlPluginWidget ui_;
  // QT widget instance
  QPointer<QWidget> widget_;
  // RCLCPP node
  rclcpp::Node::SharedPtr node_;
  // ROS2 oublisher to publish velocity commands , for maual robot jogging
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // QT timer to update robot states and publish velocity commands(if user is doing manual jog)
  QPointer<QTimer> timer_;

  // A timer used to check on the completion status of the action
  QPointer<QTimer> basic_timer_;
  // Robot controller to sent robot to goals and chek it navigation status
  RobotController * robot_controller_;

private:
};

}  // namespace botanbot_gui

#endif  // BOTANBOT_GUI__CONTROL_PLUGIN_HPP_
