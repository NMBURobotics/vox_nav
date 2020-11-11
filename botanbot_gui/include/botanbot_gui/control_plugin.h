/*
 * Copyright (c) 2020, Fetullah Atas
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef botanbot_gui__ControlPlugin_H
#define botanbot_gui__ControlPlugin_H

// ros headers
#include <rqt_gui_cpp/plugin.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// QT headers
#include <QtCore/QTimer>
#include <QtConcurrent/QtConcurrent>
#include <ui_control_plugin.h>

// this package header
#include <botanbot_gui/RobotController.h>

// STL headers
#include <vector>

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
   * @brief starts only navigation components
   *
   */
  virtual void onNavigationStandAloneButtonClick();

  /**
   * @brief starts all components of navigation(slam , localization , navigation_bringup)
   *
   */
  virtual void onNavigationFullButtonClick();

  /**
   * @brief starts only SLAM with map server
   *
   */
  virtual void onSlamMapServerButtonClick();

  /**
   * @brief start only slam with google cartographer
   *
   */
  virtual void onSlamCartoButtonClick();

  /**
   * @brief starts only AMCL localization
   *
   */
  virtual void onLocalizationButtonClick();

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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  // QT timer to update robot states and publish velocity commands(if user is doing manual jog)
  QPointer<QTimer> timer_;

  // A timer used to check on the completion status of the action
  QPointer<QTimer> basic_timer_;
  // Robot controller to sent robot to goals and chek it navigation status
  RobotController * robot_controller_;

private:
};

}  // namespace botanbot_gui

#endif  // botanbot_gui__ControlPlugin_H
