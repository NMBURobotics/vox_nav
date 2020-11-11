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

#include <botanbot_gui/control_plugin.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;
namespace botanbot_gui
{
ControlPlugin::ControlPlugin()
: rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("ControlPlugin");
}

ControlPlugin::~ControlPlugin()
{
  shutdownPlugin();
}

void ControlPlugin::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();

  timer_ = new QTimer(widget_);

  basic_timer_ = new QTimer(widget_);

  robot_controller_ = new RobotController();

  ui_.setupUi(widget_);

  context.addWidget(widget_);

  /**
   * LAUNCHERS
   *
   */

  ui_.startGazeboStandalone->setIcon(QIcon::fromTheme("call-start"));
  connect(
    ui_.startGazeboStandalone, SIGNAL(pressed()), this,
    SLOT(onGazeboStandaloneButtonClick()));

  ui_.startGazeboNavigationFull->setIcon(QIcon::fromTheme("call-start"));
  connect(
    ui_.startGazeboNavigationFull, SIGNAL(pressed()), this,
    SLOT(onGazeboNavigationFullButtonClick()));

  ui_.startNavigationStandalone->setIcon(QIcon::fromTheme("call-start"));
  connect(
    ui_.startNavigationStandalone, SIGNAL(pressed()), this,
    SLOT(onNavigationStandAloneButtonClick()));

  ui_.startNavigationFull->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.startNavigationFull, SIGNAL(pressed()), this, SLOT(onNavigationFullButtonClick()));

  ui_.startSlamMapServer->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.startSlamMapServer, SIGNAL(pressed()), this, SLOT(onSlamMapServerButtonClick()));

  ui_.startSlamCarto->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.startSlamCarto, SIGNAL(pressed()), this, SLOT(onSlamCartoButtonClick()));

  ui_.startLocalization->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.startLocalization, SIGNAL(pressed()), this, SLOT(onLocalizationButtonClick()));

  ui_.startRviz->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.startRviz, SIGNAL(pressed()), this, SLOT(onRvizButtonClick()));

  ui_.terminate->setIcon(QIcon::fromTheme("call-start"));
  connect(ui_.terminate, SIGNAL(pressed()), this, SLOT(onTerminateAllButtonClick()));

  /**
   * TELEOPRATION
   *
   */

  ui_.stop->setIcon(QIcon::fromTheme("process-stop"));
  connect(ui_.stop, SIGNAL(pressed()), this, SLOT(onStopButtonClick()));

  ui_.sendToChargeStation->setIcon(QIcon::fromTheme("system-run"));
  connect(ui_.sendToChargeStation, SIGNAL(pressed()), this, SLOT(onToChargeStationButtonClick()));

  ui_.sendToPose->setIcon(QIcon::fromTheme("system-run"));
  connect(ui_.sendToPose, SIGNAL(pressed()), this, SLOT(onToSpecificPoseButtonClick()));

  ui_.sendAlongPredefinedWaypoints->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.sendAlongPredefinedWaypoints, SIGNAL(pressed()), this,
    SLOT(onAlongPredefinedWaypointsButtonClick()));

  ui_.cancelAllGoals->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.cancelAllGoals, SIGNAL(pressed()), this,
    SLOT(onCancelAllGoals()));

  node_ = rclcpp::Node::make_shared("ControlPlugin");

  RCLCPP_INFO(node_->get_logger(), "Creating Robot Controller");

  /**
   * FUNCTIONS THAT REQUIRES TIMERS
   *
   */
  timer_->start(100);
  timer_->connect(timer_, SIGNAL(timeout()), this, SLOT(teleoperation()));
  timer_->connect(timer_, SIGNAL(timeout()), this, SLOT(updateRobotStates()));

  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  std::cout << "Initialized plugin . . " << std::endl;

  node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
}

void ControlPlugin::shutdownPlugin()
{
  delete widget_;
  delete timer_;
  delete basic_timer_;
  delete robot_controller_;
}

void ControlPlugin::saveSettings(
  qt_gui_cpp::Settings & plugin_settings,
  qt_gui_cpp::Settings & instance_settings) const
{
}

void ControlPlugin::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings,
  const qt_gui_cpp::Settings & instance_settings)
{
}

void ControlPlugin::onStopButtonClick()
{
  std::cout << "stopped robot" << std::endl;

  ui_.lrSlider->setValue(0);
  ui_.drSlider->setValue(0);

  float speed(0.0);
  float turn(0.0);

  geometry_msgs::msg::Twist twist;

  twist.linear.x = speed;
  twist.angular.z = turn;
  publisher_->publish(twist);

}

void ControlPlugin::onCancelAllGoals()
{
  std::cout << "Cancelling all goals" << std::endl;

  robot_controller_->cancelGoals();
}

void ControlPlugin::onToChargeStationButtonClick()
{
  geometry_msgs::msg::PoseStamped robot_charging_pose;
  robot_charging_pose.pose.position.x = -5.0;
  robot_charging_pose.pose.position.y = -5.0;
  robot_charging_pose.header.frame_id = "map";
  robot_charging_pose.header.stamp = rclcpp::Clock().now();

  robot_charging_pose.pose.orientation.x = 0;
  robot_charging_pose.pose.orientation.y = 0;
  robot_charging_pose.pose.orientation.z = 0;
  robot_charging_pose.pose.orientation.w = 1.0;

  QFuture<void> future = QtConcurrent::run(
    robot_controller_, &RobotController::toTargetPose,
    robot_charging_pose);

  basic_timer_->start(200);
}

void ControlPlugin::onAlongPredefinedWaypointsButtonClick()
{

  std::string this_package_path = ament_index_cpp::get_package_share_directory(
    "chiconybot_waypoint_follower_client");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/wpf_run.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/wpf_run.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::teleoperation()
{
  geometry_msgs::msg::Twist twist;

  // Linear velocity (m/s)
  float speed(0.0);
  // Angular velocity (rad/s)
  float turn(0.0);

  turn = ui_.lrSlider->value() / 50.0;
  speed = ui_.drSlider->value() / 50.0;

  // Update the Twist message
  twist.linear.x = speed;
  twist.angular.z = turn;

  if (ui_.lrSlider->value() != 0 || ui_.drSlider->value() != 0) {
    publisher_->publish(twist);
  }
}

void ControlPlugin::updateRobotStates()
{
  geometry_msgs::msg::TransformStamped curr_pose = robot_controller_->getRobotStates();

  ui_.x->display(curr_pose.transform.translation.x);
  ui_.y->display(curr_pose.transform.translation.y);

  tf2::Quaternion quat(curr_pose.transform.rotation.x, curr_pose.transform.rotation.y,
    curr_pose.transform.rotation.z,
    curr_pose.transform.rotation.w);

  double rotation_angle = quat.getAngle() - M_PI;

  ui_.theta->display(rotation_angle * 180.0 / M_PI);
}

void ControlPlugin::onToSpecificPoseButtonClick()
{
  geometry_msgs::msg::PoseStamped robot_goal_pose;
  robot_goal_pose.header.frame_id = "map";

  robot_goal_pose.pose.position.x = ui_.goal_x->value();
  robot_goal_pose.pose.position.y = ui_.goal_y->value();
  robot_goal_pose.header.stamp = rclcpp::Clock().now();

  tf2::Quaternion quat;
  quat.setRPY(0, 0, (ui_.goal_theta->value() * M_PI / 180.0));

  robot_goal_pose.pose.orientation = tf2::toMsg(quat);

  QFuture<void> future = QtConcurrent::run(
    robot_controller_, &RobotController::toTargetPose,
    robot_goal_pose);

  basic_timer_->start(200);
}

void ControlPlugin::onGazeboStandaloneButtonClick()
{
  std::cout << "Starting Gazebo Chiconybot Simulation" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/gazebo_standalone.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/gazebo_standalone.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onGazeboNavigationFullButtonClick()
{
  std::cout << "Starting whole Gazebo simulation and navigation SLAM" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/gazebo_navigation_full.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/gazebo_navigation_full.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onNavigationStandAloneButtonClick()
{
  std::cout << "Starting Navigation standalone  " << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/navigation_standalone.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/navigation_standalone.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onNavigationFullButtonClick()
{
  std::cout << "Starting Whole Navigation2 Stack(SLAM, Localization , Navigation)" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path + "/../../scripts/navigation_full.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/navigation_full.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onSlamMapServerButtonClick()
{
  std::cout << "Starting Map Server SLAM" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path + "/../../scripts/slam_map_server.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/slam_map_server.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onSlamCartoButtonClick()
{
  std::cout << "Starting Cartographer SLAM" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path + "/../../scripts/slam_carto.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/slam_carto.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onLocalizationButtonClick()
{
  std::cout << "Starting AMCL Localization" << std::endl;

  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/localization_amcl.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/localization_amcl.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onRvizButtonClick()
{
  std::cout << "Starting RViz" << std::endl;
  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path + "/../../scripts/rviz.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/rviz.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onTerminateAllButtonClick()
{
  std::cout << "Terminating all Processes across gnome terminals" << std::endl;
  std::cout << "Starting RViz" << std::endl;
  std::string this_package_path = ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/close_all_terminals.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/close_all_terminals.sh";
  system(bringup_command.c_str());
}

}  // namespace botanbot_gui
PLUGINLIB_EXPORT_CLASS(botanbot_gui::ControlPlugin, rqt_gui_cpp::Plugin)
