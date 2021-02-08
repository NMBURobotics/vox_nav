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

#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "botanbot_gui/control_plugin.hpp"

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

  node_ = rclcpp::Node::make_shared("control_plugin_rclcpp_node");

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

  ui_.startRviz->setIcon(QIcon::fromTheme("call-start"));
  connect(
    ui_.startRviz, SIGNAL(pressed()), this,
    SLOT(onRvizButtonClick()));

  ui_.terminate->setIcon(QIcon::fromTheme("call-start"));
  connect(
    ui_.terminate, SIGNAL(pressed()), this,
    SLOT(onTerminateAllButtonClick()));

  /**
   * TELEOPRATION
   *
   */

  ui_.stop->setIcon(QIcon::fromTheme("process-stop"));
  connect(ui_.stop, SIGNAL(pressed()), this, SLOT(onStopButtonClick()));

  ui_.sendToChargeStation->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.sendToChargeStation, SIGNAL(pressed()), this,
    SLOT(onToChargeStationButtonClick()));

  ui_.sendToPose->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.sendToPose, SIGNAL(pressed()), this,
    SLOT(onToSpecificPoseButtonClick()));

  ui_.sendAlongPredefinedWaypoints->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.sendAlongPredefinedWaypoints, SIGNAL(pressed()), this,
    SLOT(onAlongPredefinedWaypointsButtonClick()));

  ui_.cancelAllGoals->setIcon(QIcon::fromTheme("system-run"));
  connect(
    ui_.cancelAllGoals, SIGNAL(pressed()), this,
    SLOT(onCancelAllGoals()));

  RCLCPP_INFO(node_->get_logger(), "Creating Robot Controller");

  /**
   * FUNCTIONS THAT REQUIRES TIMERS
   *
   */
  timer_->start(100);
  timer_->connect(timer_, SIGNAL(timeout()), this, SLOT(teleoperation()));
  //timer_->connect(timer_, SIGNAL(timeout()), this, SLOT(updateRobotStates()));

  cmd_vel_publisher_ =
    node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  std::cout << "Initialized plugin . . " << std::endl;

  node_->set_parameter(rclcpp::Parameter("use_sim_time", true));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
  cmd_vel_publisher_->publish(twist);
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
  /*std::string this_package_path = ament_index_cpp::get_package_share_directory(
    "botanbot_gps_waypoint_follower");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/wpf_run.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/wpf_run.sh";
  system(bringup_command.c_str());*/
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
    cmd_vel_publisher_->publish(twist);
  }
}

void ControlPlugin::updateRobotStates()
{
  geometry_msgs::msg::PoseStamped curr_pose;
  botanbot_utilities::getCurrentPose(curr_pose, *tf_buffer_, "map", "base_link", 0.1);
  ui_.x->display(curr_pose.pose.position.x);
  ui_.y->display(curr_pose.pose.position.y);
  tf2::Quaternion quat(
    curr_pose.pose.orientation.x,
    curr_pose.pose.orientation.y,
    curr_pose.pose.orientation.z,
    curr_pose.pose.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ui_.theta->display(yaw * 180.0 / M_PI);
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
  // get string text of selected robot
  std::string selected_gazebo_world_name = ui_.gazebo_city_combobox->currentText().toStdString();

  std::string botanbot_gui_package_path =
    ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string bash_files_dir = botanbot_gui_package_path +
    "/../../scripts/start_selected_gazebo_standalone.sh";
  std::string make_bash_files_executable_command = "chmod +x " +
    bash_files_dir;
  system(make_bash_files_executable_command.c_str());

  std::string bringup_command =
    botanbot_gui_package_path + "/../../scripts/start_selected_gazebo_standalone.sh " +
    selected_gazebo_world_name;
  system(bringup_command.c_str());
}

void ControlPlugin::onGazeboNavigationFullButtonClick()
{
  // get string text of selected robot
  std::string selected_gazebo_world_name = ui_.gazebo_city_combobox->currentText().toStdString();

  std::string botanbot_gui_package_path =
    ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string bash_files_dir = botanbot_gui_package_path +
    "/../../scripts/start_selected_gazebo_navigation_full.sh";
  std::string make_bash_files_executable_command = "chmod +x " +
    bash_files_dir;
  system(make_bash_files_executable_command.c_str());

  std::string bringup_command =
    botanbot_gui_package_path + "/../../scripts/start_selected_gazebo_navigation_full.sh " +
    selected_gazebo_world_name;
  system(bringup_command.c_str());
}

void ControlPlugin::onRvizButtonClick()
{
  std::string this_package_path =
    ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir =
    this_package_path + "/../../scripts/rviz.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command = this_package_path + "/../../scripts/rviz.sh";
  system(bringup_command.c_str());
}

void ControlPlugin::onTerminateAllButtonClick()
{
  std::string this_package_path =
    ament_index_cpp::get_package_share_directory("botanbot_gui");

  std::string this_package_bash_files_dir = this_package_path +
    "/../../scripts/close_all_terminals.sh";
  std::string this_package_make_bash_files_executable_command = "chmod +x " +
    this_package_bash_files_dir;
  system(this_package_make_bash_files_executable_command.c_str());

  std::string bringup_command =
    this_package_path + "/../../scripts/close_all_terminals.sh";
  system(bringup_command.c_str());
}

}  // namespace botanbot_gui
PLUGINLIB_EXPORT_CLASS(botanbot_gui::ControlPlugin, rqt_gui_cpp::Plugin)
