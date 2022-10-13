// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2021 Norwegian University of Life Sciences, Fetullah Atas
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

#include "builtin_interfaces/msg/duration.hpp"
#include "vox_nav_control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace vox_nav_control
{
  ControllerServer::ControllerServer()
  : Node("vox_nav_controller_server_rclcpp_node"),
    pc_loader_("vox_nav_control", "vox_nav_control::ControllerCore"),
    controller_id_("MPCControllerCasadiROS"),
    controller_type_("mpc_controller::MPCControllerCasadiROS")
  {
    RCLCPP_INFO(get_logger(), "Creating");

    // Declare this node's parameters
    declare_parameter("controller_frequency", 5.0);
    get_parameter("controller_frequency", controller_frequency_);

    declare_parameter("goal_tolerance_distance", 0.5);
    get_parameter("goal_tolerance_distance", goal_tolerance_distance_);

    declare_parameter("goal_tolerance_orientation", 0.1);
    get_parameter("goal_tolerance_orientation", goal_tolerance_orientation_);

    declare_parameter("transform_timeout", 0.1);
    get_parameter("transform_timeout", transform_timeout_);

    declare_parameter("controller_plugin", controller_id_);
    get_parameter("controller_plugin", controller_id_);

    declare_parameter(controller_id_ + ".plugin", controller_type_);
    get_parameter(controller_id_ + ".plugin", controller_type_);

    try {
      controller_ =
        pc_loader_.createSharedInstance(controller_type_);
      controller_->initialize(this, controller_id_);
      RCLCPP_INFO(
        get_logger(), "Created and initialized controller plugin %s of type %s",
        controller_id_.c_str(), controller_type_.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create controller. Exception: %s",
        ex.what());
    }

    RCLCPP_INFO(
      get_logger(),
      "Selected controller id: %s", controller_id_.c_str());
    if (controller_frequency_ > 0) {
      controller_duration_ = 1.0 / controller_frequency_;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "The expected controller frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages", controller_frequency_);
      controller_duration_ = 0.0;
    }

    this->action_server_ = rclcpp_action::create_server<FollowPath>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_path",
      std::bind(&ControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ControllerServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ControllerServer::handle_accepted, this, std::placeholders::_1));

    // setup TF buffer and listerner to read transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("vox_nav/cmd_vel", 10);

    mqtt_thread_ =
      std::make_shared<std::thread>(
      std::thread(
        &ControllerServer::executeMQTTThread,
        this));

    live_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points",
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControllerServer::liveCloudCallback, this, std::placeholders::_1));

    readjusted_reference_traj_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/vox_nav/controller/readjusted_plan", 1);

    dbg_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/vox_nav/controller/readjusted_plan_cloud", rclcpp::SensorDataQoS()
      );

    RCLCPP_INFO(get_logger(), "Constructed control server ... ");

  }

  void ControllerServer::executeMQTTThread()
  {

    struct mosquitto * mosq;
    int rc;

    /* Required before calling other mosquitto functions */
    mosquitto_lib_init();

    /* Create a new client instance.
     * id = NULL -> ask the broker to generate a client id for us
     * clean session = true -> the broker should remove old sessions when we connect
     * obj = NULL -> we aren't passing any of our private data for callbacks
     */
    mosq = mosquitto_new(NULL, true, NULL);
    if (mosq == NULL) {
      RCLCPP_INFO(get_logger(), "Error: Out of memory.\n");
      return;
    }

    /* Configure callbacks. This should be done before connecting ideally. */
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_subscribe_callback_set(mosq, on_subscribe);
    mosquitto_message_callback_set(mosq, on_message);

    /* Connect to test.mosquitto.org on port 1883, with a keepalive of 60 seconds.
     * This call makes the socket connection only, it does not complete the MQTT
     * CONNECT/CONNACK flow, you should use mosquitto_loop_start() or
     * mosquitto_loop_forever() for processing net traffic. */
    rc = mosquitto_connect(mosq, "192.168.0.171", 1883, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
      mosquitto_destroy(mosq);
      RCLCPP_INFO(get_logger(), "Error: %s\n", mosquitto_strerror(rc));
      return;
    }

    /* Run the network loop in a blocking call. The only thing we do in this
     * example is to print incoming messages, so a blocking call here is fine.
     *
     * This call will continue forever, carrying automatic reconnections if
     * necessary, until the user calls mosquitto_disconnect().
     */
    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_lib_cleanup();
  }

  ControllerServer::~ControllerServer()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
    controller_.reset();
    action_server_.reset();
  }

  rclcpp_action::GoalResponse ControllerServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received goal request in order to follow a path");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ControllerServer::handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ControllerServer::handle_accepted(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerServer::followPath, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void
  ControllerServer::followPath(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    auto start_time = steady_clock_.now();
    rclcpp::Rate loop_rate(controller_frequency_);


    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto result = std::make_shared<FollowPath::Result>();

    if (!goal->path.poses.size() > 0) {
      RCLCPP_WARN(
        get_logger(), "Recieved an empty path, this was probaly unintended, ignoring the request.");
      if (rclcpp::ok()) {
        auto cycle_duration = steady_clock_.now() - start_time;
        result->total_time = cycle_duration;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Follow Path Succeeded!");
        return;
      }
    }
    geometry_msgs::msg::PoseStamped initial_robot_pose;
    vox_nav_utilities::getCurrentPose(
      initial_robot_pose, *tf_buffer_, "map", "base_link", transform_timeout_);
    nav_msgs::msg::Path path;
    path.header = goal->path.header;
    initial_robot_pose.pose.position.z = goal->path.poses.front().pose.position.z;
    path.poses.push_back(initial_robot_pose);

    for (auto && i : goal->path.poses) {
      path.poses.push_back(i);
    }

    geometry_msgs::msg::Twist computed_velocity_commands;
    // set Plan
    controller_->setPlan(path);

    rclcpp::WallRate rate(controller_frequency_);

    volatile bool is_goal_distance_tolerance_satisfied = false;
    volatile bool is_goal_orientation_tolerance_satisfied = false;

    double average_time_taken_by_controller_loop = 0.0;
    double control_cycles = 0;

    while (rclcpp::ok() && !is_goal_distance_tolerance_satisfied) {


      auto & clock = *this->get_clock();


      auto loop_start_time = steady_clock_.now();
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
        return;
      }

      geometry_msgs::msg::PoseStamped curr_robot_pose, curr_robot_steering_angle;

      vox_nav_utilities::getCurrentPose(
        curr_robot_pose, *tf_buffer_, "map", "base_link", transform_timeout_);

      // NOTE THAT THIS IS ONLY FOR POLYTUNNEL PLANNER
      /*vox_nav_control::common::readjustGlobalPlanLocally(
        curr_robot_pose, latest_live_cloud_,
        readjusted_reference_traj_publisher_,
        dbg_cloud_pub_,
        this,
        path,
        0.3, 0.2);

      controller_->setPlan(path);*/

      int nearest_traj_pose_index = vox_nav_control::common::nearestStateIndex(
        path,
        curr_robot_pose);
      curr_robot_pose.pose.position.z = path.poses[nearest_traj_pose_index].pose.position.z;

      // MQTT Subscriber, used to determine PAUSE/RESUME behaviour
      if (curr_comand_ == 2) {
        rate.sleep();
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          clock,
          2000,     // ms
          "ROBOT WONT MOVE, PAUSED, PRESS RESUME !");
        continue;
      }

      // MQTT Subscriber, used to determine PAUSE/RESUME behaviour
      if (curr_comand_ == 0) {
        rate.sleep();
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          clock,
          2000,     // ms
          "Recieved STOP command !");
        continue;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        clock, 1000, "Remaining Distance to goal %.4f ...",
        vox_nav_utilities::getEuclidianDistBetweenPoses(
          curr_robot_pose,
          path.poses.back()));

      // check if we have arrived to goal, note the goal is last pose of path
      if (vox_nav_utilities::getEuclidianDistBetweenPoses(
          curr_robot_pose,
          path.poses.back()) < goal_tolerance_distance_)
      {
        // goal has been reached
        is_goal_distance_tolerance_satisfied = true;
        // reset the velocity
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());

        RCLCPP_INFO(
          this->get_logger(), "Goal has been reached, Now adjusting correct orientation ...");

        while (rclcpp::ok() && !is_goal_orientation_tolerance_satisfied) {

          if (goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
            cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
            is_goal_orientation_tolerance_satisfied = true;
            break;
          }

          computed_velocity_commands =
            controller_->computeHeadingCorrectionCommands(curr_robot_pose);
          cmd_vel_publisher_->publish(computed_velocity_commands);
          goal_handle->publish_feedback(feedback);

          vox_nav_utilities::getCurrentPose(
            curr_robot_pose, *tf_buffer_, "map", "base_link", transform_timeout_);

          double nan, curr_robot_psi, goal_psi;
          vox_nav_utilities::getRPYfromMsgQuaternion(
            curr_robot_pose.pose.orientation, nan, nan, curr_robot_psi);

          vox_nav_utilities::getRPYfromMsgQuaternion(
            path.poses.back().pose.orientation, nan, nan, goal_psi);

          if (std::abs(curr_robot_psi - goal_psi) < goal_tolerance_orientation_) {
            // goal has been reached
            is_goal_orientation_tolerance_satisfied = true;
            // reset the velocity
            cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
            RCLCPP_INFO(
              this->get_logger(),
              "Adjusted correct orientation finishing the action with success.");
            break;
          }
        }
        break;
      }

      auto control_cycle_start_time = steady_clock_.now();
      computed_velocity_commands = controller_->computeVelocityCommands(curr_robot_pose);
      auto control_cycle_duration = steady_clock_.now() - control_cycle_start_time;

      average_time_taken_by_controller_loop += control_cycle_duration.seconds();
      control_cycles += 1.0;

      // Print the Loop Rate once in a while
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        clock,
        2000, // ms
        "Average Current Control loop rate is %.4f Hz",
        1.0 / (average_time_taken_by_controller_loop / control_cycles));

      // Update sequence
      auto elapsed_time = steady_clock_.now() - start_time;
      feedback->elapsed_time = elapsed_time;
      feedback->speed = computed_velocity_commands.linear.x;
      goal_handle->publish_feedback(feedback);
      cmd_vel_publisher_->publish(computed_velocity_commands);
      // If loop is slower than expected , notify
      auto cycle_duration = steady_clock_.now() - loop_start_time;
      if (controller_duration_ && cycle_duration.seconds() > controller_duration_) {
        RCLCPP_WARN(
          get_logger(),
          "Contol loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / controller_duration_, 1 / cycle_duration.seconds());
      }

      rate.sleep();
    }
    auto cycle_duration = steady_clock_.now() - start_time;
    // Check if goal is done
    if (rclcpp::ok()) {
      cycle_duration = steady_clock_.now() - start_time;
      result->total_time = cycle_duration;
      goal_handle->succeed(result);
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO(this->get_logger(), "Follow Path Succeeded!");
    }
  }

  void ControllerServer::liveCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    /*std::lock_guard<std::mutex> guard(latest_live_cloud_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *pcl_curr);
    pcl_ros::transformPointCloud("map", *pcl_curr, *pcl_curr, *tf_buffer_);
    latest_live_cloud_ = pcl_curr;*/
  }

}  // namespace vox_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_control::ControllerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
