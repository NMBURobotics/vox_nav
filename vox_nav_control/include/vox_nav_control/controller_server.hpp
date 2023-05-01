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

#ifndef VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_
#define VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_
#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pcl_ros/transforms.hpp>

#include <mosquittopp.h>
#include <mosquitto.h>

#include "vox_nav_msgs/action/follow_path.hpp"
#include "vox_nav_control/controller_core.hpp"
#include "vox_nav_control/common.hpp"
#include "vox_nav_control/plan_refiner_core.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/planner_helpers.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <unordered_map>

volatile int curr_comand_{1};

rclcpp::Logger mqtt_logger{rclcpp::get_logger("mqtt_logger")};

/* Callback called when the client receives a CONNACK message from the broker. */
void on_connect(struct mosquitto * mosq, void * obj, int reason_code)
{
  int rc;
  /* Print out the connection result. mosquitto_connack_string() produces an
   * appropriate string for MQTT v3.x clients, the equivalent for MQTT v5.0
   * clients is mosquitto_reason_string().
   */
  RCLCPP_INFO(
    mqtt_logger, "on_connect: %s\n",
    mosquitto_connack_string(reason_code));
  if (reason_code != 0) {
    /* If the connection fails for any reason, we don't want to keep on
     * retrying in this example, so disconnect. Without this, the client
     * will attempt to reconnect. */
    mosquitto_disconnect(mosq);
  }

  /* Making subscriptions in the on_connect() callback means that if the
   * connection drops and is automatically resumed by the client, then the
   * subscriptions will be recreated when the client reconnects. */
  rc = mosquitto_subscribe(mosq, NULL, "current_command", 2);
  if (rc != MOSQ_ERR_SUCCESS) {
    RCLCPP_INFO(mqtt_logger, "Error subscribing: %s\n", mosquitto_strerror(rc));
    /* We might as well disconnect if we were unable to subscribe */
    mosquitto_disconnect(mosq);
  }
}


/* Callback called when the broker sends a SUBACK in response to a SUBSCRIBE. */
void on_subscribe(
  struct mosquitto * mosq, void * obj, int mid, int qos_count,
  const int * granted_qos)
{
  int i;
  bool have_subscription = false;

  /* In this example we only subscribe to a single topic at once, but a
   * SUBSCRIBE can contain many topics at once, so this is one way to check
   * them all. */
  for (i = 0; i < qos_count; i++) {
    RCLCPP_INFO(mqtt_logger, "on_subscribe: %d:granted qos = %d\n", i, granted_qos[i]);
    if (granted_qos[i] <= 2) {
      have_subscription = true;
    }
  }
  if (have_subscription == false) {
    /* The broker rejected all of our subscriptions, we know we only sent
     * the one SUBSCRIBE, so there is no point remaining connected. */
    RCLCPP_INFO(mqtt_logger, "Error: All subscriptions rejected.\n");
    mosquitto_disconnect(mosq);
  }
}

/* Callback called when the client receives a message. */
void on_message(struct mosquitto * mosq, void * obj, const struct mosquitto_message * msg)
{
  /* This blindly prints the payload, but the payload can be anything so take care. */
  RCLCPP_INFO(mqtt_logger, "%s %d %s\n", msg->topic, msg->qos, (char *)msg->payload);
  curr_comand_ = std::atoi((char *)msg->payload);
}

namespace vox_nav_control
{
  class ControllerServer : public rclcpp::Node
  {
  public:
    using FollowPath = vox_nav_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

    /**
     * @brief Construct a new Controller Server object
     *
     */
    ControllerServer();

    /**
     * @brief Destroy the Controller Server object
     *
     */
    ~ControllerServer();

    /**
     * @brief
     *
     * @param uuid
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowPath::Goal> goal);

    /**
     * @brief
     *
     * @param goal_handle
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    /**
     * @brief
     *
     * @param goal_handle
     */
    void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    /**
     * @brief The action server callback which computes control effort
     */
    void followPath(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    /**
    * @brief A dedicated thread to run MQTT.
    *
    */
    void executeMQTTThread();

  protected:
    // FollowPath action server
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;

    // Controller, pluginized server
    vox_nav_control::ControllerCore::Ptr controller_;
    pluginlib::ClassLoader<vox_nav_control::ControllerCore> pc_loader_;
    std::string controller_id_;
    std::string controller_type_;
    double controller_frequency_;
    double controller_duration_;
    double goal_tolerance_distance_;
    double goal_tolerance_orientation_;
    double transform_timeout_;

    // Path refiner pluginized server
    vox_nav_control::PlanRefinerCore::Ptr plan_refiner_;
    pluginlib::ClassLoader<vox_nav_control::PlanRefinerCore> pr_loader_;
    std::string plan_refiner_id_;
    std::string plan_refiner_type_;
    bool plan_refiner_enabled_;

    // Clock
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    // tf buffer to get access to transfroms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Publisher to publish velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // MQTT stuff
    std::shared_ptr<std::thread> mqtt_thread_;
    // Publishers for the path
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_msgs_path_pub_;

  };

}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_
