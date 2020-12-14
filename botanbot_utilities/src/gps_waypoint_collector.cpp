#include <iostream>
#include "botanbot_utilities/gps_waypoint_collector.hpp"

namespace botanbot_utilities
{

GPSWaypointCollector::GPSWaypointCollector()
: Node("gps_waypoint_collector_rclcpp_node"), gps_msg_index_(0), is_first_msg_recieved_(false)
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&GPSWaypointCollector::timerCallback, this));

  navsat_fix_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10,
    std::bind(&GPSWaypointCollector::navSatFixCallback, this, std::placeholders::_1));
}

GPSWaypointCollector::~GPSWaypointCollector()
{
}

void GPSWaypointCollector::timerCallback()
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Entering to timer callback, this is periodicly called");
  if (is_first_msg_recieved_) {
    std::lock_guard<std::mutex> guard(global_mutex_);
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "gps_waypoint" + std::to_string(
        gps_msg_index_) + ": %.8f, %.8f, %.8f", reusable_navsat_msg_.latitude,
      reusable_navsat_msg_.longitude, reusable_navsat_msg_.altitude);
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "PRESS ENTER TO CONTINUE");
    getchar();
  }


}

void GPSWaypointCollector::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(global_mutex_);
  reusable_navsat_msg_ = *msg;
  is_first_msg_recieved_ = true;
}

} // namespace botanbot_utilities

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::GPSWaypointCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
