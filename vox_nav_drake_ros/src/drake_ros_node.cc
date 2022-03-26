#include <memory>
#include <utility>
#include <vector>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/basic_types.hpp>

#include "drake_ros_core/drake_ros.h"
#include "drake_ros_core/ros_interface_system.h"
#include "drake_ros_core/ros_publisher_system.h"
#include "drake_ros_core/ros_subscriber_system.h"

using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosPublisherSystem;
using drake_ros_core::RosSubscriberSystem;

int main(int argc, char const * argv[])
{

  drake_ros_core::init(0, nullptr);

  drake::systems::DiagramBuilder<double> builder;

  constexpr double kPublishPeriod = 1.0;
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(10)}.reliable();

  auto system_ros = builder.AddSystem<RosInterfaceSystem>(
    std::make_unique<DrakeRos>("pub_to_sub"));

  auto system_sub_in =
    builder.AddSystem(
    RosSubscriberSystem::Make<test_msgs::msg::BasicTypes>(
      "in", qos, system_ros->get_ros_interface()));

  auto system_pub_out =
    builder.AddSystem(
    RosPublisherSystem::Make<test_msgs::msg::BasicTypes>(
      "out", qos, system_ros->get_ros_interface(),
      {drake::systems::TriggerType::kPeriodic}, kPublishPeriod));

  builder.Connect(
    system_sub_in->get_output_port(0),
    system_pub_out->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
    *diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto & simulator_context = simulator->get_mutable_context();

  // Don't need to rclcpp::init because DrakeRos uses global rclcpp::Context by
  // default
  auto direct_ros_node = rclcpp::Node::make_shared("sub_to_pub");

  // Create publisher talking to subscriber system.
  auto direct_pub_in =
    direct_ros_node->create_publisher<test_msgs::msg::BasicTypes>("in", qos);

  // Create subscription listening to publisher system
  std::vector<std::unique_ptr<test_msgs::msg::BasicTypes>>
  rx_msgs_direct_sub_out;
  auto rx_callback_direct_sub_out =
    [&](std::unique_ptr<test_msgs::msg::BasicTypes> message) {
      rx_msgs_direct_sub_out.push_back(std::move(message));
    };
  auto direct_sub_out =
    direct_ros_node->create_subscription<test_msgs::msg::BasicTypes>(
    "out", qos, rx_callback_direct_sub_out);

  constexpr size_t kPubSubRounds = 5;
  for (size_t i = 1; i <= kPubSubRounds; ++i) {
    const size_t rx_msgs_count_before_pubsub = rx_msgs_direct_sub_out.size();
    // Publish a message to the drake ros subscriber system.
    auto message = std::make_unique<test_msgs::msg::BasicTypes>();
    message->uint64_value = i;
    direct_pub_in->publish(std::move(message));
    // Step forward to allow the message to be dispatched to the drake ros
    // subscriber system. The drake ros publisher system should not publish
    // just yet.
    rclcpp::spin_some(direct_ros_node);
    simulator->AdvanceTo(simulator_context.get_time() + kPublishPeriod / 2.);
    // Step forward until it is about time the drake ros publisher publishes.
    // Allow the message to be dispatched to the direct subscription.
    simulator->AdvanceTo(simulator_context.get_time() + kPublishPeriod / 2.);
    rclcpp::spin_some(direct_ros_node);
    const size_t rx_msgs_count_after_pubsub = rx_msgs_count_before_pubsub + 1;

    RCLCPP_INFO(direct_ros_node->get_logger(), " %.5f ", simulator_context.get_time() );
  }
  return 0;

}
