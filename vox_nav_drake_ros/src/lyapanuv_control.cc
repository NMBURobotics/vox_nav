#include <memory>
#include <utility>
#include <vector>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/basic_types.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "drake_ros_core/drake_ros.h"
#include "drake_ros_core/ros_interface_system.h"
#include "drake_ros_core/ros_publisher_system.h"
#include "drake_ros_core/ros_subscriber_system.h"

using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosPublisherSystem;
using drake_ros_core::RosSubscriberSystem;

class LyapanuvController : public drake::systems::LeafSystem<double>
{
public:
  LyapanuvController()
  {
    DeclareAbstractInputPort("states", *drake::AbstractValue::Make(geometry_msgs::msg::Pose()));
    DeclareAbstractOutputPort("controls", &LyapanuvController::calc_output_value);
  }
  virtual ~LyapanuvController() = default;

  void calc_output_value(
    const drake::systems::Context<double> & context,
    geometry_msgs::msg::Twist * control) const
  {
    auto target_x = 15.0;
    auto target_y = -15.0;
    const auto curr_pose = GetInputPort("states").Eval<geometry_msgs::msg::Pose>(context);
    // Extract yaw from quat
    tf2::Quaternion q;
    tf2::fromMsg(curr_pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    auto z1 = curr_pose.position.x;
    auto z2 = curr_pose.position.y;
    auto z3 = yaw;
    // state in polar coordinates
    auto x1 = std::sqrt(std::pow((z1 - target_x), 2) + std::pow((z2 - target_y), 2));
    auto x2 = std::atan2((z2 - target_y), (z1 - target_x));
    auto x3 = x2 - z3;

    auto k1 = 1.0;
    auto k2 = 0.1;

    auto u1 = k1 * (-x1 * std::cos(x3));
    auto u2 = k2 * (x3 + (x3 + x2) * std::cos(x3) * std::sin(x3) / x3);
    if (x1 < 0.1) {
      u1 = 0.0;
      u2 = 0.0;
    }
    control->linear.x = std::clamp<double>(u1, -0.4, 0.4);
    control->angular.z = std::clamp<double>(u2, -0.3, 0.3);
  }

private:
};


class Robot : public drake::systems::LeafSystem<double>, public rclcpp::Node
{
public:
  Robot()
  : Node("robot_rclcpp_node")
  {
    DeclareAbstractInputPort("controls", *drake::AbstractValue::Make(geometry_msgs::msg::Twist()));
    DeclareAbstractOutputPort(
      "states", &Robot::calc_output_value);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("vox_nav/cmd_vel", 10);
  }

  std::string get_name_node() const {return rclcpp::Node::get_name();}
  std::string get_name() const {return drake::systems::LeafSystem<double>::get_name();}

  virtual ~Robot() = default;

  bool getCurrentPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    tf2_ros::Buffer & tf_buffer, const std::string global_frame,
    const std::string robot_frame, const double transform_timeout) const
  {
    static rclcpp::Logger logger = rclcpp::get_logger("getCurrentPose");
    geometry_msgs::msg::PoseStamped robot_pose;

    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_frame;
    robot_pose.header.stamp = rclcpp::Time();

    try {
      global_pose = tf_buffer.transform(
        robot_pose, global_frame,
        tf2::durationFromSec(transform_timeout));
      return true;
    } catch (tf2::LookupException & ex) {
      RCLCPP_ERROR(
        logger,
        "No Transform available Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::ConnectivityException & ex) {
      RCLCPP_ERROR(
        logger,
        "Connectivity Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(
        logger,
        "Extrapolation Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::TimeoutException & ex) {
      RCLCPP_ERROR(
        logger,
        "Transform timeout with tolerance: %.4f", transform_timeout);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger, "Failed to transform from %s to %s",
        global_frame.c_str(), robot_frame.c_str());
    }
    return false;
  }

  void calc_output_value(
    const drake::systems::Context<double> & context,
    geometry_msgs::msg::Pose * states) const
  {
    geometry_msgs::msg::PoseStamped curr_robot_pose;
    double transform_timeout = 0.1;
    auto result = getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", transform_timeout);
    const auto curr_controls = GetInputPort("controls").Eval<geometry_msgs::msg::Twist>(context);
    cmd_vel_publisher_->publish(curr_controls);
    states->position = curr_robot_pose.pose.position;
    states->orientation = curr_robot_pose.pose.orientation;

    RCLCPP_INFO(
      get_logger(), "Publishing ,x: %.3f , y: %.3f  ", states->position.x, states->position.y);
  }

private:
  // setup TF buffer and listerner to read transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

// Delay's input port by one timestep to avoid algebraic loop error
// Inspired by Simulink's Memory block
template<typename T>
class Memory : public drake::systems::LeafSystem<double>
{
public:
  explicit Memory(const T & initial_value)
  {
    DeclareAbstractInputPort("value", *drake::AbstractValue::Make(T()));

    // State for value
    DeclareAbstractState(drake::Value<T>(initial_value));

    // Output depends only on the previous state
    DeclareAbstractOutputPort("value", &Memory::calc_output_value, {all_state_ticket()});

    DeclarePerStepEvent(
      drake::systems::UnrestrictedUpdateEvent<double>(
        [this](
          const drake::systems::Context<double> & context,
          const drake::systems::UnrestrictedUpdateEvent<double> &,
          drake::systems::State<double> * state) {
          // Copy input value to state
          drake::systems::AbstractValues & abstract_state = state->get_mutable_abstract_state();
          abstract_state.get_mutable_value(0).SetFrom(
            get_input_port().Eval<drake::AbstractValue>(context));
        }));
  }

  virtual ~Memory() = default;

private:
  void
  calc_output_value(const drake::systems::Context<double> & context, T * output) const
  {
    *output = context.get_abstract_state().get_value(0).get_value<T>();
  }
};

int main(int argc, char const * argv[])
{
  drake_ros_core::init(0, nullptr);
  drake::systems::DiagramBuilder<double> builder;

  auto controller = builder.AddSystem<LyapanuvController>();
  controller->set_name("controller");
  auto robot = builder.AddSystem<Robot>();
  robot->set_name("robot");

  auto sys_memory = builder.AddSystem<Memory<geometry_msgs::msg::Pose>>(geometry_msgs::msg::Pose());

  builder.Connect(robot->GetOutputPort("states"), sys_memory->get_input_port(0));
  builder.Connect(sys_memory->get_output_port(0), controller->GetInputPort("states"));
  builder.Connect(controller->GetOutputPort("controls"), robot->GetInputPort("controls"));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
    *diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto & simulator_context = simulator->get_mutable_context();

  while (true) {
    rclcpp::spin_some(robot->get_node_base_interface());
    simulator->AdvanceTo(simulator_context.get_time() + 0.025);
  }

  return 0;
}
