#include <memory>
#include <utility>
#include <vector>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <drake/common/symbolic_latex.h>
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class Robot : public drake::systems::LeafSystem<double>
{
public:
  Robot()
  {
    DeclareVectorInputPort("robot_input_control", 2);
    auto state_index = this->DeclareContinuousState(3);
    DeclareStateOutputPort("robot_output_states", state_index);
  }
  virtual ~Robot() = default;
// Compute the actual physics.
  void DoCalcTimeDerivatives(
    const drake::systems::Context<double> & context,
    drake::systems::ContinuousState<double> * derivatives) const
  {
    const drake::Vector2<double> u = this->EvalVectorInput(context, 0)->value();
    drake::Vector3<double> state = context.get_continuous_state_vector().CopyToVector();
    drake::Vector3<double> xdot;
    double L = 1.32;
    auto x_dot = u.x() * std::cos(state.z());
    auto y_dot = u.x() * std::sin(state.z());
    auto thata_dot = u.x() * std::tan(u.y() / L);
    xdot << x_dot, y_dot, thata_dot;
    derivatives->SetFromVector(xdot);
  }
};

class MeshcatSliders : public drake::systems::LeafSystem<double>
{
public:
  MeshcatSliders(
    const std::shared_ptr<drake::geometry::Meshcat> meshcat,
    const std::vector<std::string> slider_names)
  : meshcat_(meshcat),
    slider_names_(slider_names)
  {
    DeclareVectorOutputPort(
      "mesh_sliders_output_controls", 2, &MeshcatSliders::calc_output_value,
      {all_state_ticket()});
  }
  virtual ~MeshcatSliders() = default;

private:
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::vector<std::string> slider_names_;
  void
  calc_output_value(
    const drake::systems::Context<double> & context,
    drake::systems::BasicVector<double> * output) const
  {
    for (size_t i = 0; i < slider_names_.size(); i++) {
      output->SetAtIndex(i, meshcat_->GetSliderValue(slider_names_[i]));
    }
  }
};


int main(int argc, char const * argv[])
{

  drake::systems::DiagramBuilder<double> builder;
  auto meshcat_ = std::make_shared<drake::geometry::Meshcat>();
  std::vector<std::string> slider_names = {"v", "phi"};
  meshcat_->AddSlider("v", -1, 1, 0.05, 0.0);
  meshcat_->AddSlider("phi", -1, 1, 0.05, 0.0);

  auto robot = builder.AddSystem<Robot>();
  robot->set_name("robot");

  auto teleop = builder.AddSystem<MeshcatSliders>(meshcat_, slider_names);
  teleop->set_name("teleop");

  builder.Connect(
    teleop->GetOutputPort("mesh_sliders_output_controls"),
    robot->GetInputPort("robot_input_control"));

  std::string vox_nav_drake_ros_package_path =
    ament_index_cpp::get_package_share_directory("vox_nav_drake_ros");

  auto [plant, scene_graph] =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0001 /* time_step */);
  const std::string & pathname = vox_nav_drake_ros_package_path + "/urdf/botanbot.urdf";
  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile(pathname);

  plant.Finalize();
  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1000;

  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
    &builder, scene_graph, meshcat_, std::move(params));
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  drake::systems::Context<double> & plant_context =
    diagram->GetMutableSubsystemContext(plant, context.get());
  drake::systems::Simulator<double> simulator(*diagram);
  auto & simulator_context = simulator.get_mutable_context();
  simulator.Initialize();

  while (1) {
    const drake::systems::ContinuousState<double> & state =
      simulator_context.get_continuous_state();
    const drake::VectorX<double> & curr_pose = state.CopyToVector();
    drake::math::RollPitchYawd rpy(0, 0, curr_pose.z());
    drake::Vector3<double> xyz(curr_pose.x(), curr_pose.y(), 0);
    drake::math::RigidTransformd X_WB(rpy, xyz);
    plant.SetFreeBodyPoseInWorldFrame(
      &plant_context, plant.GetBodyByName(
        "base_link"), X_WB);
    simulator.AdvanceTo(simulator_context.get_time() + 0.01);
    diagram->Publish(*context);
  }

  return 0;
}
