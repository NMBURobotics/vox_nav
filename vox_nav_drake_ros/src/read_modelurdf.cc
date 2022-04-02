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


using drake::symbolic::Expression;
using drake::symbolic::Monomial;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::symbolic::internal::DegreeType;


class Robot : public drake::systems::LeafSystem<double>
{
public:
  Robot()
  {
    DeclareVectorInputPort("controls", 2);
    auto state_index = this->DeclareContinuousState(3);
    DeclareStateOutputPort("states", state_index);
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
      "controls_meshcat", 2, &MeshcatSliders::calc_output_value,
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

  std::string vox_nav_drake_ros_package_path =
    ament_index_cpp::get_package_share_directory("vox_nav_drake_ros");

  drake::systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.001 /* time_step */);
  const std::string & pathname = vox_nav_drake_ros_package_path + "/urdf/botanbot.urdf";
  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile(pathname);

  const drake::multibody::PrismaticJoint<double> & bot_x =
    plant.GetJointByName<drake::multibody::PrismaticJoint>("x");
  const drake::multibody::PrismaticJoint<double> & bot_y =
    plant.GetJointByName<drake::multibody::PrismaticJoint>("y");
  const drake::multibody::RevoluteJoint<double> & bot_theta =
    plant.GetJointByName<drake::multibody::RevoluteJoint>("theta");

  plant.Finalize();

  std::cout << plant.get_actuation_input_port().size() << std::endl;
  std::cout << plant.get_actuation_input_port().get_data_type() << std::endl;
  std::cout << plant.get_actuation_input_port().size() << std::endl;


  drake::geometry::SceneGraph<double> * scene_graph_{};
  auto meshcat_ = std::make_shared<drake::geometry::Meshcat>();
  drake::geometry::MeshcatVisualizerParams params = {};
  scene_graph_ = &scene_graph;


  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
    &builder, scene_graph, meshcat_, std::move(params));

  meshcat_->AddSlider("v", -5, 5, 0.1, 0.0);
  meshcat_->AddSlider("phi", -1, 1, 0.05, 0.0);
  std::vector<std::string> slider_names = {"v", "phi"};

  auto input_system_meshcat = builder.AddSystem<MeshcatSliders>(meshcat_, slider_names);

  auto robot = builder.AddSystem<Robot>();
  robot->set_name("robot");


  auto zero_torque =
    builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
    Eigen::VectorXd::Zero(3));

  builder.Connect(
    zero_torque->get_output_port(),
    plant.get_actuation_input_port());


  auto diagram = builder.Build();


  std::cout << zero_torque->get_output_port().size() << std::endl;
  std::cout << zero_torque->get_output_port().get_data_type() << std::endl;


  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  // Set the initial conditions (x, y, theta)
  auto context = diagram->CreateDefaultContext();
  context->SetContinuousState(x0);

  drake::systems::Context<double> & plant_context =
    diagram->GetMutableSubsystemContext(plant, context.get());

  drake::systems::Simulator<double> simulator(plant);

  simulator.Initialize();
  while (1) {
    //diagram->Publish(*context);
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.1);
    /*bot_x.set_translation(&plant_context, meshcat_->GetSliderValue("v"));
    bot_y.set_translation(&plant_context, simulator.get_context().get_time() * 0.001);
    bot_theta.set_angle(&plant_context, meshcat_->GetSliderValue("phi"));*/
  }

  return 0;
}
