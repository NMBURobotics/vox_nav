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

#include <botanbot_control/mpc_controller/mpc_controller.hpp>

#include <botanbot_control/mpc_controller/mpc_acado/acado_common.h>
#include <botanbot_control/mpc_controller/mpc_acado/acado_auxiliary_functions.h>


ACADOworkspace acadoWorkspace;
ACADOvariables acadoVariables;

namespace botanbot_control
{
namespace mpc_controller
{

using AcadoReal = real_t;
constexpr auto HORIZON = static_cast<std::size_t>(ACADO_N);
// State variable indices
static_assert(ACADO_NX == 6, "Unexpected num of state variables");
constexpr auto NX = static_cast<std::size_t>(ACADO_NX);
constexpr auto IDX_X = 0U;
constexpr auto IDX_Y = 1U;
constexpr auto IDX_HEADING = 2U;
constexpr auto IDX_VEL_LONG = 3U;
constexpr auto IDX_ACCEL = 4U;
constexpr auto IDX_WHEEL_ANGLE = 5U;
// Control variable indices
static_assert(ACADO_NU == 2, "Unexpected num of control variables");
constexpr auto NU = static_cast<std::size_t>(ACADO_NU);
constexpr auto IDX_JERK = 0U;
constexpr auto IDX_WHEEL_ANGLE_RATE = 1U;

MPCController::MPCController()
{
  // Reset all solver memory
  std::memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  std::memset(&acadoVariables, 0, sizeof( acadoVariables ));

}

MPCController::~MPCController()
{
}

void MPCController::solve()
{
  const auto prep_ret = acado_preparationStep();
  if (0 != prep_ret) {
    std::string err_str{"Solver preparation error: ", std::string::allocator_type{}};
    err_str += std::to_string(prep_ret);
    throw std::runtime_error{err_str};
  }
  const auto solve_ret = acado_feedbackStep();
  if (0 != solve_ret) {
    std::string err_str{"Solver error: ", std::string::allocator_type{}};
    err_str += std::to_string(solve_ret);
    throw std::runtime_error{err_str};
  }
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  auto current_idx = 0;
  const auto dt = 0.001;
  nav_msgs::msg::Path ref_path;

  const auto max_pts = ref_path.poses.size();

  const auto horizon = std::min(static_cast<std::size_t>(max_pts - current_idx), HORIZON);
  // Consider different ways of updating initial guess for reference update

  std::fill(&acadoVariables.u[0U], &acadoVariables.u[HORIZON * NU], AcadoReal{});
  acado_initializeNodesByForwardSimulation();

  solve();

  const auto idx = static_cast<std::size_t>(count) * static_cast<std::size_t>(NX);
  const auto longitudinal0 = static_cast<Real>(acadoVariables.x[idx + IDX_ACCEL]);
  const auto lateral0 = static_cast<Real>(acadoVariables.x[idx + IDX_WHEEL_ANGLE]);
  const auto jdx = (static_cast<std::size_t>(count) + 1U) * static_cast<std::size_t>(NX);
  const auto longitudinal1 = static_cast<Real>(acadoVariables.x[jdx + IDX_ACCEL]);
  const auto lateral1 = static_cast<Real>(acadoVariables.x[jdx + IDX_WHEEL_ANGLE]);

  return geometry_msgs::msg::TwistStamped();

}


}  // namespace mpc_controller

}  // namespace botanbot_control
