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

ACADOworkspace acadoWorkspace;
ACADOvariables acadoVariables;

namespace botanbot_control
{
namespace mpc_controller
{

using AcadoReal = real_t;

MPCController::MPCController()
{
}

MPCController::~MPCController()
{
}

}  // namespace mpc_controller

}  // namespace botanbot_control
