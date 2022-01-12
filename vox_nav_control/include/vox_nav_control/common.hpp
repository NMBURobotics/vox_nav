// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_CONTROL__COMMON_HPP_
#define VOX_NAV_CONTROL__COMMON_HPP_

#include <Eigen/Eigen>

namespace vox_nav_control
{

  /**
     * @brief We model obstacles with ellipsoids
     * They are then used in opti stack
     *
     */
  struct Ellipsoid
  {
    Eigen::Vector2f center;
    Eigen::Vector2f axes;
    bool is_dynamic;
    double heading;
  };

  /**
   * @brief a struct to keep states organized
   *
   */
  struct States
  {
    double x;     // X position
    double y;     // Y position
    double psi;     // heading angle
    double v;     // linear velocity
    States()
    : x(0.0),
      y(0.0),
      psi(0.0),
      v(0.0) {}
  };

  /**
   * @brief a struct to keep control inputs organized
   *
   */
  struct ControlInput
  {
    double acc;     // accelration command
    double df;     // steering angle command
    ControlInput()
    : acc(0.0),
      df(0.0) {}
  };

  /**
 * @brief all parameters used by MPC class,
 * user needs to create and reconfigure this
 *
 */
  struct Parameters
  {
    // timesteps in MPC Horizon
    int N;
    // discretization time between timesteps(s)
    double DT;
    // distance from CoG to front axle(m)
    double L_F;
    // distance from CoG to rear axle(m)
    double L_R;
    // min / max velocity constraint(m / s)
    double V_MIN;
    double V_MAX;
    // min / max acceleration constraint(m / s ^ 2)
    double A_MIN;
    double A_MAX;
    // min / max front steer angle constraint(rad)
    double DF_MIN;
    double DF_MAX;
    // min / max jerk constraint(m / s ^ 3)
    double A_DOT_MIN;
    double A_DOT_MAX;
    // min / max front steer angle rate constraint(rad / s)
    double DF_DOT_MIN;
    double DF_DOT_MAX;
    // weights on x, y, psi, and v.
    std::vector<double> Q;
    // weights on jerk and skew rate(steering angle derivative)
    std::vector<double> R;
    // enable/disable debug messages
    bool debug_mode;
    // set this true only if user figured the configration
    bool params_configured;

    // Assign meaningful default values to this parameters
    Parameters()
    : N(10),
      DT(0.1),
      L_F(0.65),
      L_R(0.65),
      V_MIN(-10.0),
      V_MAX(10.0),
      A_MIN(-1.0),
      A_MAX(1.0),
      DF_MIN(-0.5),
      DF_MAX(0.5),
      A_DOT_MIN(-1.0),
      A_DOT_MAX(1.0),
      DF_DOT_MIN(-0.5),
      DF_DOT_MAX(0.5),
      Q({100.0, 100.0, 10.0, 0.1}),
      R({10.0, 10.0}),
      debug_mode(true),
      params_configured(false) {}
  };

}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__COMMON_HPP_
