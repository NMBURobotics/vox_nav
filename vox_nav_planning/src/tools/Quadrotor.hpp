// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

// credit: https://github.com/AtsushiSakai/PythonRobotics/blob/master/AerialNavigation/drone_3d_trajectory_following/Quadrotor.py
// credit: Daniel Ingram (daniel-s-ingram)

#include <Eigen/Dense>

class QuadrotorControl
{
private:
// Simulation parameters
  double g = 1.0;
  double m = 1.0;
  double Ixx = 1;
  double Iyy = 1;
  double Izz = 1;

// Proportional coefficients
  double Kp_x = 1;
  double Kp_y = 1;
  double Kp_z = 1;
  double Kp_roll = 25;
  double Kp_pitch = 25;
  double Kp_yaw = 25;

// Derivative coefficients
  double Kd_x = 10;
  double Kd_y = 10;
  double Kd_z = 10;

public:
  QuadrotorControl()
  {

  }
  ~QuadrotorControl() {}

  std::vector<double> step(
    double x_pos, // curr x
    double y_pos, // curr y
    double z_pos, // curr z
    double roll,      // curr roll
    double pitch, // curr pitch
    double yaw,   // curr yaw
    double x_vel, // curr x vel
    double y_vel, // curr y vel
    double z_vel, // curr z vel
    double roll_vel,     // curr roll vel
    double pitch_vel, // curr pitch vel
    double yaw_vel,   // curr yaw vel
    double x_acc, // curr x acc
    double y_acc, // curr y acc
    double z_acc, // curr z acc
    double des_z_pos, // desired z pos
    double des_yaw,   // desired yaw
    double des_z_vel, // desired z vel
    double des_x_acc, // desired x acc
    double des_y_acc, // desired y acc
    double des_z_acc, // desired z acc
    double dt // time step size
  )
  {
    double thrust = m * (g + des_z_acc + Kp_z * (des_z_pos - z_pos) + Kd_z * (des_z_vel - z_vel));

    double roll_torque = Kp_roll *
      (((des_x_acc * sin(des_yaw) - des_y_acc * cos(des_yaw)) / g) - roll);
    double pitch_torque = Kp_pitch *
      (((des_x_acc * cos(des_yaw) - des_y_acc * sin(des_yaw)) / g) - pitch);
    double yaw_torque = Kp_yaw * (des_yaw - yaw);

    // Update the ang velocities
    roll_vel += roll_torque * dt / Ixx;
    pitch_vel += pitch_torque * dt / Iyy;
    yaw_vel += yaw_torque * dt / Izz;

    // Update the angles
    roll += roll_vel * dt;
    pitch += pitch_vel * dt;
    yaw += yaw_vel * dt;

    // get rotation matrix from angles
    auto R = rotation_matrix(roll, pitch, yaw);

    // calculate linear acceleration
    Eigen::Vector3d acc =
      (R * Eigen::Vector3d(0, 0, thrust) - Eigen::Vector3d(0, 0, m * g)) / m;

    x_acc = acc[0];
    y_acc = acc[1];
    z_acc = acc[2];

    // Update velocities based on new acceleration
    x_vel += x_acc * dt;
    y_vel += y_acc * dt;
    z_vel += z_acc * dt;

    // Update positions based on new velocities
    x_pos += x_vel * dt;
    y_pos += y_vel * dt;
    z_pos += z_vel * dt;

    // return new states
    std::vector<double> quadrotor_state = {x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel,
      z_vel, roll_vel, pitch_vel, yaw_vel, x_acc, y_acc, z_acc};

    // return new states
    return quadrotor_state;

  }

  Eigen::Matrix3d rotation_matrix(double roll, double pitch, double yaw)
  {
    Eigen::Matrix3d R;
    R << cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
      sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll),
      sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll),
      -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll),
      -sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw);
    return R;
  }

};
