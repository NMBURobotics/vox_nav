#include <casadi/casadi.hpp>

class MPCController
{
private:
  std::shared_ptr<casadi::Opti> opti_;

  int N = 10;                                    // timesteps in MPC Horizon
  double DT = 0.2;                               // discretization time between timesteps(s)
  double L_F = 1.5213;                           // distance from CoG to front axle(m)
  double L_R = 1.4987;                           // distance from CoG to rear axle(m)
  double V_MIN = 0.0;                            // min / max velocity constraint(m / s)
  double V_MAX = 20.0;
  double A_MIN = -3.0;                           // min / max acceleration constraint(m / s ^ 2)
  double A_MAX = 2.0;
  double DF_MIN = -0.5;                          // min / max front steer angle constraint(rad)
  double DF_MAX = 0.5;
  double A_DOT_MIN = -1.5;                       // min / max jerk constraint(m / s ^ 3)
  double A_DOT_MAX = 1.5;
  double DF_DOT_MIN = -0.5;                                 // min / max front steer angle rate constraint(rad / s)
  double DF_DOT_MAX = 0.5;
  std::vector<double> vector_Q = {1.0, 1.0, 10.0, 0.1};     // weights on x, y, psi, and v.
  std::vector<double> vector_R = {10.0, 100.0};             // weights on jerk and slew rate(steering angle derivative)

  // We need weight matrixes as diagonal
  casadi::Matrix<double> Q;
  casadi::Matrix<double> R;

  casadi::MX u_prev_;   // previous input: [u_{acc, -1}, u_{df, -1}]
  casadi::MX z_curr_;   // current state:  [x_0, y_0, psi_0, v_0]

public:
  MPCController(/* args */);
  ~MPCController();
};
