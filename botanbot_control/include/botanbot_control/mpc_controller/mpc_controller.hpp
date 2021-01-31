#include <casadi/casadi.hpp>
#include <chrono>

struct SolutionResult
{
  bool is_optimal;
  typedef std::pair<double, double> control_input_t;
  int solve_time_ms;
  control_input_t control_input;
};

class MPCController
{
private:
  std::shared_ptr<casadi::Opti> opti_;

  int N = 20;                                                // timesteps in MPC Horizon
  double DT = 0.1;                                          // discretization time between timesteps(s)
  double L_F = 0.66;                                        // distance from CoG to front axle(m)
  double L_R = 0.66;                                        // distance from CoG to rear axle(m)
  double V_MIN = -10.0;                                     // min / max velocity constraint(m / s)
  double V_MAX = 10.0;
  double A_MIN = -2.0;                                      // min / max acceleration constraint(m / s ^ 2)
  double A_MAX = 2.0;
  double DF_MIN = -0.6;                                     // min / max front steer angle constraint(rad)
  double DF_MAX = 0.6;
  double A_DOT_MIN = -1.5;                                  // min / max jerk constraint(m / s ^ 3)
  double A_DOT_MAX = 1.5;
  double DF_DOT_MIN = -0.5;                                 // min / max front steer angle rate constraint(rad / s)
  double DF_DOT_MAX = 0.5;
  std::vector<double> vector_Q = {1.0, 1.0, 10.0, 0.1};     // weights on x, y, psi, and v.
  std::vector<double> vector_R = {10.0, 100.0};             // weights on jerk and slew rate(steering angle derivative)

  // used to slice casadi matrixes
  casadi::Slice slice_all_;

  // We need weight matrixes as diagonal
  casadi::Matrix<double> Q;
  casadi::Matrix<double> R;

  casadi::MX u_prev_;   // previous input: [u_{acc, -1}, u_{df, -1}]
  casadi::MX z_curr_;   // current state:  [x_0, y_0, psi_0, v_0]

  // reference traj that we would like to follow
  // First index corresponds to our desired state at timestep k+1:
  // i.e. z_ref_[0,:] = z_{desired, 1}.
  // second index selects the state element from [x_k, y_k, psi_k, v_k].
  casadi::MX z_ref_;
  casadi::MX x_ref_;
  casadi::MX y_ref_;
  casadi::MX psi_ref_;
  casadi::MX v_ref_;

  // Decision vars
  // Actual trajectory we will follow given the optimal solution.
  // First index is the timestep k, i.e. self.z_dv[0,:] is z_0.
  // It has N+1 timesteps since we go from z_0, ..., z_N.
  // Second index is the state element, as detailed below.
  casadi::MX z_dv_;
  casadi::MX x_dv_;
  casadi::MX y_dv_;
  casadi::MX psi_dv_;
  casadi::MX v_dv_;

  // Control inputs used to achieve z_dv according to dynamics.
  // First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
  // Second index is the input element as detailed below.
  casadi::MX u_dv_;
  casadi::MX acc_dv_;
  casadi::MX df_dv_;

  // Slack variables used to relax input rate constraints.
  // Matches self.u_dv in structure but timesteps range from -1, ..., N-1.
  casadi::MX sl_dv_;
  casadi::MX sl_acc_dv_;
  casadi::MX sl_df_dv_;

public:
  MPCController(/* args */);
  ~MPCController();

  void addConstraints();
  void addCost();
  SolutionResult solve();
  void updateInitialCondition(double x0, double y0, double psi0, double v0);
  void updateReference(
    std::vector<double> x_ref,
    std::vector<double> y_ref,
    std::vector<double> psi_ref,
    std::vector<double> v_ref);
  void updatePreviousInput(
    double acc_prev,
    double df_prev);
  void updateSlackVars(
    std::vector<double> sl_acc_dv,
    std::vector<double> sl_df_dv);
  void update(
    std::vector<double> inital_condition,
    std::vector<std::vector<double>> references,
    std::vector<double> previous_inputs);
};
