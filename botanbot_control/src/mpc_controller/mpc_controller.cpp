#include <botanbot_control/mpc_controller/mpc_controller.hpp>

MPCController::MPCController(/* args */)
{
  opti_ = std::make_shared<casadi::Opti>();

  Q = Q.diag(vector_Q);
  R = R.diag(vector_R);

  // previous control input
  u_prev_ = opti_->parameter(2);
  // curr state
  z_curr_ = opti_->parameter(4);

  // reference traj that we would like to follow
  z_ref_ = opti_->parameter(N, 4);
  x_ref_ = z_ref_(slice_all_, 0);
  y_ref_ = z_ref_(slice_all_, 1);
  psi_ref_ = z_ref_(slice_all_, 2);
  v_ref_ = z_ref_(slice_all_, 3);

  // Decision vars
  z_dv_ = opti_->variable(N + 1, 4);
  x_dv_ = z_dv_(slice_all_, 0);
  y_dv_ = z_dv_(slice_all_, 1);
  psi_dv_ = z_dv_(slice_all_, 2);
  v_dv_ = z_dv_(slice_all_, 3);

  // Control inputs used to achieve z_dv
  u_dv_ = opti_->variable(N, 2);
  acc_dv_ = u_dv_(slice_all_, 0);
  df_dv_ = u_dv_(slice_all_, 1);

  // Slack variables used to relax input rate constraints.
  sl_dv_ = opti_->variable(N, 2);
  sl_acc_dv_ = sl_dv_(slice_all_, 0);
  sl_df_dv_ = sl_dv_(slice_all_, 1);

  std::cout << "Q diag: " << Q << std::endl;
  std::cout << "R diag: " << R << std::endl;
  std::cout << "z_ref_: " << z_ref_.dim() << std::endl;
  std::cout << "x_ref_: " << x_ref_.dim() << std::endl;
  std::cout << "y_ref_: " << y_ref_.dim() << std::endl;
  std::cout << "psi_ref_: " << psi_ref_.dim() << std::endl;
  std::cout << "v_ref_: " << v_ref_.dim() << std::endl;

  addConstraints();
  addCost();
  updateInitialCondition(0.0, 0.0, 0.0, 1.0);

  std::vector<double> mock_x_ref, mock_y_ref, mock_psi_ref, mock_v_ref;
  for (int i = 0; i < N; i++) {
    mock_x_ref.push_back(0.0);
    mock_y_ref.push_back(0.0);
    mock_psi_ref.push_back(0.0);
    mock_v_ref.push_back(0.0);
  }
  updateReference(mock_x_ref, mock_y_ref, mock_psi_ref, mock_v_ref);
  updatePreviousInput(0.0, 0.0);
  std::vector<double> mock_sl_acc_dv, mock_sl_df_dv;
  for (int i = 0; i < N; i++) {
    mock_sl_acc_dv.push_back(0.0);
    mock_sl_df_dv.push_back(0.0);
  }
  updateSlackVars(mock_sl_acc_dv, mock_sl_df_dv);

  opti_->solver("ipopt");
  solve();

}

MPCController::~MPCController()
{
}

void MPCController::addConstraints()
{
  //  State Bound Constraints
  opti_->subject_to(opti_->bounded(V_MIN, v_dv_, V_MAX));

  // Initial state constraints
  opti_->subject_to(x_dv_(0) == z_curr_(0));
  opti_->subject_to(y_dv_(0) == z_curr_(1));
  opti_->subject_to(psi_dv_(0) == z_curr_(2));
  opti_->subject_to(v_dv_(0) == z_curr_(3));

  // state dynamics constraints
  for (int i = 0; i < N; i++) {
    auto beta = casadi::MX::atan(L_R / (L_F + L_R) * casadi::MX::cos(df_dv_(i)));
    opti_->subject_to(
      x_dv_(i + 1) ==
      x_dv_(i) + DT * (v_dv_(i) * casadi::MX::cos(psi_dv_(i) + beta)));
    opti_->subject_to(
      y_dv_(i + 1) ==
      y_dv_(i) + DT * (v_dv_(i) * casadi::MX::sin(psi_dv_(i) + beta)));
    opti_->subject_to(
      psi_dv_(i + 1) ==
      psi_dv_(i) + DT * (v_dv_(i) / L_R * casadi::MX::sin(beta)));
    opti_->subject_to(
      v_dv_(i + 1) ==
      v_dv_(i) + DT * acc_dv_(i));
  }

  //  Input Bound Constraints
  opti_->subject_to(opti_->bounded(A_MIN, acc_dv_, A_MAX));
  opti_->subject_to(opti_->bounded(DF_MIN, df_dv_, DF_MAX));

  //  Input Rate Bound Constraints
  opti_->subject_to(
    opti_->bounded(
      A_DOT_MIN * DT - sl_acc_dv_(0),
      acc_dv_(0) - u_prev_(0),
      A_DOT_MAX * DT + sl_acc_dv_(0)));

  opti_->subject_to(
    opti_->bounded(
      DF_DOT_MIN * DT - sl_df_dv_(0),
      df_dv_(0) - u_prev_(1),
      DF_DOT_MAX * DT + sl_df_dv_(0)));

  for (int i = 0; i < N - 1; i++) {
    opti_->subject_to(
      opti_->bounded(
        A_DOT_MIN * DT - sl_acc_dv_(i + 1),
        acc_dv_(i + 1) - acc_dv_(i),
        A_DOT_MAX * DT + sl_acc_dv_(i + 1)));

    opti_->subject_to(
      opti_->bounded(
        DF_DOT_MIN * DT - sl_df_dv_(i + 1),
        df_dv_(i + 1) - df_dv_(i),
        DF_DOT_MAX * DT + sl_df_dv_(i + 1)));
  }

  //  Other Constraints
  opti_->subject_to(0 <= sl_df_dv_);
  opti_->subject_to(0 <= sl_acc_dv_);

  //e .g. things like collision avoidance or lateral acceleration bounds could go here.

}

void MPCController::addCost()
{
  auto quad_form = [](casadi::MX z, casadi::Matrix<double> Q)
    {
      return casadi::MX::mtimes(z, casadi::MX::mtimes(Q, z.T()));
    };

  casadi::MX cost = 0.0;

  // tracking cost cost
  for (int i = 0; i < N; i++) {
    cost += quad_form(z_dv_(i + 1, slice_all_) - z_ref_(i, slice_all_), Q);
  }

  // input derivative cost
  for (int i = 0; i < N - 1; i++) {
    cost += quad_form(u_dv_(i + 1, slice_all_) - u_dv_(i, slice_all_), R);
  }

  // slack cost
  cost += (casadi::MX::sum1(sl_df_dv_) + casadi::MX::sum1(sl_acc_dv_));

  opti_->minimize(cost);
}

SolutionResult MPCController::solve()
{
  bool is_solution_optimal(false);
  auto start = std::chrono::high_resolution_clock::now();
  casadi::native_DM u_mpc, z_mpc, sl_mpc, z_ref;
  try {
    auto sol = opti_->solve();
    u_mpc = sol.value(u_dv_);
    z_mpc = sol.value(z_dv_);
    sl_mpc = sol.value(sl_dv_);
    z_ref = sol.value(z_ref_);
    is_solution_optimal = true;
  } catch (const std::exception & e) {
    std::cout << "NON optimal solution" << '\n';
    u_mpc = opti_->debug().value(u_dv_);
    z_mpc = opti_->debug().value(z_dv_);
    sl_mpc = opti_->debug().value(sl_dv_);
    z_ref = opti_->debug().value(z_ref_);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto execution_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "Solve took: " << execution_time << " ms" << std::endl;


  SolutionResult res;
  res.solve_time_ms = execution_time;
  res.is_optimal = is_solution_optimal;
  res.control_input = std::make_pair<double, double>(u_mpc(0, 0).scalar(), u_mpc(0, 1).scalar());

  return res;
}

void MPCController::updateInitialCondition(double x0, double y0, double psi0, double v0)
{
  opti_->set_value(z_curr_, {x0, y0, psi0, v0});
  std::cout << "Initial states are : " << opti_->debug().value(z_curr_) << std::endl;
}

void MPCController::updateReference(
  std::vector<double> x_ref,
  std::vector<double> y_ref,
  std::vector<double> psi_ref,
  std::vector<double> v_ref)
{
  opti_->set_value(x_ref_, x_ref);
  opti_->set_value(y_ref_, y_ref);
  opti_->set_value(psi_ref_, psi_ref);
  opti_->set_value(v_ref_, v_ref);
}

void MPCController::updateSlackVars(std::vector<double> sl_acc_dv, std::vector<double> sl_df_dv)
{
  opti_->set_initial(sl_dv_(slice_all_, 0), sl_acc_dv);
  opti_->set_initial(sl_dv_(slice_all_, 1), sl_df_dv);
}

void MPCController::updatePreviousInput(double acc_prev, double df_prev)
{
  opti_->set_value(u_prev_, {acc_prev, df_prev});
}

void MPCController::update(
  std::vector<double> inital_condition,
  std::vector<std::vector<double>> references,
  std::vector<double> previous_inputs)
{
  updateInitialCondition(
    inital_condition[0],
    inital_condition[1],
    inital_condition[2],
    inital_condition[3]);
  updateReference(
    references[0],
    references[1],
    references[2],
    references[3]);
  updatePreviousInput(
    previous_inputs[0],
    previous_inputs[1]);
}
