#include <botanbot_control/mpc_controller/mpc_controller.hpp>

MPCController::MPCController(/* args */)
{
  opti_ = std::make_shared<casadi::Opti>();

  Q = Q.diag(vector_Q);
  R = R.diag(vector_R);


  std::cout << "Q diag: " << Q << std::endl;
  std::cout << "R diag: " << R << std::endl;

  u_prev_ = opti_->parameter(2);
  z_curr_ = opti_->parameter(4);
}

MPCController::~MPCController()
{
}
