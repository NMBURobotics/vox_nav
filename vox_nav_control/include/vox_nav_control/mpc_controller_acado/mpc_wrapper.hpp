#include <Eigen/Eigen>

#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_common.h>
#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_auxiliary_functions.h>

namespace vox_nav_control
{
  namespace mpc_controller_acado
  {

    static constexpr int kSamples = ACADO_N;        // number of samples
    static constexpr int kStateSize = ACADO_NX; // number of states
    static constexpr int kRefSize = ACADO_NY; // number of reference states
    static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
    static constexpr int kInputSize = ACADO_NU; // number of inputs
    static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
    static constexpr int kOdSize = ACADO_NOD; // number of online data

    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;


  }
}
