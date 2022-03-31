#include <memory>
#include <utility>
#include <vector>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <drake/common/symbolic_latex.h>
#include <rclcpp/rclcpp.hpp>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

using drake::symbolic::Expression;
using drake::symbolic::Monomial;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::symbolic::internal::DegreeType;


int main(int argc, char const * argv[])
{
  drake::solvers::MathematicalProgram prog_;

  // Find the Lyapunov function V(x) for system:
  //
  //     ẋ₀ = -x₁ + 1.5x₀² - 0.5x₀³
  //     ẋ₁ = 3x₀ - x₁
  //
  // by solving sum-of-squared problems:
  //
  //     V(x) is sum-of-squares
  //    -V̇(x) is sum-of-squares
  drake::VectorX<Variable> x = prog_.NewIndeterminates<2>();
  const auto & x0 = x(0);
  const auto & x1 = x(1);

  // Form the dynamics of the system.
  drake::Vector2<drake::symbolic::Polynomial> dynamics;
  /* *INDENT-OFF* */
  dynamics << drake::symbolic::Polynomial{-x1 + 1.5 * x0 * x0 - 0.5 * pow(x0, 3)},
              drake::symbolic::Polynomial{3 * x0 - x1};
  /* *INDENT-ON* */

  // Adds V(x) as a 4th order sum-of-squares polynomial.
  const drake::symbolic::Polynomial V{prog_.NewSosPolynomial({x0, x1}, 4).first};

  // Computes Vdot.
  const drake::symbolic::Polynomial Vdot = V.Jacobian(x).transpose().dot(dynamics);

  // -Vdot is sum-of-squares.
  prog_.AddSosConstraint(-Vdot);
  auto result_ = drake::solvers::Solve(prog_);

  std::cout << result_.GetSolution() << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << V.decision_variables().to_string() << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << drake::symbolic::ToLatex(V.ToExpression()) << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << drake::symbolic::ToLatex(Vdot.ToExpression()) << std::endl;
  std::cout << "=====================" << std::endl;

  std::cout <<
    drake::symbolic::ToLatex(
    drake::symbolic::Polynomial(result_.GetSolution(V)).RemoveTermsWithSmallCoefficients(1e-5).
    ToExpression()) << std::endl;


  //
  // Find the Lyapunov function V(x) for system:
  //
  //     ẋ₀ = -x₁ + 1.5x₀² - 0.5x₀³
  //     ẋ₁ = 3x₀ - x₁
  //     ẋ₂ = '2'

  // by solving sum-of-squared problems:
  //
  //     V(x) is sum-of-squares
  //    -V̇(x) is sum-of-squares


  return 0;
}
