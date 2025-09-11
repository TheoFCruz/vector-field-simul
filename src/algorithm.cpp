#include "algorithm.hpp"

std::array<double, 2> circle_trajectory(
  double xc,
  double yc,
  double radius,
  double pos_x,
  double pos_y)
{
  double dx = pos_x - xc;
  double dy = pos_y - yc;

  // Convergence term
  double alpha = dx*dx + dy*dy - radius*radius;

  std::array<double, 2> grad;
  grad[0] = 2*dx;
  grad[1] = 2*dy;

  double G = 0.1;

  // Circulation term
  std::array<double, 2> wedge;
  wedge[0] = -grad[1];
  wedge[1] = grad[0];

  double H = VTAN/(2*radius);

  // Getting the input
  std::array<double, 2> control_input;

  control_input[0] = -G*alpha*grad[0] + H*wedge[0];
  control_input[1] = -G*alpha*grad[1] + H*wedge[1];

  return control_input;
}
