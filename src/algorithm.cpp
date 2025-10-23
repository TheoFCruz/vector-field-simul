#include "algorithm.hpp"
#include <array>
#include <cstdio>

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

std::array<double, 2> moving_trajectory(
  double xc0,
  double yc0,
  double radius,
  double pos_x,
  double pos_y,
  double vel_x,
  double time)
{
  double dx = pos_x - (xc0 + vel_x*time);
  double dy = pos_y - yc0;

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

  // Time varying correction term
  std::array<double, 2> P;
  
  double r2 = dx*dx + dy*dy;
  P[0] = (vel_x*dx/r2)*dx;
  P[1] = (vel_x*dx/r2)*dy;

  // Getting the input
  std::array<double, 2> control_input;

  control_input[0] = -G*alpha*grad[0] + H*wedge[0] + P[0];
  control_input[1] = -G*alpha*grad[1] + H*wedge[1] + P[1];

  return control_input;
}
