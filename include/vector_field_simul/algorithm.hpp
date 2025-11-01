#ifndef ALGORITHM
#define ALGORITHM

#include <array>

/**
* alpha = (x-xc)^2 + (y-yc)^2 - r^2
* grad(alpha) = 2[x-xc, y-yc]
*
* V = 0.5*alpha^2
*
* @return [u1, u2, V]
*/
std::array<double, 3> circle_trajectory(
  double xc,
  double xy,
  double radius,
  double pos_x,
  double pos_y
);

/**
* P = M^-1 * a
*
* @return [u1, u2, V]
*/
std::array<double, 3> moving_trajectory(
  double xc0,
  double xy0,
  double radius,
  double pos_x,
  double pos_y,
  double vel_x,
  double time
);

#endif // !ALGORITHM
