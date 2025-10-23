#ifndef ALGORITHM
#define ALGORITHM

#include <array>

// Circulation velocity
const double VTAN = 0.3;

/**
* alpha = (x-xc)^2 + (y-yc)^2 - r^2
* grad(alpha) = 2[x-xc, y-yc]
*
* V = 0.5*alpha^2
*/
std::array<double, 2> circle_trajectory(
  double xc,
  double xy,
  double radius,
  double pos_x,
  double pos_y
);

std::array<double, 2> moving_trajectory(
  double xc0,
  double xy0,
  double radius,
  double pos_x,
  double pos_y,
  double vel_x,
  double time
);

#endif // !ALGORITHM
