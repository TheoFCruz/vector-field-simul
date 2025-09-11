#ifndef ALGORITHM
#define ALGORITHM

#include <array>

// Circulation velocity
const double VTAN = 0.4;

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

#endif // !ALGORITHM
