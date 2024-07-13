#ifndef WHEEL_STUCK_UTILS__MATH__MATH_HPP_
#define WHEEL_STUCK_UTILS__MATH__MATH_HPP_

#include <algorithm>

namespace wheel_stuck_utils::math
{
inline double lerp(const double a, const double b, double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return a + t * (b - a);
}
}  // namespace wheel_stuck_utils::math

#endif  // WHEEL_STUCK_UTILS__MATH__MATH_HPP_
