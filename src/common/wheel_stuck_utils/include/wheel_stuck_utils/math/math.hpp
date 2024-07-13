#ifndef WHEEL_STUCK_UTILS__MATH__MATH_HPP_
#define WHEEL_STUCK_UTILS__MATH__MATH_HPP_

#include <algorithm>

namespace wheel_stuck_utils::math
{
inline double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

inline double lerp(const double a, const double b, double t)
{
  t = clamp01(t);
  return a + t * (b - a);
}

}  // namespace wheel_stuck_utils::math

#endif  // WHEEL_STUCK_UTILS__MATH__MATH_HPP_
