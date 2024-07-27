// Copyright 2024 Fool Stuck Engineers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WHEEL_STUCK_UTILS__MATH__MATH_HPP_
#define WHEEL_STUCK_UTILS__MATH__MATH_HPP_

#include <algorithm>

namespace wheel_stuck_utils::math
{
inline double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

inline double lerp(const double a, const double b, const double t)
{
  return a + clamp01(t) * (b - a);
}

// lerpの逆関数
inline double inverse_lerp(const double a, const double b, const double value)
{
  if (a != b) {
    return clamp01((value - a) / (b - a));
  } else {
    return 0;  // ゼロ除算のため値を定義し得ないが、便宜上0を返す。
  }
}

// 符号(-1, 0, 1)を返す
inline int sign(const double i)
{
  return (i > 0) ? +1 : (i < 0) ? -1 : 0;
  /*
  if(i > 0)
    return 1;
  else if(i < 0)
    return -1;
  else
    return 0;
  */
}

// 可変長引数を受け取り最小値を返す
template <typename T>
inline T min(T a)
{
  return a;
}

template <typename T, typename... Args>
inline T min(T a, Args... args)
{
  return std::min(a, min(args...));
}

// 可変長引数を受け取り最大値を返す
template <typename T>
inline T max(T b)
{
  return b;
}

template <typename T, typename... Args>
inline T max(T b, Args... args)
{
  return std::max(b, max(args...));
}

}  // namespace wheel_stuck_utils::math

#endif  // WHEEL_STUCK_UTILS__MATH__MATH_HPP_
