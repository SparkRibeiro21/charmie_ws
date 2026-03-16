#pragma once
#include <cmath>
#include <algorithm>

namespace sdnl
{
inline double wrap_pi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));  // [-pi, pi]
}

inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}
}  // namespace sdnl
