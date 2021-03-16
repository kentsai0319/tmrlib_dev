#pragma once

#include "tmrl/types.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace tmrl
{
namespace utils
{

inline double deg(double ang) { return (180.0 / M_PI) * ang; }
inline double rad(double ang) { return (M_PI / 180.0) * ang; }

inline vectorXd degs(const vectorXd &angs)
{
  vectorXd rv(angs.size());
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = deg(angs[i]); }
  return rv;
}
template<typename T, std::size_t N>
inline std::array<T, N> degs(const std::array<T, N> &angs)
{
  typename std::array<T, N> rv;
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = deg(angs[i]); }
  return rv;
}

inline vectorXd rads(const vectorXd &angs)
{
  vectorXd rv(angs.size());
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = rad(angs[i]); }
  return rv;
}
template<typename T, std::size_t N>
inline std::array<T, N> rads(const std::array<T, N> &angs)
{
  typename std::array<T, N> rv;
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = rad(angs[i]); }
  return rv;
}

inline PoseEular mmdeg(const PoseEular &pose)
{
  PoseEular rv;
  for (size_t i = 0; i < 3; ++i) { rv[i] = 1000.0 * pose[i]; }
  for (size_t i = 3; i < 6; ++i) { rv[i] =  deg(pose[i]); }
  return rv;
}
inline PoseEular m_rad(const PoseEular &pose)
{
  PoseEular rv;
  for (size_t i = 0; i < 3; ++i) { rv[i] = 0.001 * pose[i]; }
  for (size_t i = 3; i < 6; ++i) { rv[i] =  rad(pose[i]); }
  return rv;
}

}
}