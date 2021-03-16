#pragma once

#include <vector>
#include <array>
#include <string>
#include <sstream>

namespace tmrl
{

using vectorXd = std::vector<double>;

using vector3d = std::array<double, 3>;
using vector6d = std::array<double, 6>;
using vector7d = std::array<double, 7>;

using PoseEular = std::array<double, 6>;
using PoseVec = std::array<double, 6>;
using PoseQ = std::array<double, 7>;


template<typename T, std::size_t N>
inline std::vector<T> to_vectorX(const std::array<T, N> &vec)
{
  return std::vector<T>(vec.begin(), vec.end());
}
template<std::size_t N>
inline vectorXd to_vectorXd(const std::array<double, N> &vec)
{
  return vectorXd(vec.begin(), vec.end());
}

template<typename T, std::size_t N>
inline std::array<T, N> to_array(const std::vector<T> &vec)
{
  typename std::array<T, N> rv;
  //assert(vec.size() == rv.size());
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = vec[i]; }
  return rv;
}
template<std::size_t N>
inline std::array<double, N> to_arrayd(const vectorXd &vec)
{
  typename std::array<double, N> rv;
  //assert(vec.size() == rv.size());
  for (size_t i = 0; i < rv.size(); ++i) { rv[i] = vec[i]; }
  return rv;
}

inline std::string to_string(const vectorXd &vec)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}
template<std::size_t N>
inline std::string to_string(const std::array<double, N> &vec)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

}