#pragma once


#include <vector>
#include <cassert>
#include <cstdint>

# define M_PI		3.14159265358979323846	/* pi */

namespace beacon
{

// constexpr double M_PI = 3.14159265358979323846; /* pi */

template <typename T>
T Min(std::vector<T> vec)
{
  std::size_t size = vec.size();
  assert(size > 0);

  auto ret = vec.at(0);
  for(auto item : vec)
  {
    if(item < ret)
    {
      ret = item;
    }
  }

  return ret;
}

template <typename T>
T Max(std::vector<T> vec)
{
  std::size_t size = vec.size();
  assert(size > 0);

  auto ret = vec.at(0);
  for(auto item : vec)
  {
    if(item > ret)
    {
      ret = item;
    }
  }

  return ret;
}

template <typename T>
int32_t Sign(T num)
{
  if(num < 0)
  {
    return -1;
  }
  if(num > 0)
  {
    return 1;
  }
  return 0;
}

template <typename T>
double Mod2Pi(T theta)
{
  while(theta > M_PI)
  {
    theta -= 2.0 * M_PI;
  }

  while(theta < -M_PI)
  {
    theta += 2.0 * M_PI;
  }
  return theta;
}

} // namespace beacon