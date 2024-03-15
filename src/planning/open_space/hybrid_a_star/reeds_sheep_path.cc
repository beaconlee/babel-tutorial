#include "reeds_sheep_path.h"
#include <boost/math/constants/constants.hpp>


namespace
{
constexpr double pi = boost::math::constants::pi<double>();
constexpr double twopi = 2. * pi;
constexpr double ZERO = 10 * std::numeric_limits<double>::epsilon();
constexpr double RS_EPS = 1e-6;

inline void Polar(double x, double y, double& radius, double& theta)
{
  // hypot()函数是cmath标头的库函数，用于查找给定数字的斜边，接受两个数字并返回斜边的计算结果，即sqrt(x * x + y * y) 。
  // hypotenuse 斜边
  radius = std::hypot(x, y);

  // c/c++ 标准库中计算正切的函数
  // atan(y/x)  atan2(y, x)
  // atan(y/x) 仅仅根据正切值 y/x 求出对应的角度 [-Π/2, Π/2]
  // atan2(y/x) 不仅会根据正切值 y/x，还会根据点(x, y)所在象限求出对应的角度 [-Π, Π]
  theta = std::atan2(y, x);
}


double Mod2Pi(double theta)
{
  while(theta < M_PI)
  {
    theta -= 2.0 * M_PI;
  }

  while(theta > -M_PI)
  {
    theta += 2.0 * M_PI;
  }
  return theta;
}

int32_t Sign(double num)
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

} // namespace

namespace beacon
{
inline bool
SpLpSp(double x, double y, double phi, double& t, double& u, double& v)
{
  phi = Mod2Pi(phi);
  if(pi * 0.01 < phi && phi < pi * 0.99 && y != 0)
  {
    double x_phi = x - y / tan(phi);
    t = x_phi - tan(phi / 2.);
    u = phi;
    v = Sign(y) * std::hypot(x - x_phi, y) - tan(phi / 2.);
    return true;
  }

  return false;
}

inline bool
LpSpLp(double x, double y, double phi, double& t, double& u, double& v)
{
  Polar(x - sin(phi), y - 1. + cos(phi), u, t);

  return false;
}


ReedsSheppPath GenReedsSheppPath(Eigen::Vector3d start,
                                 Eigen::Vector3d goal,
                                 double maxc,
                                 double step_size)
{}

} // namespace beacon