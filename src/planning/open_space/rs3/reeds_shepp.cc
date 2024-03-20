#include "reeds_shepp.h"
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;
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
  // u 是极径， t 是 theta
  Polar(x - sin(phi), y - 1. + cos(phi), u, t);
  if(t >= -ZERO)
  {
    v = Mod2Pi(phi - t);
    if(v >= -ZERO)
    {
      assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
      assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(Mod2Pi(t + v - phi)) < RS_EPS);
      return true;
    }
  }
  return false;
}

// formula 8.2
inline bool
LpSpRp(double x, double y, double phi, double& t, double& u, double& v)
{
  double t1, u1;
  Polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
  u1 = u1 * u1;
  if(u1 >= 4.)
  {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = Mod2Pi(t1 + theta);
    v = Mod2Pi(t - phi);
    assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(Mod2Pi(t - v - phi)) < RS_EPS);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}
Vector2d polar(double x, double y)
{
  double r = hypot(x, y);
  double theta = atan2(y, x);

  return {r, theta};
}

// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool
LpRmL(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
  Polar(xi, eta, u1, theta);
  if(u1 <= 4.)
  {
    u = -2. * asin(.25 * u1);
    t = Mod2Pi(theta + .5 * u + pi);
    v = Mod2Pi(phi - t + u);
    assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(Mod2Pi(t - u + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}


void SetPath(std::vector<beacon::ReedsSheppPath>& paths,
             double t,
             double u,
             double v,
             std::vector<char> ctypes,
             double step_size)
{
  beacon::ReedsSheppPath path = {
      std::vector<double>{t, u, v},
      ctypes,
      double(std::abs(t) + std::abs(u) + std::abs(v))};

  for(auto i_path : paths)
  {
    bool type_is_same = (i_path.char_ == path.char_);
    bool length_is_close = ((i_path.l_ - path.l_) <= step_size);
    if(type_is_same && length_is_close)
    {
      return;
    }
  }

  if(path.l_ <= step_size)
  {
    return;
  }

  paths.emplace_back(path);
}


void StraightCurveStraight(double x,
                           double y,
                           double phi,
                           std::vector<beacon::ReedsSheppPath>& paths,
                           double step_size)
{
  double t, u, v;
  if(SpLpSp(x, y, phi, t, u, v))
  {
    SetPath(paths, t, u, v, {'S', 'L', 'S'}, step_size);
  }
  if(SpLpSp(x, -y, -phi, t, u, v)) // timeflip
  {
    SetPath(paths, -t, -u, -v, {'S', 'R', 'S'}, step_size);
  }
}


void CurveStraightCurve(double x,
                        double y,
                        double phi,
                        std::vector<beacon::ReedsSheppPath>& paths,
                        double step_size)
{
  double t, u, v;
  if(LpSpLp(x, y, phi, t, u, v))
  {
    SetPath(paths, t, u, v, {'L', 'S', 'L'}, step_size);
  }
  if(LpSpLp(-x, y, -phi, t, u, v)) // timeflip
  {
    SetPath(paths, -t, -u, -v, {'L', 'S', 'L'}, step_size);
  }
  if(LpSpLp(x, -y, -phi, t, u, v)) // reflect
  {
    SetPath(paths, t, u, v, {'R', 'S', 'R'}, step_size);
  }
  if(LpSpLp(-x, -y, phi, t, u, v)) // timeflip + reflect
  {
    SetPath(paths, -t, -u, -v, {'R', 'S', 'R'}, step_size);
  }
  if(LpSpRp(x, y, phi, t, u, v))
  {
    SetPath(paths, t, u, v, {'L', 'S', 'R'}, step_size);
  }
  if(LpSpRp(-x, y, -phi, t, u, v)) // timeflip
  {
    SetPath(paths, -t, -u, -v, {'L', 'S', 'R'}, step_size);
  }
  if(LpSpRp(x, -y, -phi, t, u, v)) // reflect
  {
    SetPath(paths, t, u, v, {'R', 'S', 'L'}, step_size);
  }
  if(LpSpRp(-x, -y, phi, t, u, v)) // timeflip + reflect
    SetPath(paths, -t, -u, -v, {'R', 'S', 'L'}, step_size);
}


void set_path(vector<beacon::ReedsSheppPath>& paths,
              Eigen::Vector3d lengths,
              vector<char> ctypes,
              double step_size)
{
  beacon::ReedsSheppPath path;
  path.char_ = ctypes;
  path.lengths_ = {lengths[0], lengths[1], lengths[2]};
  path.l_ = abs(lengths[0]) + abs(lengths[1]) + abs(lengths[2]);

  for(beacon::ReedsSheppPath i_path : paths)
  {
    bool type_is_same = (i_path.char_ == path.char_);
    bool length_is_close = ((i_path.l_ - path.l_) <= step_size);
    if(type_is_same && length_is_close)
    {
      return;
    }
  }

  if(path.l_ <= step_size)
  {
    return;
  }

  paths.push_back(path);
}


void CurveCurveCurve(double x,
                     double y,
                     double phi,
                     std::vector<beacon::ReedsSheppPath>& paths,
                     double step_size)
{
  double t, u, v;
  if(LpRmL(x, y, phi, t, u, v))
  {
    SetPath(paths, t, u, v, {'L', 'R', 'L'}, step_size);
  }

  if(LpRmL(-x, y, -phi, t, u, v)) // timeflip
  {
    SetPath(paths, -t, -u, -v, {'L', 'R', 'L'}, step_size);
  }

  if(LpRmL(x, -y, -phi, t, u, v)) // reflect
  {
    SetPath(paths, t, u, v, {'R', 'L', 'R'}, step_size);
  }

  if(LpRmL(-x, -y, phi, t, u, v)) // timeflip + reflect
  {
    SetPath(paths, -t, -u, -v, {'R', 'L', 'R'}, step_size);
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);
  if(LpRmL(xb, yb, phi, t, u, v))
  {
    SetPath(paths, v, u, t, {'L', 'R', 'L'}, step_size);
  }

  if(LpRmL(-xb, yb, -phi, t, u, v)) // timeflip
  {
    SetPath(paths, -v, -u, -t, {'L', 'R', 'L'}, step_size);
  }

  if(LpRmL(xb, -yb, -phi, t, u, v)) // reflect
  {
    SetPath(paths, v, u, t, {'R', 'L', 'R'}, step_size);
  }

  if(LpRmL(-xb, -yb, phi, t, u, v)) // timeflip + reflect
  {
    SetPath(paths, -v, -u, -t, {'R', 'L', 'R'}, step_size);
  }
}



} // namespace

namespace beacon
{



ReedsSheppPath
ReedsShepp::AnalysticExpantion(std::shared_ptr<TrajectoryNode> start,
                               std::shared_ptr<TrajectoryNode> goal,
                               std::shared_ptr<Frame>& frame,
                               double step_size)
{
  Eigen::Vector3d v3d_start(
      start->x_list_.back(), start->y_list_.back(), start->yaw_list_.back());
  Eigen::Vector3d v3d_goal(
      goal->x_list_.back(), goal->y_list_.back(), goal->yaw_list_.back());

  double maxc = std::tan(frame->vc.max_steer_) / frame->vc.whell_base_;
  DEBUG_LOG

  auto paths = CalcRSPaths(v3d_start, v3d_goal, maxc, step_size);
  DEBUG_LOG

  if(paths.empty())
  {
    return {};
  }
  DEBUG_LOG
  LOG(paths.size())

  std::sort(
      std::begin(paths),
      std::end(paths),
      [&frame, this](const ReedsSheppPath& path_a, const ReedsSheppPath& path_b)
      {
        DEBUG_LOG

        return CalcRspathCost(path_a, frame) < CalcRspathCost(path_b, frame);
      });
  DEBUG_LOG

  for(auto path : paths)
  {
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
    for(size_t idx = 0; idx < path.x_vec_.size(); idx += COLLISION_CHECK_STEP)
    {
      path_x.push_back(path.x_vec_[idx]);
      path_y.push_back(path.y_vec_[idx]);
      path_yaw.push_back(path.yaw_vec_[idx]);
    }
    if(!IsCollision(path_x, path_y, path_yaw, frame))
    {
      return path;
    }
  }

  return {};
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


Vector2d polar(double x, double y)
{
  double r = hypot(x, y);
  double theta = atan2(y, x);

  return {r, theta};
}

Vector3d straight_left_straight(double x, double y, double phi, bool& flag)
{
  DEBUG_LOG
  flag = false;
  phi = Mod2Pi(phi);
  DEBUG_LOG
  if(M_PI * 0.01 < phi && phi < M_PI * 0.99 && y != 0)
  {
    double xd = -y / tan(phi) + x;
    double t = xd - tan(phi / 2.0);
    double u = phi;
    double v = Sign(y) * hypot(x - xd, y) - tan(phi / 2.0);
    flag = true;
    return {t, u, v};
  }
  DEBUG_LOG

  return {0.0, 0.0, 0.0};
}

Vector3d left_straight_left(double x, double y, double phi, bool& flag)
{
  flag = false;
  //   return {r, theta};
  Vector2d ut = polar(x - sin(phi), y - 1.0 + cos(phi));
  if(ut[1] >= 0.0)
  {
    double v = Mod2Pi(phi - ut[1]);
    if(v >= 0.0)
    {
      flag = true;
      return {ut[1], ut[0], v};
    }
  }

  return {0.0, 0.0, 0.0};
}

Vector3d left_straight_right(double x, double y, double phi, bool& flag)
{
  flag = false;
  Vector2d ut1 = polar(x + sin(phi), y - 1.0 - cos(phi));
  double u1 = ut1[0] * ut1[0];
  if(u1 >= 4.0)
  {
    double u = sqrt(u1 - 4.0);
    double theta = atan2(2.0, u);
    double t = Mod2Pi(ut1[1] + theta);
    double v = Mod2Pi(t - phi);

    if(t >= 0.0 && v >= 0.0)
    {
      flag = true;
      return {t, u, v};
    }
  }

  return {0.0, 0.0, 0.0};
}

Vector3d left_right_left(double x, double y, double phi, bool& flag)
{
  flag = false;
  // double r = hypot(x, y);
  // double theta = atan2(y, x);

  // return {r, theta};
  Vector2d ut1 = polar(x - sin(phi), y - 1.0 + cos(phi));
  // ut1[0] r     t
  // ut1[1] theta v

  if(ut1[0] <= 4.0)
  {
    double u = -2.0 * asin(0.25 * ut1[0]);
    double t = Mod2Pi(ut1[1] + 0.5 * u + M_PI);
    double v = Mod2Pi(phi - t + u);

    if(t >= 0.0 && 0.0 >= u)
    {
      flag = true;
      return {t, u, v};
    }
  }

  return {0.0, 0.0, 0.0};
}


void straight_curve_straight(double x,
                             double y,
                             double phi,
                             vector<ReedsSheppPath>& paths,
                             double step_size)
{
  bool flag = false;
  DEBUG_LOG
  DEBUG_LOG
  Eigen::Vector3d tuv = straight_left_straight(x, y, phi, flag);
  DEBUG_LOG
  if(flag)
  {
    set_path(paths, tuv, {'S', 'L', 'S'}, step_size);
  }

  tuv = straight_left_straight(x, -y, -phi, flag);
  if(flag)
  {
    set_path(paths, tuv, {'S', 'R', 'S'}, step_size);
  }
  DEBUG_LOG
}

void curve_straight_curve(double x,
                          double y,
                          double phi,
                          vector<ReedsSheppPath>& paths,
                          double step_size)
{
  DEBUG_LOG
  bool flag = false;
  Eigen::Vector3d tuv = left_straight_left(x, y, phi, flag);
  if(flag)
  {
    set_path(paths, tuv, {'L', 'S', 'L'}, step_size);
  }

  tuv = left_straight_left(-x, y, -phi, flag);
  if(flag)
  {
    set_path(paths, -1 * tuv, {'L', 'S', 'L'}, step_size);
  }

  tuv = left_straight_left(x, -y, -phi, flag);
  if(flag)
  {
    set_path(paths, tuv, {'R', 'S', 'R'}, step_size);
  }
  tuv = left_straight_left(-x, -y, phi, flag);
  if(flag)
  {
    set_path(paths, -1 * tuv, {'R', 'S', 'R'}, step_size);
  }

  tuv = left_straight_right(x, y, phi, flag);
  if(flag)
  {
    set_path(paths, tuv, {'L', 'S', 'R'}, step_size);
  }

  tuv = left_straight_right(-x, y, -phi, flag);
  if(flag)
  {
    set_path(paths, -1 * tuv, {'L', 'S', 'R'}, step_size);
  }

  tuv = left_straight_right(x, -y, -phi, flag);
  if(flag)
  {
    set_path(paths, tuv, {'R', 'S', 'L'}, step_size);
  }

  tuv = left_straight_right(-x, -y, phi, flag);
  if(flag)
  {
    set_path(paths, -1 * tuv, {'R', 'S', 'L'}, step_size);
  }
}

void curve_curve_curve(double x,
                       double y,
                       double phi,
                       vector<ReedsSheppPath>& paths,
                       double step_size)
{
  DEBUG_LOG
  bool flag = false;
  Eigen::Vector3d tuv = left_right_left(x, y, phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, tuv, {'L', 'R', 'L'}, step_size);
  }

  tuv = left_right_left(-x, y, -phi, flag);
  if(flag)
  {
    DEBUG_LOG
    set_path(paths, -1 * tuv, {'L', 'R', 'L'}, step_size);
  }

  tuv = left_right_left(x, -y, -phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, tuv, {'R', 'L', 'R'}, step_size);
  }

  tuv = left_right_left(-x, -y, phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, -1 * tuv, {'R', 'L', 'R'}, step_size);
  }

  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);

  tuv = left_right_left(xb, yb, phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, {tuv[2], tuv[1], tuv[0]}, {'L', 'R', 'L'}, step_size);
  }

  tuv = left_right_left(-xb, yb, -phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, {-tuv[2], -tuv[1], -tuv[0]}, {'L', 'R', 'L'}, step_size);
  }

  tuv = left_right_left(xb, -yb, -phi, flag);
  if(flag)
  {
    DEBUG_LOG
    set_path(paths, {tuv[2], tuv[1], tuv[0]}, {'R', 'L', 'R'}, step_size);
  }

  tuv = left_right_left(-xb, -yb, phi, flag);
  if(flag)
  {
    DEBUG_LOG
    ///////////////
    set_path(paths, {-tuv[2], -tuv[1], -tuv[0]}, {'R', 'L', 'R'}, step_size);
  }
}

vector<ReedsSheppPath> generate_path(Eigen::Vector3d start,
                                     Eigen::Vector3d goal,
                                     double max_curvature,
                                     double step_size)
{
  double dx = goal.x() - start.x();
  double dy = goal.y() - start.y();
  double dth = goal.z() - start.z();
  double c = cos(start.z());
  double s = sin(start.z());
  double x = (c * dx + s * dy) * max_curvature;
  double y = (-s * dx + c * dy) * max_curvature;

  std::vector<ReedsSheppPath> paths;
  straight_curve_straight(x, y, dth, paths, step_size);
  curve_straight_curve(x, y, dth, paths, step_size);
  curve_curve_curve(x, y, dth, paths, step_size);

  return paths;
}


std::vector<ReedsSheppPath> ReedsShepp::GenReedsSheppPath(Eigen::Vector3d start,
                                                          Eigen::Vector3d goal,
                                                          double max_curvature,
                                                          double step_size)
{
  double dx = goal.x() - start.x();
  double dy = goal.y() - start.y();
  double dth = goal.z() - start.z();
  double c = cos(start.z());
  double s = sin(start.z());
  // max_curvature = 1 / r
  // 除以 max_curvatrue 进行归一化
  double x = (c * dx + s * dy) * max_curvature;
  double y = (-s * dx + c * dy) * max_curvature;

  std::vector<ReedsSheppPath> paths;

  StraightCurveStraight(x, y, dth, paths, step_size);
  CurveStraightCurve(x, y, dth, paths, step_size);
  CurveCurveCurve(x, y, dth, paths, step_size);
  return paths;
}

double ReedsShepp::CalcRspathCost(ReedsSheppPath rspath,
                                  std::shared_ptr<Frame>& frame)
{
  double cost = 0.0;
  DEBUG_LOG

  for(auto lr : rspath.lengths_)
  {
    if(lr >= 0)
    {
      cost += 1;
    }
    else
    {
      cost += std::abs(lr) * BACKWARD_COST;
    }
  }
  DEBUG_LOG

  for(size_t idx = 0; idx < rspath.lengths_.size() - 1; ++idx)
  {
    if(rspath.lengths_[idx] * rspath.lengths_[idx + 1] < 0.)
    {
      cost += GEAR_COST;
    }
  }
  DEBUG_LOG

  for(char ctype : rspath.char_)
  {
    if(ctype != 'S')
    {
      cost += STEER_ANGLE_COST * std::abs(frame->vc.max_steer_);
    }
  }
  DEBUG_LOG

  std::vector<double> ulist(rspath.char_.size(), 0);
  for(size_t idx = 0; idx < rspath.char_.size(); ++idx)
  {
    if(rspath.char_[idx] == 'R')
    {
      ulist[idx] = -frame->vc.max_steer_;
    }
    else if(rspath.char_[idx] == 'L')
    {
      ulist[idx] = frame->vc.max_steer_;
    }
  }
  DEBUG_LOG

  for(size_t idx = 0; idx < rspath.char_.size(); ++idx)
  {
    cost += (STEER_CHANGE_COST * std::abs(ulist[idx + 1] - ulist[idx]));
  }

  return cost;
}


bool ReedsShepp::IsCollision(std::vector<double>& x,
                             std::vector<double>& y,
                             std::vector<double>& yaw,
                             std::shared_ptr<Frame>& frame)
{
  for(size_t idx = 0; idx < x.size(); ++idx)
  {
    int d = 1;
    double dl = (frame->vc.rf_ - frame->vc.rb_) / 2.0;
    double r = (frame->vc.rf_ + frame->vc.rb_) / 2.0 + d;
    double cx = x[idx] + dl * cos(yaw[idx]);
    double cy = y[idx] + dl * sin(yaw[idx]);
    std::vector<point_t> ids = frame->obs->NeighborhoodPoints({cx, cy}, r);

    for(const point_t& ob : ids)
    {
      double xo = ob[0] - cx;
      double yo = ob[1] - cy;
      double dx = xo * cos(yaw[idx]) + yo * sin(yaw[idx]);
      double dy = -xo * sin(yaw[idx]) + yo * cos(yaw[idx]);

      if(abs(dx) < r && abs(dy) < frame->vc.width_ / 2 + d)
      {
        return true;
      }
    }
  }

  return false;
}

std::vector<std::vector<double>>
ReedsShepp::SolveRSPath(std::vector<double> lengths,
                        std::vector<char> modes,
                        double max_curvature,
                        double step_size)
{
  std::vector<std::vector<double>> interpolate_dists_list =
      CalcInterpolateDistsList(lengths, step_size);

  Eigen::Vector3d origin(0, 0, 0);
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> yaws;
  std::vector<double> directions;

  for(size_t idx = 0; idx < lengths.size(); ++idx)
  {
    for(double dist : interpolate_dists_list[idx])
    {
      Eigen::Vector4d state =
          Interpolate(dist, lengths[idx], modes[idx], max_curvature, origin);
      xs.push_back(state[0]);
      ys.push_back(state[1]);
      yaws.push_back(state[2]);
      directions.push_back(state[3]);
    }
    origin[0] = xs.back();
    origin[1] = ys.back();
    origin[2] = yaws.back();
  }

  return {xs, ys, yaws, directions};
}



std::vector<std::vector<double>>
ReedsShepp::CalcInterpolateDistsList(std::vector<double> lengths,
                                     double step_size)
{
  std::vector<std::vector<double>> interpolate_dists_list;
  for(double length : lengths)
  {
    std::vector<double> interp_dists;
    int len_sign = Sign(length);
    for(double d = 0; d < abs(length); d += step_size)
    {
      interp_dists.push_back(len_sign * d);
    }
    interp_dists.push_back(length);
    interpolate_dists_list.emplace_back(interp_dists);
  }

  return interpolate_dists_list;
}


Eigen::Vector4d ReedsShepp::Interpolate(double dist,
                                        double length,
                                        char mode,
                                        double max_curvature,
                                        Eigen::Vector3d origin)
{
  Eigen::Vector4d inter(0, 0, 0, 0);
  if(mode == 'S')
  {
    inter[0] = origin[0] + dist / max_curvature * cos(origin[2]);
    inter[1] = origin[1] + dist / max_curvature * sin(origin[2]);
    inter[2] = origin[2];
  }
  else
  {
    double ldx = sin(dist) / max_curvature;
    double ldy = 0.0;
    if(mode == 'L')
    {
      ldy = (1.0 - cos(dist)) / max_curvature;
      inter[2] = origin[2] + dist;
    }
    else if(mode == 'R')
    {
      ldy = (1.0 - cos(dist)) / -max_curvature;
      inter[2] = origin[2] - dist;
    }
    double gdx = cos(-origin[2]) * ldx + sin(-origin[2]) * ldy;
    double gdy = -sin(-origin[2]) * ldx + cos(-origin[2]) * ldy;
    inter[0] = origin[0] + gdx;
    inter[1] = origin[1] + gdy;
  }
  inter[3] = length > 0.0 ? 1 : -1;

  return inter;
}


ReedsSheppPath reeds_shepp_path(Eigen::Vector3d s,
                                Eigen::Vector3d g,
                                double maxc,
                                double step_size)
{
  ReedsShepp rs;
  DEBUG_LOG
  auto paths = rs.CalcRSPaths(s, g, maxc, step_size);
  int best_path_index = -1;
  DEBUG_LOG

  for(size_t idx = 0; idx < paths.size(); ++idx)
  {
    if(best_path_index == -1 ||
       abs(paths[best_path_index].l_) > abs(paths[idx].l_))
    {
      best_path_index = idx;
    }
  }
  DEBUG_LOG

  if(best_path_index == -1)
  {
    DEBUG_LOG
    return ReedsSheppPath();
  }
  DEBUG_LOG

  return paths[best_path_index];
}

std::vector<ReedsSheppPath> ReedsShepp::CalcRSPaths(Eigen::Vector3d start,
                                                    Eigen::Vector3d goal,
                                                    double max_curvature,
                                                    double step_size)
{
  DEBUG_LOG
  // generate_path
  // GenReedsSheppPath
  // 最大曲率 max curvature
  std::vector<ReedsSheppPath> paths =
      GenReedsSheppPath(start, goal, max_curvature, step_size);
  DEBUG_LOG

  for(ReedsSheppPath& path : paths)
  {
    std::vector<std::vector<double>> status =
        SolveRSPath(path.lengths_, path.char_, max_curvature, step_size);
    DEBUG_LOG

    for(size_t idx = 0; idx < status[0].size(); ++idx)
    {
      double ix = status[0][idx];
      double iy = status[1][idx];
      double yaw = status[2][idx];

      int direction = static_cast<int>(status[3][idx]);

      path.x_vec_.emplace_back(std::cos(-start[2]) * ix +
                               std::sin(-start[2]) * iy + start[0]);
      path.y_vec_.emplace_back(-std::sin(-start[2]) * ix +
                               std::cos(-start[2]) * iy + start[1]);
      path.yaw_vec_.emplace_back(Mod2Pi(yaw + start[2]));
      path.direc_vec_.emplace_back(direction);
    }

    for(size_t idx = 0; idx < path.lengths_.size(); ++idx)
    {
      path.lengths_[idx] /= max_curvature;
    }
    path.l_ /= max_curvature;
  }
  DEBUG_LOG

  return paths;
}


} // namespace beacon