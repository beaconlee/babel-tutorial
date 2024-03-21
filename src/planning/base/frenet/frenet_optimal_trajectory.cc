#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

#include <Eigen/Core>
#include <fmt/core.h>

#include "src/planning/base/frenet/cubic_spline.h"
#include "cubic_spline.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

using std::vector;
using namespace Eigen;

namespace beacon
{

// Parameter
constexpr double MAX_SPEED = 50.0 / 3.6;   //maximum speed [m/s]
constexpr double MAX_ACCEL = 2.0;          //maximum acceleration [m/ss]
constexpr double MAX_CURVATURE = 1.0;      //maximum curvature [1/m]
constexpr double MAX_ROAD_WIDTH = 7.0;     //maximum road width [m]
constexpr double D_ROAD_W = 1.0;           //road width sampling length [m]
constexpr double DT = 0.2;                 //time tick [s]
constexpr double MAX_T = 5.0;              //max prediction time [m]
constexpr double MIN_T = 4.0;              //min prediction time [m]
constexpr double TARGET_SPEED = 30. / 3.6; //target speed [m/s]
constexpr double D_T_S = 5.0 / 3.6;        //target speed sampling length [m/s]
constexpr double N_S_SAMPLE = 1;           //sampling number of target speed
constexpr double ROBOT_RADIUS = 2.0;       //robot radius [m]
// cost weights
constexpr double K_J = 0.1;
constexpr double K_T = 0.1;
constexpr double K_D = 1.0;
constexpr double K_LAT = 1.0;
constexpr double K_LON = 1.0;


class FrenetPath
{
public:
  vector<double> t;
  vector<double> d;
  vector<double> d_d;
  vector<double> d_dd;
  vector<double> d_ddd;
  vector<double> s;
  vector<double> s_d;
  vector<double> s_dd;
  vector<double> s_ddd;
  double cd = 0.0;
  double cv = 0.0;
  double cf = 0.0;

  vector<double> x;
  vector<double> y;
  vector<double> yaw;
  vector<double> ds;
  vector<double> c;

  double max_speed = 0.0;
  double max_accel = 0.0;
  double max_curvature = 0.0;

  FrenetPath() {}
  ~FrenetPath() {}
};

vector<FrenetPath> calc_frenet_paths(double c_speed,
                                     double c_accel,
                                     double c_d,
                                     double c_d_d,
                                     double c_d_dd,
                                     double s0)
{
  vector<FrenetPath> frenet_paths;

  for(double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W)
  {
    for(double Ti = MIN_T; Ti < MAX_T; Ti += DT)
    {
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);

      for(double t = 0.; t < Ti; t += DT)
      {
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }

      for(double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE;
          tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
          tv += D_T_S)
      {
        FrenetPath tfp = fp;
        QuarticPolynomial lon_qp(s0, c_speed, c_accel, tv, 0.0, Ti);

        for(double t : fp.t)
        {
          tfp.s.push_back(lon_qp.calc_point(t));
          tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
          tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
          tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
          if(tfp.s_d.back() > tfp.max_speed)
          {
            tfp.max_speed = tfp.s_d.back();
          }
          if(tfp.s_dd.back() > tfp.max_accel)
          {
            tfp.max_accel = tfp.s_dd.back();
          }
        }
        std::transform(tfp.d_ddd.begin(),
                       tfp.d_ddd.end(),
                       tfp.d_ddd.begin(),
                       [](int x) { return x * x; });
        std::transform(tfp.s_ddd.begin(),
                       tfp.s_ddd.end(),
                       tfp.s_ddd.begin(),
                       [](int x) { return x * x; });
        double Jp = std::accumulate(tfp.d_ddd.begin(), tfp.d_ddd.end(), 0.0);
        double Js = std::accumulate(tfp.s_ddd.begin(), tfp.s_ddd.end(), 0.0);

        double ds = pow(TARGET_SPEED - tfp.s_d.back(), 2);
        tfp.cd = K_J * Jp + K_T * Ti + K_D * pow(tfp.d.back(), 2);
        tfp.cv = K_J * Js + K_T * Ti + K_D * ds;
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv;

        frenet_paths.push_back(tfp);
      }
    }
  }

  return frenet_paths;
}

void calc_global_paths(vector<FrenetPath>& fplist, CubicSpline2D& csp)
{
  for(FrenetPath& fp : fplist)
  {
    for(size_t idx = 0; idx < fp.s.size(); ++idx)
    {
      if(fp.s[idx] > csp.s.back())
      {
        break;
      }

      Vector2d ixy = csp.calc_position(fp.s[idx]);
      double i_yaw = csp.calc_yaw(fp.s[idx]);
      double di = fp.d[idx];
      double fx = ixy[0] + di * cos(i_yaw + M_PI_2);
      double fy = ixy[1] + di * sin(i_yaw + M_PI_2);
      fp.x.emplace_back(fx);
      fp.y.emplace_back(fy);
    }

    for(size_t idx = 0; idx + 1 < fp.x.size(); ++idx)
    {
      double dx = fp.x[idx + 1] - fp.x[idx];
      double dy = fp.y[idx + 1] - fp.y[idx];
      fp.yaw.emplace_back(atan2(dy, dx));
      fp.ds.emplace_back(hypot(dx, dy));
    }
    if(!fp.yaw.empty())
    {
      fp.yaw.push_back(fp.yaw.back());
      fp.ds.push_back(fp.ds.back());

      for(size_t idx = 0; idx < fp.yaw.size() - 1; ++idx)
      {
        fp.c.emplace_back((fp.yaw[idx + 1] - fp.yaw[idx]) / fp.ds[idx]);
        if(fp.c.back() > fp.max_curvature)
        {
          fp.max_curvature = fp.c.back();
        }
      }
    }
  }
}

bool check_collision(const FrenetPath& fp, const vector<vector<double>>& obs)
{
  for(size_t i = 0; i < obs[0].size(); ++i)
  {
    for(size_t j = 0; j < fp.x.size(); ++j)
    {
      double dist = hypot(fp.x[j] - obs[0][i], fp.y[j] - obs[1][i]);
      if(dist <= ROBOT_RADIUS)
      {
        return false;
      }
    }
  }

  return true;
}

vector<FrenetPath> check_paths(vector<FrenetPath>& fplist,
                               const vector<vector<double>>& obs)
{
  vector<FrenetPath> final_fp;

  for(size_t idx = 0; idx < fplist.size(); ++idx)
  {
    if(fplist[idx].max_speed < MAX_SPEED && fplist[idx].max_accel < MAX_ACCEL &&
       fplist[idx].max_curvature < MAX_CURVATURE &&
       check_collision(fplist[idx], obs))
    {
      final_fp.push_back(fplist[idx]);
    }
  }

  return final_fp;
}

FrenetPath frenet_optimal_planning(CubicSpline2D& csp,
                                   double s0,
                                   double c_speed,
                                   double c_accel,
                                   double c_d,
                                   double c_d_d,
                                   double c_d_dd,
                                   const vector<vector<double>>& obs)
{
  vector<FrenetPath> fplist =
      calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fplist, csp);
  vector<FrenetPath> final_paths = check_paths(fplist, obs);

  double min_cost = std::numeric_limits<double>::max();
  FrenetPath best_path;
  for(const FrenetPath& fp : final_paths)
  {
    if(min_cost >= fp.cf)
    {
      min_cost = fp.cf;
      best_path = fp;
    }
  }

  return best_path;
}

} // namespace beacon
