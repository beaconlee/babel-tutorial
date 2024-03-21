#pragma once

#include <vector>
#include <Eigen/Eigen>

class CubicSpline
{
private:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
  std::vector<double> h;
  int nx;

public:
  CubicSpline() {}
  CubicSpline(std::vector<double> _x, std::vector<double> _y);
  ~CubicSpline() {}
  Eigen::MatrixXd calc_A(void);
  Eigen::VectorXd calc_B(void);
  double calc_position(double _x) const;
  double calc_first_derivative(double _x) const;
  double calc_second_derivative(double _x) const;
  double operator()(double _x, int dd = 0) const;
};

class CubicSpline2D
{
private:
  CubicSpline sx;
  CubicSpline sy;

public:
  std::vector<double> s;

  CubicSpline2D() {}
  CubicSpline2D(std::vector<double> _x, std::vector<double> _y);
  ~CubicSpline2D() {}
  std::vector<double> calc_s(std::vector<double> _x, std::vector<double> _y);
  Eigen::Vector2d calc_position(double _s) const;
  double calc_yaw(double _s) const;
  double calc_curvature(double _s) const;
  Eigen::Vector2d operator()(double _s, int n = 0) const;

  static std::vector<std::vector<double>> calc_spline_course(
      std::vector<double> x, std::vector<double> y, double ds = 0.1);
};
