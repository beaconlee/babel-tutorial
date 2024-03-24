#pragma once

#include <vector>
#include <Eigen/Eigen>

#include <iostream>

#define DEBUG false
#define DEBUG_LOG                                                              \
  if(DEBUG)                                                                    \
  {                                                                            \
    std::cout << __FILE__ << "  " << __FUNCTION__ << "   " << __LINE__         \
              << "\n";                                                         \
  }


namespace beacon
{
class CubicSpline
{
public:
  CubicSpline() = delete;
  CubicSpline(std::vector<double> x, std::vector<double> y);
  ~CubicSpline() = default;

  double CalcPosition(double x) const;
  double Calc1Derivative(double x) const;
  double Calc2Derivative(double x) const;
  double operator()(double x, int32_t d = 0) const;

private:
  bool CheckX(double x) const;
  Eigen::MatrixXd CalcA();
  Eigen::MatrixXd CalcB();
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;


  std::vector<double> x_coord_;
  std::vector<double> y_coord_;

  size_t size_;
};



class CubicSpline2D
{
public:
  CubicSpline2D() = delete;
  CubicSpline2D(std::vector<double> x, std::vector<double> y);
  ~CubicSpline2D() = default;

  Eigen::Vector2d CalcPosition(double s) const;
  double CalcYaw(double s) const;
  double CalcCurvatur(double s) const;
  Eigen::Vector2d operator()(double s, int32_t n = 0) const;

  static std::vector<std::vector<double>> CalcSplineCourse(
      std::vector<double> x, std::vector<double> y, double ds = 0.1);

  


private:
  std::vector<double> CalcS(std::vector<double> x, std::vector<double> y);
  std::vector<double> s_;
  CubicSpline xs_;
  CubicSpline ys_;
};
} // namespace beacon