#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cmath>
namespace beacon
{
// 五次多项式
class QuinticPolynomial
{
public:
  QuinticPolynomial() = default;
  QuinticPolynomial(double start,
                    double startd,
                    double startdd,
                    double end,
                    double endd,
                    double enddd,
                    double time);
  double CalcPoint(double time) const;
  double Calc1Derivative(double time) const;
  double Calc2Derivative(double time) const;
  double Calc3Derivative(double time) const;

private:
  double a0_{0.};
  double a1_{0.};
  double a2_{0.};
  double a3_{0.};
  double a4_{0.};
  double a5_{0.};
};
} // namespace beacon