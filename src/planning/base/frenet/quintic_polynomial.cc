#include "quintic_polynomial.h"

namespace beacon
{
QuinticPolynomial::QuinticPolynomial(double start,
                                     double startd,
                                     double startdd,
                                     double end,
                                     double endd,
                                     double enddd,
                                     double time) // time = Tend - Tstart
{
  /*
    我们另 t0 = 0 来简化这个一元五次方程的求解
    dt = a0_ + a1_*t + a2_*t^2 + a3_*t^3 + a4_*t^4 + a5_*t^5
    dt' = a1_ + 2*a2_*t + ...
    dt'' = 2*a2_ + 6a2_*t + ...
  */
  a0_ = start;
  a1_ = startd;
  a2_ = startdd;

  /*
      令 T = t1 - t0，剩余的三个系数 a3_，a4_，a5_ 可通过解如下方程得到：
      将后三个方程等式中含有 a0_ a1_ a2_ 替换成 xs vxs axs，并移项到等式右边
      然后就只剩下 a3_ a4_ a5_  然后求解方程

      d(time)   = a0_ + time*a1_ + time^2*a2_ + time^3   *a3_   + time^4    *a4_  + time^5    *a5_
      d(time)'  =       a1_      + 2*time*a2_ + 3*time^2 *a3_   + 4*time^3  *a4_  + 5*time^4  *a5_
      d(time)'' =                  2*a2_      + 6*time   *a3_   + 12*time^2 *a4_  + 20*time^3 *a5_
  */
  Eigen::MatrixXd a;
  a << pow(time, 3), pow(time, 4), pow(time, 5), 3 * pow(time, 2),
      4 * pow(time, 3), 5 * pow(time, 4), 6 * time, 12 * pow(time, 2),
      20 * pow(time, 3);

  Eigen::VectorXd b;
  b << end - a0_ - time * a1_ - pow(time, 2) * a2_, endd - a1_ - 2 * time * a2_,
      enddd - 2 * a2_;

  Eigen::Vector3d ret = a.colPivHouseholderQr().solve(b);
  a3_ = ret[0];
  a4_ = ret[1];
  a5_ = ret[2];
}

double QuinticPolynomial::CalcPoint(double time) const
{
  return a0_ + a1_ * time + a2_ * pow(time, 2) + a3_ * pow(time, 3) +
         a4_ * pow(time, 4) + a5_ * pow(time, 5);
}

double QuinticPolynomial::Calc1Derivative(double time) const
{
  return a1_ + 2 * a2_ * time + 3 * a3_ * pow(time, 2) +
         4 * a4_ * pow(time, 3) + 5 * a5_ * pow(time, 4);
}

double QuinticPolynomial::Calc2Derivative(double time) const
{
  return 2 * a2_ + 6 * a3_ * time + 12 * a4_ * pow(time, 2) +
         20 * a5_ * pow(time, 3);
}

double QuinticPolynomial::Calc3Derivative(double time) const
{
  return 6 * time + 24 * a4_ * time + 60 * a5_ * pow(time, 2);
}

} // namespace beacon