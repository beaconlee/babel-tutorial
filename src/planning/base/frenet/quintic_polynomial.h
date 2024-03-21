///
// @file quintic_polynomial.h
// @author beacon (bquanlicn@gmail.com)
// @brief
// @version 0.1
// @date 2024-03-21
//
// @copyright Copyright (c) 2024
//
//

#pragma once


#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace beacon
{

// 五次多项式
/**
 * @brief 
 * 这段代码定义了一个 QuinticPolynomial 类的构造函数，用于生成一个五次多项式，描述了一个物体在起始位置、起始速度、起始加速度到达终点位置、终点速度、终点加速度的运动轨迹。具体来说：

- `QuinticPolynomial` 是一个类名，表示五次多项式。
- 参数包括起始位置 `xs`、起始速度 `vxs`、起始加速度 `axs`、终点位置 `xe`、终点速度 `vxe`、终点加速度 `axe` 和时间 `time`。
- 在构造函数中，首先将参数 `xs`、`vxs` 和 `axs` 赋值给类成员变量 `a0_`、`a1_` 和 `a2_`。
- 然后，构造一个 3x3 的矩阵 `A`，并用给定的时间 `time` 计算了矩阵的每个元素。
- 构造一个 3 维向量 `b`，其中包含了目标位置、速度和加速度与起始位置、速度和加速度的关系。
- 使用 `Eigen` 库中的 `colPivHouseholderQr().solve(b)` 方法解方程组 `Ax = b`，得到 `x` 向量，包含了五次多项式的系数。
- 最后将 `x` 向量中的值分别赋给类成员变量 `a3_`、`a4_` 和 `a5_`，这些值分别对应五次多项式的高阶系数。


`colPivHouseholderQr().solve(b)` 是对矩阵方程进行求解的操作。
具体来说，它使用了列主元素的 Householder QR 分解方法来解决形如 Ax=b 的线性方程组，其中 A 是一个方阵，b 是一个列向量。
Householder QR 分解是一种矩阵分解方法，用于将矩阵分解为一个正交矩阵和一个上三角矩阵的乘积。通过这种分解，可以有效地求解线性方程组。

在这段代码中，`A` 是一个 3x3 的矩阵，`b` 是一个 3 维向量，`colPivHouseholderQr().solve(b)` 的作用是求解方程组 `Ax=b`，并返回解向量 `x`，其中包含了五次多项式的系数。

 */
class QuinticPolynomial
{
public:
  QuinticPolynomial(double xs,   // 起始位置
                    double vxs,  // 起始位置速度
                    double axs,  // 起始位置加速度
                    double xe,   // 终点位置
                    double vxe,  // 终点位置速度
                    double axe,  // 终点位置加速度
                    double time) // 时间  time = t1 - t0
  {
    /*
      我们另 t0 = 0 来简化这个六元方程组的求解
      dt = a0_ + a1_*t + a2_*t^2 + a3_*t^3 + a4_*t^4 + a5_*t^5
      dt' = a1_ + 2*a2_*t + ...
      dt'' = 2*a2_ + 6a2_*t + ...


      令 T = t1 - t0，剩余的三个系数 a3_，a4_，a5_ 可通过解如下方程得到：

    */
    
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;

    /*
      将后三个方程等式中含有 a0_ a1_ a2_ 替换成 xs vxs axs，并移项到等式右边
      然后就只剩下 a3_ a4_ a5_  然后求解方程
    */

    Eigen::Matrix3d A;
    A << pow(time, 3), pow(time, 4), pow(time, 5), 3 * pow(time, 2),
        4 * pow(time, 3), 5 * pow(time, 4), 6 * time, 12 * pow(time, 2),
        20 * pow(time, 3);

    Eigen::Vector3d b;
    b << xe - a0_ - a1_ * time - a2_ * pow(time, 2),
         vxe - a1_ - 2 * a2_ * time,
         axe - 2 * a2_;

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    a3_ = x[0];
    a4_ = x[1];
    a5_ = x[2];
  }
  ~QuinticPolynomial() {}

  double calc_point(double t)
  {
    double xt = a0_ + a1_ * t + a2_ * pow(t, 2) + a3_ * pow(t, 3) +
                a4_ * pow(t, 4) + a5_ * pow(t, 5);
    return xt;
  }

  double calc_first_derivative(double t)
  {
    double xt = a1_ + 2 * a2_ * t + 3 * a3_ * pow(t, 2) + 4 * a4_ * pow(t, 3) +
                5 * a5_ * pow(t, 4);
    return xt;
  }

  double calc_second_derivative(double t)
  {
    double xt =
        2 * a2_ + 6 * a3_ * t + 12 * a4_ * pow(t, 2) + 20 * a5_ * pow(t, 3);
    return xt;
  }

  double calc_third_derivative(double t)
  {
    double xt = 6 * a3_ + 24 * a4_ * t + 60 * a5_ * pow(t, 2);
    return xt;
  }

private:
  double a0_;
  double a1_;
  double a2_;
  double a3_;
  double a4_;
  double a5_;
};
/*
这段代码定义了一个名为 `QuinticPolynomial` 的类，用于计算五次多项式的值及其导数。这个类包含以下成员变量和方法：

1. 成员变量：
   - `a0_`, `a1_`, `a2_`, `a3_`, `a4_`, `a5_`：五次多项式的系数。

2. 构造函数：
   - `QuinticPolynomial`：接受起始位置 `xs`、起始速度 `vxs`、起始加速度 `axs`、结束位置 `xe`、结束速度 `vxe`、结束加速度 `axe` 和时间 `time` 作为参数，计算五次多项式的系数。

3. 方法：
   - `calc_point`：计算给定时间 `t` 时的五次多项式值。
   - `calc_first_derivative`：计算给定时间 `t` 时的五次多项式一阶导数值。
   - `calc_second_derivative`：计算给定时间 `t` 时的五次多项式二阶导数值。
   - `calc_third_derivative`：计算给定时间 `t` 时的五次多项式三阶导数值。
*/
} // namespace beacon