#pragma once

#include <vector>
#include <Eigen/Eigen>

class CubicSpline
{
private:
  std::vector<double> x; // 节点的横坐标
  std::vector<double> y; // 节点的纵坐标

  /*
  矩阵 A：矩阵 A 是一个 nx × nx 的对称三对角矩阵，其中 nx 是插值节点的数量。矩阵 A 的主对角线和两条相邻的对角线上的元素用于表示插值函数的二阶导数，而其他元素都为零。具体地，对于内部节点 i，矩阵 A 的对角线元素为 2*(h[i-1]+h[i])，其中 h[i] 表示节点 i 的步长（即相邻节点间的距离）。矩阵 A 的相邻对角线元素为 h[i-1]，表示相邻节点之间的关系。首尾节点处，由于缺少相邻节点，对应的元素为特殊值。

  矩阵 B：矩阵 B 是一个 nx × m 的矩阵，其中 m 是插值函数的输出维度（通常为 1）。矩阵 B 的每一行对应一个插值节点，每一列对应插值函数的一个输出维度。矩阵 B 的元素由插值节点处的输出值组成。
  */
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
  std::vector<double> h; // 每两个点之间的距离  h = diff(x);
  int nx;                // nx = x.size();   插入节点的数量

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
