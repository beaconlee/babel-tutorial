#include "mcubic_spline.h"
#include <memory>



namespace
{
template <typename T>
std::vector<T> Diff(const std::vector<T>& vec)
{
  std::vector<T> ret;
  for(size_t idx = 1; idx < vec.size(); ++idx)
  {
    ret.push_back(vec.at(idx) - vec.at(idx - 1));
  }
  return ret;
}

template <typename T>
std::vector<T> CumSum(const std::vector<T>& vec)
{
  std::vector<T> ret;
  T tmp = 0;
  for(size_t idx = 0; idx < vec.size(); ++idx)
  {
    tmp += vec.at(idx);
    ret.push_back(tmp);
  }
  return ret;
}
} // namespace

namespace beacon
{

CubicSpline::CubicSpline(std::vector<double> x, std::vector<double> y)
  : h_(Diff(x))
  , x_coord_(std::move(x))
  , y_coord_(std::move(y))
  , size_(x_coord_.size())
{
  bool not_valid = std::any_of(
      std::begin(h_), std::end(h_), [](double val) { return val < 0; });
  if(not_valid)
  {
    throw std::invalid_argument(
        "x coordinates must be sorted in ascending order");
  }

  a_ = y_coord_;
  Eigen::MatrixXd a = CalcA();
  Eigen::MatrixXd b = CalcB();

  Eigen::VectorXd c = a.colPivHouseholderQr().solve(b);
  double* c_pointor = c.data();
  c_.assign(c_pointor, c_pointor + c.rows());

  for(size_t idx = 0; idx < (size_ - 1); ++idx)
  {
    double d_tmp = (c_.at(idx + 1) - c_.at(idx)) / (3.0 * h_.at(idx));
    double b_tmp = (a_.at(idx + 1) - a_.at(idx)) / h_.at(idx) -
                   h_.at(idx) * (c_.at(idx + 1) + 2 * c_.at(idx)) / 3.;
    d_.push_back(d_tmp);
    b_.push_back(b_tmp);
  }
}

Eigen::MatrixXd CubicSpline::CalcA()
{
  // size_ = points.size();
  // 有 n-1 行 + 2 个首尾点确定的行 = n+1 = size_;
  //
  // 参数矩阵 A 的大小是 (size_, size_)
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(size_, size_);
  size_t idx = 0;

  // 循环目的：对 1~n-1 行进行赋值
  // for(; idx < diff_h.size() - 2; ++idx)
  // 这里我犯了一个错误，这里我使用的是 diff_h.size()
  // 实际应该使用的是 size_
  for(; idx < size_ - 2; ++idx)
  {
    a(idx + 1, idx + 1) = 2 * (h_[idx] + h_[idx + 1]);
    // 对 对角点 的 左边 和 上边 赋值
    // 当前点的左边
    a(idx + 1, idx) = h_[idx];
    // 这里会对 (0, 1) 进行赋值
    // 但这个循环的目的是对 1~n-1 行进行赋值，所以在循环后需要清除
    // 当前点的上面
    a(idx, idx + 1) = h_[idx];
  }
  // 推出循环后 idx = size_ - 2;  也就是倒数第二行
  // 上面的循环对 a(0, 1) 进行赋值了，这里应该是0
  a(0, 1) = 0.;
  // 对 (size_-2, size_-2) 的右边一个 (size_-2, size_-1) 进行赋值
  a(idx, idx + 1) = h_[idx];

  // 增加端点条件，增加最后两个约束条件，使得方程组数目正好为 4n
  // 这里采用的是 自由边界（Nature）
  // 对第一行赋值，初始点
  a(0, 0) = 1.;
  // 对最后一行赋值，结束点
  a(size_ - 1, size_ - 1) = 1.;
  a(size_ - 1, size_ - 2) = 0.;

  return a;
}

Eigen::MatrixXd CubicSpline::CalcB()
{
  Eigen::VectorXd b = Eigen::VectorXd::Zero(size_);
  for(size_t idx = 1; idx < size_ - 2; ++idx)
  {
    b(idx) = 6. * (a_.at(idx + 1) - a_.at(idx) / h_.at(idx) -
                   (a_.at(idx) - a_.at(idx - 1)) / h_.at(idx - 1));
  }

  return b;

  // 这里还有种写法，但是我感觉这样会多计算一次 idx+1
  /*
  for(size_t idx = 0; idx < (nx - 2); ++idx)
  {
    B(idx + 1) = 3.0 * (a[idx + 2] - a[idx + 1]) / h[idx + 1] -
                 3.0 * (a[idx + 1] - a[idx]) / h[idx];
  }
  */
}


double CubicSpline::CalcPosition(double x) const
{
  if(!CheckX(x))
  {
    return std::numeric_limits<double>::min();
  }

  // 这种做法很重要，要记住啊
  auto it = std::upper_bound(std::begin(x_coord_), std::end(x_coord_), x);
  int idx = std::distance(std::begin(x_coord_), it);

  double dx = x - x_coord_.at(idx);

  return a_.at(idx) + b_.at(idx) * dx + c_.at(idx) * std::pow(dx, 2) +
         d_.at(idx) * std::pow(dx, 3);
}


bool CubicSpline::CheckX(double x) const
{
  if(x_coord_.empty())
  {
    return false;
  }

  if(x < *std::begin(x_coord_) || x > *std::end(x_coord_))
  {
    return false;
  }

  return true;
}

double CubicSpline::Calc1Derivative(double x) const
{
  if(!CheckX(x))
  {
    return std::numeric_limits<double>::min();
  }

  auto it = std::upper_bound(std::begin(x_coord_), std::end(x_coord_), x);
  int32_t idx = std::distance(std::begin(x_coord_), it);

  double dx = x - x_coord_.at(idx);
  return b_.at(idx) + 2 * c_.at(idx) * dx + 3 * d_.at(idx) * pow(dx, 2);
}


double CubicSpline::Calc2Derivative(double x) const
{
  if(!CheckX(x))
  {
    return std::numeric_limits<double>::min();
  }

  auto it = std::upper_bound(std::begin(x_coord_), std::end(x_coord_), x);
  int32_t idx = std::distance(std::begin(x_coord_), it);

  double dx = x - x_coord_.at(idx);
  return 2 * c_.at(idx) + 6 * d_.at(idx) * dx;
}

double CubicSpline::operator()(double x, int32_t deriv) const
{
  if(!CheckX(x))
  {
    return std::numeric_limits<double>::min();
  }

  switch(deriv)
  {
    case 0:
      {
        return CalcPosition(x);
      }
    case 1:
      {
        return Calc1Derivative(x);
      }
    case 2:
      {
        return Calc2Derivative(x);
      }
    default:
      {
        return std::numeric_limits<double>::min();
      }
  }
}


CubicSpline2D::CubicSpline2D(std::vector<double> x, std::vector<double> y)
  : xs_(x, y)
  , ys_(x, y)
{
  s_ = CalcS(x, y);
}

Eigen::Vector2d CubicSpline2D::CalcPosition(double s) const
{
  double x = xs_.CalcPosition(s);
  double y = ys_.CalcPosition(s);

  return {x, y};
}

double CubicSpline2D::CalcYaw(double s) const
{
  double dx = xs_.Calc1Derivative(s);
  double dy = ys_.Calc1Derivative(s);

  return std::atan2(dy, dx);
}

double CubicSpline2D::CalcCurvatur(double s) const
{
  double dx = xs_.Calc1Derivative(s);
  double dy = ys_.Calc1Derivative(s);

  double ddx = xs_.Calc2Derivative(s);
  double ddy = ys_.Calc2Derivative(s);

  return (ddy * dx - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
}

Eigen::Vector2d CubicSpline2D::operator()(double s, int32_t n) const
{
  return {xs_(s, n), ys_(s, n)};
}

std::vector<double> CubicSpline2D::CalcS(std::vector<double> x,
                                         std::vector<double> y)
{
  DEBUG_LOG
  std::vector<double> dx = Diff(x);
  std::vector<double> dy = Diff(y);
  DEBUG_LOG
  std::vector<double> ds;
  ds.resize(x.size());
  std::vector<double> s;
  s.resize(x.size());
  DEBUG_LOG

  // for(size_t idx = 0; idx < x.size(); ++idx)
  // 这里犯了一个错
  // diff 的大小 比 x 的大小少一个
  for(size_t idx = 0; idx < dx.size(); ++idx)
  {
    ds.emplace_back(std::hypot(dx.at(idx), dy.at(idx)));
  }
  auto cum = CumSum(ds);
  s.insert(std::end(s), std::begin(cum), std::end(cum));
  DEBUG_LOG

  return s;
}


std::vector<std::vector<double>> CubicSpline2D::CalcSplineCourse(
    std::vector<double> x, std::vector<double> y, double ds)
{
  std::unique_ptr<CubicSpline2D> up = std::make_unique<CubicSpline2D>(x, y);
  std::vector<std::vector<double>> output(4);

  for(double s = up->s_.front(); s < up->s_.back(); s += ds)
  {
    auto ixy = up->CalcPosition(s);
    output[0].push_back(ixy(0));
    output[1].push_back(ixy(1));
    output[2].push_back(up->CalcYaw(s));
    output[3].push_back(up->CalcCurvatur(s));
  }

  return output;
}

} // namespace beacon