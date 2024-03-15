#pragma once

#include <vector>
#include <Eigen/Core>
namespace beacon
{
constexpr double DEFAULT_STRP = 0.1;

class ReedsSheppPath
{
public:
  std::vector<double> lengths_;
  std::vector<char> char_;

  double l_{0.0};
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<double> yaw_vec_;
  std::vector<double> yaw_reso_vec_;
  std::vector<int> direc_vec_;

  ReedsSheppPath() = default;
  ~ReedsSheppPath() = default;

  ReedsSheppPath(std::vector<double> x_vec,
                 std::vector<double> y_vec,
                 std::vector<double> yaw_vec,
                 std::vector<int> dir_vec)
    : x_vec_(std::move(x_vec))
    , y_vec_(std::move(y_vec))
    , yaw_vec_(std::move(yaw_vec))
    , direc_vec_(std::move(dir_vec))
  {}

  ReedsSheppPath(std::vector<double> x_vec,
                 std::vector<double> y_vec,
                 std::vector<double> yaw_vec,
                 std::vector<double> yaw_reso_vec,
                 std::vector<int> dir_vec)
    : x_vec_(std::move(x_vec))
    , y_vec_(std::move(y_vec))
    , yaw_vec_(std::move(yaw_vec))
    , yaw_reso_vec_(std::move(yaw_reso_vec))
    , direc_vec_(std::move(dir_vec))
  {}
};

ReedsSheppPath GenReedsSheppPath(Eigen::Vector3d start,
                                 Eigen::Vector3d goal,
                                 double maxc,
                                 double step_size = DEFAULT_STRP);

std::vector<ReedsSheppPath> CalcRSPaths(Eigen::Vector3d start,
                                        Eigen::Vector3d goal,
                                        double macx,
                                        double step_size);

} // namespace beacon