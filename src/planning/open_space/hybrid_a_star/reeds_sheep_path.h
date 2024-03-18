#pragma once

#include "src/common/frame.h"
#include "src/common/status.h"
#include "src/planning/open_space/hybrid_a_star/trajectory_node.h"


#include <memory>
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

  ReedsSheppPath(std::vector<double> lengths, std::vector<char> cchar, double l)
    : lengths_(std::move(lengths))
    , char_(std::move(cchar))
    , l_(l)
  {}

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

class ReedsShepp
{
public:
  ReedsSheppPath AnalysticExpantion(std::shared_ptr<TrajectoryNode> start,
                                    std::shared_ptr<TrajectoryNode> goal,
                                    std::shared_ptr<Frame>& frame,
                                    double step_size = DEFAULT_STRP);
  double CalcRspathCost(ReedsSheppPath rspath, std::shared_ptr<Frame>& frame);

private:
  std::vector<ReedsSheppPath> CalcRSPaths(Eigen::Vector3d start,
                                          Eigen::Vector3d goal,
                                          double macx,
                                          double step_size);


  bool IsCollision(std::vector<double>& x,
                   std::vector<double>& y,
                   std::vector<double>& yaw,
                   std::shared_ptr<Frame>& frame);

  std::vector<ReedsSheppPath>
  GenReedsSheppPath(Eigen::Vector3d start,
                    Eigen::Vector3d goal,
                    double maxc,
                    double step_size = DEFAULT_STRP);

  std::vector<std::vector<double>> SolveRSPath(std::vector<double> lengths,
                                               std::vector<char> modes,
                                               double max_curvature,
                                               double step_size);


  //计算插值距离列表
  std::vector<std::vector<double>>
  CalcInterpolateDistsList(std::vector<double> lengths, double step_size);


  /*
  "interpolate" 是指在两个或多个已知值之间推断出中间值的过程。在数学和计算机科学中，插值是一种常见的技术，用于通过已知数据点之间的关系来估计中间点的值。这种方法通常用于创建平滑的曲线或表面，以便在连续的空间中表示离散的数据。
  */
  Eigen::Vector4d Interpolate(double dist,
                              double length,
                              char mode,
                              double max_curvature,
                              Eigen::Vector3d origin);
};



// ReedsSheppPath GenReedsSheppPath(Eigen::Vector3d start,
//                                  Eigen::Vector3d goal,
//                                  double maxc,
//                                  double step_size = DEFAULT_STRP);

// std::vector<ReedsSheppPath> CalcRSPaths(Eigen::Vector3d start,
//                                         Eigen::Vector3d goal,
//                                         double macx,
//                                         double step_size);

} // namespace beacon