#pragma once

#include "src/common/frame.h"
#include "src/common/status.h"
#include "trajectory_node.h"


#include <memory>
#include <vector>
#include <Eigen/Core>
namespace beacon
{

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
                                    double step_size = MOVE_STEP);
  double CalcRspathCost(ReedsSheppPath rspath, std::shared_ptr<Frame>& frame);

  std::vector<ReedsSheppPath> CalcRSPaths(Eigen::Vector3d start,
                                          Eigen::Vector3d goal,
                                          double macx,
                                          double step_size);

private:
  bool IsCollision(std::vector<double>& x,
                   std::vector<double>& y,
                   std::vector<double>& yaw,
                   std::shared_ptr<Frame>& frame);

  std::vector<ReedsSheppPath> GenReedsSheppPath(Eigen::Vector3d start,
                                                Eigen::Vector3d goal,
                                                double maxc,
                                                double step_size = MOVE_STEP);

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


class Path
{
public:
  std::vector<double> lengths;
  std::vector<char> ctypes;
  double L;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> yawt;
  std::vector<int> directions;

  Path(std::vector<double> _x,
       std::vector<double> _y,
       std::vector<double> _yaw,
       std::vector<int> _dir)
    : x(_x)
    , y(_y)
    , yaw(_yaw)
    , directions(_dir)
  {}
  Path(std::vector<double> _x,
       std::vector<double> _y,
       std::vector<double> _yaw,
       std::vector<double> _yawt,
       std::vector<int> _dir)
    : x(_x)
    , y(_y)
    , yaw(_yaw)
    , yawt(_yawt)
    , directions(_dir)
  {}
  Path() {}
  ~Path() {}
};

ReedsSheppPath reeds_shepp_path(Eigen::Vector3d s,
                                Eigen::Vector3d g,
                                double maxc,
                                double step_size = 0.2);

} // namespace beacon