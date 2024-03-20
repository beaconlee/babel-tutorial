#pragma once



#include <vector>

namespace beacon
{
struct TrajectoryNode
{
  int x_coord_{0};
  int y_coord_{0};
  int yaw_{0};
  int direction_{};

  std::vector<double> x_list_{};
  std::vector<double> y_list_{};
  std::vector<double> yaw_list_{};
  std::vector<int> direction_list_{};

  double steer_{0.};
  double cost_{0.};
  int pind_{0};

  TrajectoryNode(int x_coord,
                 int y_coord,
                 int yaw,
                 int direction,
                 std::vector<double> x_list,
                 std::vector<double> y_list,
                 std::vector<double> yaw_list,
                 std::vector<int> direction_list,
                 double steer,
                 double cost,
                 double pind)
    : x_coord_(x_coord)
    , y_coord_(y_coord)
    , yaw_(yaw)
    , direction_(direction)
    , x_list_(x_list)
    , y_list_(y_list)
    , yaw_list_(yaw_list)
    , direction_list_(direction_list)
    , steer_(steer)
    , cost_(cost)
    , pind_(pind)
  {}

  TrajectoryNode(int x_coord, int y_coord, int yaw)
    : x_coord_(x_coord)
    , y_coord_(y_coord)
    , yaw_(yaw)

  {}


  TrajectoryNode() = default;
  TrajectoryNode(const TrajectoryNode&) = default;
  ~TrajectoryNode() = default;
};
} // namespace beacon