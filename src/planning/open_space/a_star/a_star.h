#pragma once


#include "src/common/status.h"
#include "src/common/datastruct/kd_tree.h"


#include <array>
#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <functional>
#include <unordered_map>



template <>
struct std::hash<Eigen::Vector2d>
{
  std::size_t operator()(const Eigen::Vector2d& grid) const noexcept
  {
    return std::hash<double>()(grid.x() * grid.x() * grid.x() * grid.x() *
                               grid.y() * grid.y()*0.987654314324);
  }
};


namespace beacon
{

namespace
{
struct DIRS
{
  int x;
  int y;
};

// class MotionType
// {
//   // East West North South
//   static constexpr DIRS North = {1, 0};
//   static constexpr DIRS West = {0, -1};
//   static constexpr DIRS East = {0, 1};
//   static constexpr DIRS South = {-1, 0};
//   static constexpr DIRS East_North = {1, 1};
//   static constexpr DIRS East_South = {1, -1};
//   static constexpr DIRS West_North = {-1, 1};
//   static constexpr DIRS West_South = {-1, -1};
// };

} // namespace

class AstarResult
{
public:
  std::unordered_map<Eigen::Vector2d, Eigen::Vector2d> came_from{};
  std::unordered_map<Eigen::Vector2d, double> cost_so_far{};
};

class AStar
{
public:
  Status Plan(const std::shared_ptr<KDTree>& obs,
              Eigen::Vector2d start,
              Eigen::Vector2d goal,
              std::shared_ptr<AstarResult> result);

  Status Plan(const std::shared_ptr<KDTree>& obs,
              Eigen::Vector2d goal,
              std::shared_ptr<AstarResult> result);

private:
  double Heuristic(Eigen::Vector2d a, Eigen::Vector2d b) const
  {
    return std::abs(a.x() - b.x()) + std::abs(a.y() - b.y());
  }

  std::vector<Eigen::Vector2d>
  Neighbors(Eigen::Vector2d curr, const std::shared_ptr<KDTree>& obs) const;

  static const std::array<DIRS, 8> motion_set_;
  // 下面这样编译会报错
  // static constexpr std::array<DIRS, 8> motion_set_ = {
  //   {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}}};
};



} // namespace beacon