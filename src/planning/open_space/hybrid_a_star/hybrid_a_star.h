#pragma once

#include "../a_star/a_star.h"
#include "trajectory_node.h"

#include "src/common/status.h"
#include "src/common/frame.h"

#include <Eigen/Core>
#include <vector>

namespace beacon
{

class HybridAStarResult
{
public:
  std::unordered_map<Eigen::Vector2d, Eigen::Vector2d> came_from{};
  std::unordered_map<Eigen::Vector2d, double> cost_so_far{};
};

class HybridAStar
{
public:
  void Init(point_arr_t points);

  Status Plan(Eigen::Vector3d start,
              Eigen::Vector3d goal,
              std::shared_ptr<Frame> frame);


private:
  std::shared_ptr<AStar> astar_;
  std::shared_ptr<Frame> frame_;
  std::shared_ptr<AstarResult> astar_result_;

  void CalcNextNode();

  int CalcIndex(const std::shared_ptr<TrajectoryNode>& node);

  void CalcMotionSet(std::shared_ptr<Frame> frame);

  double CalcHybridCost(const std::shared_ptr<TrajectoryNode>& node);

  void CalcParameters(point_arr_t points);

  std::pair<std::vector<double>, std::vector<int>> motion_set_;
};
} // namespace beacon