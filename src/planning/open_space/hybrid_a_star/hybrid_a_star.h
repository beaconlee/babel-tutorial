#pragma once

#include "src/planning/open_space/a_star/a_star.h"
#include "trajectory_node.h"
#include "reeds_sheep_path.h"

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
  std::shared_ptr<ReedsSheppPath> rs_path_;
  std::shared_ptr<ReedsShepp> rs_;

  std::shared_ptr<TrajectoryNode> CalcNextNode(
      std::shared_ptr<TrajectoryNode>& curr_node, int c_id, double u, int d);

  int CalcIndex(const std::shared_ptr<TrajectoryNode>& node);

  void CalcMotionSet(std::shared_ptr<Frame> frame);

  double CalcHybridCost(const std::shared_ptr<TrajectoryNode>& node);

  void CalcParameters(point_arr_t points);

  bool IsIndexOk(int xind,
                 int yind,
                 const std::vector<double>& xlist,
                 const std::vector<double>& ylist,
                 const std::vector<double>& yawlist);

  bool IsCollision(std::vector<double>& x,
                   std::vector<double>& y,
                   std::vector<double>& yaw);

  bool ReedsSheepPath(std::shared_ptr<TrajectoryNode> n_curr,
                      std::shared_ptr<TrajectoryNode> ngoal,
                      std::shared_ptr<TrajectoryNode>& fpath);

  std::pair<std::vector<double>, std::vector<int>> motion_set_;
};
} // namespace beacon