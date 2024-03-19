#include "a_star.h"



#include "src/common/datastruct/priority_queue.h"
#include "src/common/utils.h"

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fmtmsg.h>
#include <fmt/printf.h>

namespace beacon
{
const std::array<DIRS, 8> AStar::motion_set_ = {
    {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}}};

std::vector<Eigen::Vector2d>
AStar::Neighbors(Eigen::Vector2d curr, const std::shared_ptr<KDTree>& obs) const
{
  std::vector<Eigen::Vector2d> results;
  results.reserve(8);
  for(auto dir : motion_set_)
  {
    Eigen::Vector2d next{curr.x() + dir.x, curr.y() + dir.y};
    // DEBUG_LOG
    auto result = obs->NeighborhoodIndices({next.x(), next.y()}, 1.0);
    // DEBUG_LOG

    if(result.size() == 0)
    {
      results.push_back(next);
    }
  }
  return results;
}

std::vector<Eigen::Vector2d>
AStar::Neighbors(Eigen::Vector2d curr,
                 const std::vector<std::vector<double>>& obs) const
{
  // std::cout << "curr.x()" << curr.x() << "  curr.y()" << curr.y() << "\n";
  std::vector<Eigen::Vector2d> results;
  results.reserve(8);
  DEBUG_LOG
  // std::cout << "将要执行 motion_set_\n";
  DEBUG_LOG

  for(auto dir : motion_set_)
  {
    Eigen::Vector2d next{curr.x() + dir.x, curr.y() + dir.y};
    // std::cout << "next.x()" << next.x() << "  next.y()" << next.y() << "\n";

    if(next.x() <= minx_ || next.x() >= maxx_ || next.y() <= miny_ ||
       next.y() >= maxy_)
    {
      // std::cout << "执行 continue\n";
      // std::cout << "minx_" << maxx_ << " miny_" << miny_ << " maxx_" << maxx_
      //           << " maxy_" << maxy_ << "\n";
      continue;
    }
    DEBUG_LOG

    if(obs[next.x()][next.y()] == 0)
    {
      results.push_back(next);
    }
  }
  DEBUG_LOG

  // std::cout << "results.size() " << results.size() << "\n";
  return results;
}

Status AStar::Plan(const std::shared_ptr<KDTree>& obs,
                   Eigen::Vector2d start,
                   Eigen::Vector2d goal,
                   std::shared_ptr<AstarResult> result)
{
  PriorityQueue<Eigen::Vector2d, double> frontier;
  frontier.Put(start, 0);
  result->came_from[start] = start;
  result->cost_so_far[start] = 0;

  while(!frontier.Empty())
  {
    auto curr = frontier.Get();
    if(curr == goal)
    {
      return Status::OK;
    }

    for(auto next : Neighbors(curr, obs))
    {
      double new_cost = result->cost_so_far[curr] + 1;

      if(result->cost_so_far.find(next) == result->cost_so_far.end() ||
         new_cost < result->cost_so_far[next])
      {
        result->cost_so_far[next] = new_cost;
        double priority = new_cost + Heuristic(next, goal);
        result->came_from[next] = curr;
        frontier.Put(next, priority);
      }
    }
  }
  return Status::ERROR;
}

void AStar::UpdateAStar(std::vector<std::vector<double>> obs)
{
  minx_ = 1;
  miny_ = 1;
  maxx_ = 50;
  maxy_ = 30;
  // std::cout << "minx_" << maxx_ << " miny_" << miny_ << " maxx_" << maxx_
  //           << " maxy_" << maxy_ << "\n";
}

Status AStar::Plan(const std::vector<std::vector<double>>& obs,
                   Eigen::Vector2d goal,
                   std::shared_ptr<AstarResult> result)
{
  UpdateAStar(obs);
  // std::cout << "obs.size(): " << obs.size() << "\n";
  // std::cout << "obs[0].size(): " << obs[0].size() << "\n";

  PriorityQueue<Eigen::Vector2d, double> frontier;
  frontier.Put(goal, 0);
  DEBUG_LOG

  if(result.get() == nullptr)
  {
    std::cout << "nullptr\n";
    return Status::ERROR;
  }
  DEBUG_LOG

  result->hmap_[goal.x()][goal.y()] = 0;

  // for(int i = 0; i < result->hmap_.size(); ++i)
  // {
  //   for(int j = 0; j < result->hmap_[0].size(); ++j)
  //   {
  //     std::cout << result->hmap_[i][j] << ' ';
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "\n";
  // std::cout << "\n";
  // std::cout << "\n";

  DEBUG_LOG

  while(true)
  {
    if(frontier.Empty())
    {
      break;
    }
    DEBUG_LOG

    auto curr = frontier.Get();
    DEBUG_LOG
    // DEBUG_LOG
    for(Eigen::Vector2d next : Neighbors(curr, obs))
    {
      // DEBUG_LOG
      // std::cout << "enter neighbors\n";
      // std::cout << "cuur.x()" << curr.x() << "  curr.y()" << curr.y() << " ";
      double new_cost = result->hmap_[curr.x()][curr.y()] + 4;
      // DEBUG_LOG

      if(new_cost < result->hmap_[next.x()][next.y()])
      {
        // DEBUG_LOG
        result->cost_so_far[next] = new_cost;
        result->hmap_[next.x()][next.y()] = new_cost;
        double priority = new_cost + Heuristic(next, goal);
        frontier.Put(next, priority);
      }
      // DEBUG_LOG
    }
  }

  // for(int i = 0; i < result->hmap_.size(); ++i)
  // {
  //   for(int j = 0; j < result->hmap_[0].size(); ++j)
  //   {
  //     std::cout << std::setw(4) << std::setfill(' ') << result->hmap_[i][j]
  //               << ' ';
  //   }
  //   std::cout << "\n";
  // }

  return Status::OK;
}
} // namespace beacon