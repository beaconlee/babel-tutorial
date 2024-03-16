#include "a_star.h"



#include "src/common/datastruct/priority_queue.h"

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

Status AStar::Plan(const std::shared_ptr<KDTree>& obs,
                   Eigen::Vector2d goal,
                   std::shared_ptr<AstarResult> result)
{
  PriorityQueue<Eigen::Vector2d, double> frontier;
  // DEBUG_LOG

  frontier.Put(goal, 0);
  // DEBUG_LOG

  if(result.get() == nullptr)
  {
    std::cout << "nullptr\n";
  }
  DEBUG_LOG

  result->cost_so_far[goal] = 0;
  int64_t times = 0;
  DEBUG_LOG

  while(true)
  {
    ++times;

    if(times % 1000 == 0)
    {
      DEBUG_LOG
      std::cout << "times: " << times << "\n";
    }
    
    if(frontier.Empty())
    {
      return Status::OK;
    }

    auto curr = frontier.Get();
    // DEBUG_LOG
    for(Eigen::Vector2d next : Neighbors(curr, obs))
    {
      // DEBUG_LOG
      double new_cost = result->cost_so_far[curr] + 1;
      // DEBUG_LOG

      if(result->cost_so_far.find(next) == result->cost_so_far.end() ||
         new_cost < result->cost_so_far[next])
      {
        // DEBUG_LOG
        result->cost_so_far[next] = new_cost;
        double priority = new_cost + Heuristic(next, goal);
        frontier.Put(next, priority);
      }
      // DEBUG_LOG
    }
  }
  return Status::ERROR;
}
} // namespace beacon