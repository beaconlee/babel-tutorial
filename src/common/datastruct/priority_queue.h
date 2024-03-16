#pragma once

#include <queue>
#include <vector>
#include <utility>
#include <iostream>
#include <Eigen/Core>

namespace beacon
{

struct Vector2dGreater
{
  bool operator()(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) const
  {
    // Define your comparison criteria here
    // For example, prioritize lower x values:
    return lhs.x() < rhs.x();

    // You can add other criteria like:
    // - Prioritize lower y values if x values are equal
    // - Prioritize based on custom distance calculations
  }
};

struct CompareVector2d
{
  bool operator()(const std::pair<double, Eigen::Vector2d>& lhs,
                  const std::pair<double, Eigen::Vector2d>& rhs) const
  {
    // Compare the magnitude of the vectors
    return lhs.first > rhs.first;
  }
};

template <typename T, typename priority_t>
struct PriorityQueue
{
  using PQElement = std::pair<priority_t, T>;

  std::priority_queue<PQElement, std::vector<PQElement>, CompareVector2d>
      elements_;

  [[nodiscard]] inline bool Empty() const
  {
    return elements_.empty();
  }

  inline void Put(T item, priority_t priority)
  {
    // std::cout << __LINE__ << "\n";

    elements_.emplace(priority, item);
    // std::cout << __LINE__ << "\n";
  }

  T Get()
  {
    auto best_item = elements_.top().second;
    elements_.pop();
    // std::cout<<"pop\n";
    return best_item;
  }
};

} // namespace beacon