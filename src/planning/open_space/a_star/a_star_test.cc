#include "a_star.h"

// #include <fmt/printf.h>
#include <iostream>

int main()
{
  beacon::point_arr_t points;
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";
  for(size_t idx = 0; idx <= 50; ++idx)
  {
    points.push_back({static_cast<double>(0), static_cast<double>(idx)});
    points.push_back({static_cast<double>(50), static_cast<double>(idx)});
  }

  for(size_t idx = 1; idx <= 50; ++idx)
  {
    points.push_back({static_cast<double>(idx), static_cast<double>(0)});
    points.push_back({static_cast<double>(idx), static_cast<double>(50)});
  }
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";

  std::shared_ptr<beacon::KDTree> kdtree =
      std::make_shared<beacon::KDTree>(points);
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";

  beacon::AStar a;
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";

  std::shared_ptr<beacon::AstarResult> resutlt =
      std::make_shared<beacon::AstarResult>();
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";

  a.Plan(kdtree, Eigen::Vector2d{25, 25}, resutlt);
  std::cout << "\n";
  // fmt::print("{}", __LINE__);
  std::cout << __LINE__ << "\n";

  std::cout << "result:\n";
  std::cout << resutlt->came_from.size() << "\n";
  std::cout << resutlt->cost_so_far.size() << "\n";

  return 0;
}