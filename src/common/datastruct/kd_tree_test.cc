#include "kd_tree.h"
#include <iostream>

int main()
{
  beacon::point_arr_t points;
  for(size_t idx = 0; idx < 50; ++idx)
  {
    points.push_back({static_cast<double>(idx), static_cast<double>(idx)});
  }

  std::unique_ptr<beacon::KDTree> kdtree;
  kdtree = std::make_unique<beacon::KDTree>(points);

  std::vector<beacon::point_t> ids =
      kdtree->NeighborhoodPoints({20.0, 20.0}, 5);

  std::cout << "neighborhood points:\n";
  for(auto item : ids)
  {
    for(auto item_idx : item)
    {
      std::cout << item_idx << " ";
    }
    std::cout << "\n";
  }
  auto nearest = kdtree->NearestPoint({22.0, 12.0});

  std::cout << "nearest point:\n";
  for(auto dim : nearest)
  {
    std::cout << dim << " ";
  }
  std::cout << "\n";
  return 0;
}