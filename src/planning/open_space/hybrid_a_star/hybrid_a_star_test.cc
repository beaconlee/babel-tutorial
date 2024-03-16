#include "hybrid_a_star.h"

int main()
{
  beacon::HybridAStar ha;
  std::shared_ptr<beacon::Frame> frame = std::make_shared<beacon::Frame>();


  beacon::point_arr_t points;
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

  Eigen::Vector3d start{1.0, 2.0, 3.0};
  Eigen::Vector3d goal{1.0, 2.0, 3.0};
  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  ha.Init(points);
  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  std::cout << "enter plan\n";
  ha.Plan(start, goal, frame);
  return 0;
}