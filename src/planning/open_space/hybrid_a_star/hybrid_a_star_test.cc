#include "hybrid_a_star.h"
#include "src/common/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
  beacon::HybridAStar ha;
  std::shared_ptr<beacon::Frame> frame = std::make_shared<beacon::Frame>();


  beacon::point_arr_t points;
  for(size_t idx = 1; idx <= 50; ++idx)
  {
    points.push_back({static_cast<double>(1), static_cast<double>(idx)});
    points.push_back({static_cast<double>(50), static_cast<double>(idx)});
  }

  for(size_t idx = 2; idx <= 50; ++idx)
  {
    points.push_back({static_cast<double>(idx), static_cast<double>(1)});
    points.push_back({static_cast<double>(idx), static_cast<double>(50)});
  }

  std::vector<std::vector<double>> obs(2);
  for(size_t idx = 0; idx < 50; ++idx)
  {
    obs[0].push_back(idx);
    obs[1].push_back(0);
  }
  for(size_t idx = 0; idx < 50; ++idx)
  {
    obs[0].push_back(idx);
    obs[1].push_back(30 - 1);
  }
  for(double idx = 0; idx < 30 - 0.5; idx += 0.5)
  {
    obs[0].push_back(0);
    obs[1].push_back(idx);
  }
  for(double idx = 0; idx < 30 - 0.5; idx += 0.5)
  {
    obs[0].push_back(50 - 1);
    obs[1].push_back(idx);
  }
  for(size_t idx = 10; idx < 21; ++idx)
  {
    obs[0].push_back(idx);
    obs[1].push_back(15);
  }
  for(double idx = 0; idx < 15; idx += 0.5)
  {
    obs[0].push_back(20);
    obs[1].push_back(idx);
  }
  for(double idx = 15; idx < 30; idx += 0.5)
  {
    obs[0].push_back(30);
    obs[1].push_back(idx);
  }
  for(double idx = 0; idx < 16; idx += 0.5)
  {
    obs[0].push_back(40);
    obs[1].push_back(idx);
  }

  // for(auto point : points)
  // {
  //   std::cout << point[0] << point[1] << "\n";
  // }

  plt::title("Hybrid A*");
  // for(size_t idx = 1; idx < points.size(); ++idx)
  // {
  //   plt::plot(points[idx], points[idx], "sk");
  // }
  plt::plot(obs[0], obs[1], "sk");

  plt::show();


  // Eigen::Vector3d start{1.0, 2.0, 3.0};
  // Eigen::Vector3d goal{1.0, 2.0, 3.0};
  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  // ha.Init(points);
  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  // std::cout << "enter plan\n";
  // ha.Plan(start, goal, frame);
  return 0;
}