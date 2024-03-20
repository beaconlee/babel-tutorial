#include "hybrid_a_star.h"
#include "src/common/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
  plt::title("Hybrid A*");

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
  for(double idx = 15; idx < 29.5; idx += 0.5)
  {
    obs[0].push_back(30);
    obs[1].push_back(idx);
  }
  for(double idx = 0; idx < 16; idx += 0.5)
  {
    obs[0].push_back(40);
    obs[1].push_back(idx);
  }
  plt::plot(obs[0], obs[1], "sk");

  beacon::HybridAStar ha;
  std::shared_ptr<beacon::Frame> frame = std::make_shared<beacon::Frame>();


  Eigen::Vector3d start{10.0, 7.0, 120 * M_PI / 180};
  Eigen::Vector3d goal{45.0, 20.0, M_PI_2};
  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  ha.Init(obs);
  std::cout << "enter plan\n";
  ha.Plan(start, goal, frame);

  
  plt::show();
  return 0;
}