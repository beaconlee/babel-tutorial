#include <cmath>
#include <string>
#include <fmt/core.h>
#include <Eigen/Core>

#include "src/common/utils.h"
#include "src/common/matplotlibcpp.h"
#include "reeds_shepp.h"

using std::string;
using namespace Eigen;
namespace plt = matplotlibcpp;
constexpr bool show_animation = true;

int main(int argc, char** argv)
{
  Vector3d start(-10.0, -10.0, M_PI_4);
  Vector3d goal(0., 0., -M_PI_2);
  double curvature = 0.1;
  double step_size = 0.05;
  Path path = reeds_shepp_path(start, goal, curvature, step_size);
  std::cout << "解析 最终的 path\n";
  std::cout << "解析 最终的 path\n";
  std::cout << "解析 最终的 path\n";
  std::cout << "解析 最终的 path\n";
  for(size_t idx = 0; idx < path.x.size(); ++idx)
  {
    std::cout << "(" << path.x[idx] << ", " << path.y[idx] << ", "
              << path.yaw[idx] << ")"
              << " ";
  }


  string final_mode = "final course ";
  final_mode.push_back(path.ctypes[0]);
  final_mode.push_back(path.ctypes[1]);
  final_mode.push_back(path.ctypes[2]);

  double steer = 0.0;
  if(show_animation)
  {
    for(size_t idx = 0; idx < path.x.size(); ++idx)
    {
      // plt::cla();
      // plt::named_plot(final_mode, path.x, path.y);
      // plt::arrow(start[0], start[1], cos(start[2]), sin(start[2]), "r", 0.075);
      // plt::arrow(goal[0], goal[1], cos(goal[2]), sin(goal[2]), "g", 0.075);

      if(idx < path.x.size() - 2)
      {
        double dy = (path.yaw[idx + 1] - path.yaw[idx]) / 0.1;
        steer = -beacon::Mod2Pi(atan(-3.5 * dy / path.directions[idx]));
      }
      else
      {
        steer = 0.0;
      }
      // utils::draw_vehicle({path.x[idx], path.y[idx], path.yaw[idx]}, steer, vc);
      // plt::legend({{"loc", "upper left"}});
      // plt::grid(true);
      // plt::axis("equal");
      // plt::title("Reeds Shepp Path Planning");
      // plt::pause(0.001);
    }
    // plt::show();
  }

  return 0;
}
