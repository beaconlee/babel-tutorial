#include "mcubic_spline.h"
#include <iostream>
#include "src/common/matplotlibcpp.h"

namespace plt = matplotlibcpp;


int main()
{
  std::cout << "hello world\n";

  std::vector<double> wx = {0.0, 10.0, 20.5, 35.0, 70.5};
  std::vector<double> wy = {0.0, -6.0, 5.0, 6.5, 0.0};
  beacon::CubicSpline2D cs2(wx, wy);
  plt::plot();
  return 0;
}