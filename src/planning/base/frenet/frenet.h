#pragma once

#include <vector>



namespace beacon
{
struct FrenetPath
{
  std::vector<double> x_list_{};
  std::vector<double> y_list_{};
  std::vector<double> yaw_list_{};
  std::vector<double> ds_{};
  std::vector<double> c_{};


  double max_speed_{0.0};
  double max_accel_{0.0};
  double max_curvature_{0.0};

  std::vector<double> t_{};
  std::vector<double> s_{};
  std::vector<double> sd_{};
  std::vector<double> sdd_{};
  std::vector<double> sddd_{};
  std::vector<double> l_{};
  std::vector<double> ld_{};
  std::vector<double> ldd_{};
  std::vector<double> lddd_{};
};


class Frenet
{
public:

private:


};

} // namespace beacon