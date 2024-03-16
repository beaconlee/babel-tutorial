#pragma once

namespace beacon
{
struct VehicleConfig
{
  // VC(4.5, 1.0, 3.0, 3.5, 0.5, 1.0, 0.6);
  double rf_{4.5};    // [m] distance from rear to vehicle front end of vehicle
  double rb_{1.0};    // [m] distance from rear to vehicle back end of vehicle
  double width_{3.0}; // [m] width of vehicle
  double whell_distance_{2.1}; // [m] distance between left-right wheels
  double whell_base_{3.5};     // [m] Wheel base
  double tyre_radius_{0.5};    // [m] Tyre radius 轮胎半径
  double tyre_width_{1.0};     // [m] Tyre width
  double max_steer_{0.6};
  double max_speed_ = 55.0 / 3.6;
  double min_speed_ = -20.0 / 3.6;
};
} // namespace beacon