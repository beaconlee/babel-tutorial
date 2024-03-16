#pragma once

#include "vehicle_config.h"
#include "datastruct/kd_tree.h"
#include "para.h"

#include <boost/math/constants/constants.hpp>

namespace beacon
{

constexpr bool show_animation = true;
constexpr int N_STEER = 3;      // steer command number
constexpr double XY_RESO = 2.0; // [m]
constexpr double YAW_RESO =
    15 * boost::math::constants::pi<double>() / 180; // [rad]
constexpr double MOVE_STEP = 0.4;          // [m] path interporate resolution
constexpr double COLLISION_CHECK_STEP = 5; // skip number for collision check
constexpr double EXTEND_BOUND = 1;         // collision check range extended
constexpr double GEAR_COST = 100.0;        // switch back penalty cost
constexpr double BACKWARD_COST = 5.0;      // backward penalty cost
constexpr double STEER_CHANGE_COST = 5.0;  // steer angle change penalty cost
constexpr double STEER_ANGLE_COST = 1.0;   // steer angle penalty cost
constexpr double H_COST = 15.0;            // Heuristic cost penalty cost

class Frame
{
public:
  VehicleConfig vc{};
  std::shared_ptr<Para> para_{};
  std::shared_ptr<KDTree> obs{};
};
}; // namespace beacon