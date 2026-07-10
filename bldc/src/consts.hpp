#ifndef _COSNTS_HPP
#define _COSNTS_HPP
#include "pidff.hpp"

constexpr float max_vel = 600;
constexpr pidff_consts vel_pidff_consts{
  .kP = 0,
  .kI = 0,
  .kD = 0,
  .kS = 0,
  .kV = 0,
  .max_out = 0.7,
  .min_out = -0.7,
};

#endif