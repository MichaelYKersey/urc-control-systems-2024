#include "swerve_module.hpp"
#include "vector2d.hpp"
#include <drivetrain.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;
  constexpr sec refresh_rate = 5;  // 5s to ignore interpolation (or interpolate until next update)
  constexpr auto refresh_rate_chrono = 5s;
  drivetrain dt(resources::swerve_modules(), refresh_rate);
  auto clock = resources::clock();

  dt.set_target_state(chassis_velocities(vector2d(0,0),1), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(1,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,1),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,1),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(1,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,0),1), true);
  hal::delay(*clock, refresh_rate_chrono);
  dt.set_target_state(chassis_velocities(vector2d(0,0),0), true);
  hal::delay(*clock, refresh_rate_chrono);
  // resources::reset();
  // each loop:
  // -if stop message stop then stop drive
  // -if respond to heartbeat
  // -if homing stop drive and run homing sequence (make interuptable by MC to
  // cancel) -else update target chassis value if needed run periodic to keep
  // drivetrain running smoothly -return any readings requested by MC
}
}  // namespace sjsu::drive
