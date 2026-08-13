#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <resource_list.hpp>
#include <swerve_module.hpp>

namespace sjsu::drive {

void application()
{
  auto console = resources::console();
  auto clock = resources::clock();
  auto can_transceiver = resources::can_transceiver();
  auto can_fillter = resources::get_new_can_filter();
  hal::actuator::rmd_drc_v2 motor(
    *can_transceiver,
    *can_fillter,
    *clock,
    1.0f,
    0x141);

  while (true) {
    motor.velocity_control(100);
    hal::print<64>(*console,"vel:%f\n",motor.feedback().speed());
    hal::delay(*clock, 1s);
  }
}
}  // namespace sjsu::drive
