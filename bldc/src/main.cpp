#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can_callback.cpp"
#include "foc_thread.cpp"
#include "micromod-rp2350.cpp"
#include "pidff.cpp"
#include "pidff.hpp"
#include "shared_resources.hpp"
#include "consts.hpp"

int main()
{
  using namespace std::chrono_literals;
  auto out = resources::console();
  auto clk = resources::clock();
  multicore_launch_core1(&core2);
  hal::print(*out, "Program Start=================\n");
  velocity_pidff pid(vel_pidff_consts, clk);
  can_commands current_control_mode = can_commands::power;

  canbus_setup();
  // float step = 0.1;
  int i = 0;
  for (;;) {
    hal::delay(*clk, 100us);
    float angle = shared_angle;
    float vel = shared_angular_vel;
    float power = 0;
    // group reading to minimize chance of tearing
    can_commands control_mode = requested_control_mode;
    float command_value = requested_command_value;

    // execute mode
    bool command_change = current_control_mode != control_mode;
    current_control_mode = control_mode;
    switch (requested_control_mode) {
      case can_commands::power:
          power = command_value;
        break;
      case can_commands::set_velocity:
        if (command_change || pid.get_target() != command_value * max_vel) {
          pid.set_target(command_value * max_vel);
        }
        power = pid.update(vel);
        break;
      default:
        break;
    }
    shared_power_portion = power;
    // prints
    if (i == 0) {
      i = 5000;
      // hal::print<64>(*out, "Zero Angle (deg): %f\t", shared_zero_angle *
      // 360.0f / (2
      // * std::numbers::pi_v<float>));
      hal::print<64>(*out, "Angle: %f\t", angle);
      hal::print<64>(*out, "velocity: %f\t", vel);
      hal::print<64>(*out, "Power: %f\t", power);
      hal::print(*out, "\n");
      i--;
    }
  }
}
