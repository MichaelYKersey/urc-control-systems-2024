#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can.cpp"
#include "foc_thread.cpp"
#include "micromod-rp2350.cpp"
#include "pidff.hpp"
#include "shared_resources.hpp"
#include "pidff.cpp"

int main()
{
  using namespace std::chrono_literals;
  auto out = resources::console();
  auto clk = resources::clock();
  multicore_launch_core1(&core2);
  hal::print(*out, "Program Start=================\n");
  pidff_consts vel_pidff_consts{
    .kP = 0,
    .kI = 0,
    .kD = 0,
    .kS = 0,
    .kV = 0,
    .max_out = 0.7,
    .min_out = -0.7,
  };
  velocity_pidff pid(vel_pidff_consts,clk);
  pid.set_target(600.0f);
  
  canbus_setup();
  // float step = 0.1;
  int i = 0;
  for (;;) {
    hal::delay(*clk, 100us);
    float angle = shared_angle;
    float vel = shared_angular_vel;
    float power  = pid.update(vel);
    shared_power_portion = power;
    if (i == 0) {
      i = 5000;
      // hal::print<64>(*out, "Zero Angle (deg): %f\t", shared_zero_angle * 360.0f / (2
      // * std::numbers::pi_v<float>));
      hal::print<64>(*out, "Angle: %f\t", angle);
      hal::print<64>(*out, "velocity: %f\t", vel);
      hal::print<64>(*out, "Power: %f\t", power);
      hal::print(*out, "\n");
    }
  }
}
