#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can.cpp"
#include "foc_thread.cpp"
#include "micromod-rp2350.cpp"
#include "shared_resources.hpp"

int main()
{
  using namespace std::chrono_literals;
  auto out = resources::console();
  auto clk = resources::clock();
  multicore_launch_core1(&core2);
  hal::print(*out, "Program Start=================\n");

  canbus_setup();
  shared_power_portion = 0.7;
  // float step = 0.1;
  for (;;) {
    // hal::print<64>(*out, "Power: %f\n", shared_power_portion);
    float angle = shared_angle;
    float vel = shared_angular_vel;
    hal::print<64>(*out, "Angle: %f\t", angle);
    hal::print<64>(*out, "velocity: %f\t", vel);
    // hal::print<64>(*out, "Zero Angle: %f\t", shared_zero_angle * 360.0f / (2
    // * std::numbers::pi_v<float>));
    hal::print(*out, "\n");
    hal::delay(*clk, 250ms);
    // shared_power_portion *= -1;
  }
}
