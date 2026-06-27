#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can.cpp"
#include "shared_resources.hpp"
#include "foc_thread.cpp"
#include "micromod-rp2350.cpp"

int main()
{
  using namespace std::chrono_literals;
  namespace rp = hal::rp;
  auto out = resources::console();
  auto clk = resources::clock();
  multicore_launch_core1(&core2);
  hal::print(*out, "Program Start=================\n");

  canbus_setup();
  shared_power_portion = 0.7;
  // float step = 0.1;
  for (;;) {
    hal::print<64>(*out, "Power: %f\n", shared_power_portion);
    hal::print<64>(*out, "Zero Angle: %f\n", shared_zero_angle);
    hal::delay(*clk, 5s);
    shared_power_portion *= -1;
  }
}
