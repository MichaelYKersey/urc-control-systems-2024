#include <libhal-picosdk/serial.hpp>
#include <libhal-picosdk/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can.cpp"
#include "shared_resources.hpp"
#include "foc_thread.cpp"

int main()
{
  using namespace std::chrono_literals;
  namespace rp = hal::rp;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  multicore_launch_core1(&core2);

  canbus_setup();

  for (;;) {
    hal::print<64>(out, "Hello world! Direction: %d\n", dir);
    hal::delay(clk, 1s);
  }
}
