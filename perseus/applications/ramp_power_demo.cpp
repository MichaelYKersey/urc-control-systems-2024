#include "can_commands.hpp"
#include <bldc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();

  auto h_bridge = resources::h_bridge();

  hal::print(*console, "ramp start\n");

  uint64_t deadline;
  for (int i=15; i<=100; i++) {
    h_bridge->power(0.01*i);
    hal::print<64>(*console,"Power:%d\n",i);
    deadline = hal::future_deadline(*clock, 1000ms);
    while (deadline > clock->uptime()) {
    }
  }
  hal::delay(*clock, 10s);
  for (int i=100; i>=15; i--) {
    h_bridge->power(0.01*i);
    hal::print<64>(*console,"Power:%d\n",i);
    deadline = hal::future_deadline(*clock, 1000ms);
    while (deadline > clock->uptime()) {
    }
  }
}

};  // namespace sjsu::perseus
