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

  hal::print(*console, "start loop\n");

  uint64_t deadline;
  while (true) {
    hal::print(*console,"power: 0.3\n");
    h_bridge->power(0.3);
    deadline = hal::future_deadline(*clock, 10s);
    while (deadline > clock->uptime()) {
    }

    hal::print(*console,"power: -0.3\n");
    h_bridge->power(-0.3);
    deadline = hal::future_deadline(*clock, 10s);
    while (deadline > clock->uptime()) {
    }
  }
}

};  // namespace sjsu::perseus
