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

  auto homing_limitswitch = resources::homing_pin();

  bool state = !homing_limitswitch->level();
  while (true) {
    bool new_state = homing_limitswitch->level(); 
    if (state != new_state) {
      state = new_state;
      hal::delay(*clock, 100ms);
    } else {
      continue;
    }
    if (state) {
      hal::print(*console, "pressed\n");
    } else {
      hal::print(*console, "not pressed\n");
    }
  }
}

};  // namespace sjsu::perseus
