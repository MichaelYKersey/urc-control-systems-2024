#include <drivetrain_math.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console,"\n app starting");
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> prop[] = { resources::front_left_prop(),
                  resources::front_right_prop(),
                  resources::back_left_prop(),
                  resources::back_right_prop() };
  hal::delay(*clock, 1s);
  hal::print(*console,"\n forward");
  for (int i = 0; i < 4; i++) {
    prop[i]->velocity_control(1);
  }
  hal::delay(*clock, 3s);
  hal::print(*console,"\n backward");
  for (int i = 0; i < 4; i++) {
    prop[i]->velocity_control(-1);
  }
  hal::delay(*clock, 3s);hal::print(*console,"\n Starting");
  for (int i = 0; i < 4; i++) {
    prop[i]->velocity_control(0);
  }
}
}  // namespace sjsu::drive
