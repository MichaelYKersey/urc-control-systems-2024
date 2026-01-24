#include <drivetrain_math.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "app starting\n");
  auto motor = resources::front_right_steer();
  auto can_transceiver = resources::can_transceiver();
  auto homing_can_finder = hal::can_message_finder(*can_transceiver,0x110);
  auto drive_can_finder = hal::can_message_finder(*can_transceiver,0xC);
  bool send = false;
  int count = 0;
  while (true) {
    if (homing_can_finder.find()) {
      hal::print(*console, "\nHoming command arrived\n");
      send = !send;
      can_transceiver->send({.id=0x111,.length=0});
    }
    auto dv = drive_can_finder.find();
    if (dv) {
      hal::print(*console, "\nDrive arrived\n");
      dv->id=0xD;
      can_transceiver->send(dv.value());
    }
    if (send) {
      motor->velocity_control(0);
      hal::delay(*clock, 10ms);
      if (count %10 == 0) {
        hal::print(*console, "s");
      }
    } else {
      if (count == 0) {  
        hal::print(*console, "r");
      }
    }
    count++;
    count %= 10000;
  }
}
}  // namespace sjsu::drive
