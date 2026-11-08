#include "bldc_servo.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo_settings.hpp>
#include <can_util.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // general
  auto clock = resources::clock();
  auto console = resources::console();

  // variables used to initialize bldc_perseus
  bldc_perseus::pid_settings pid_settings_position;
  bldc_perseus::pid_settings pid_settings_velocity = { .kp = 0.0001,
                                                       .ki = 0.000,
                                                       .kd = 0.01 };
  bldc_perseus::physical_servo_values physical_servo_values;
  // variables used to initialize can_perseus
  can_perseus::servo_address allowed_id;
  hal::u16 listen_prev = 0x00;
  auto switches_ptr = resources::switches();
  allowed_id = static_cast<can_perseus::servo_address>(
    static_cast<hal::u16>(switches_ptr->read_switch_value()) + 0x120);
  hal::u16 group_id = 0x120;
  // set servo values according to switch
  switch (allowed_id) {
    case can_perseus::track_servo:
      pid_settings_position = track_pid_settings;
      physical_servo_values = track_physical_servo_values;
      break;
    case can_perseus::shoulder_servo:
      pid_settings_position = shoulder_pid_settings;
      physical_servo_values = shoulder_physical_servo_values;
      break;
    case can_perseus::elbow_servo:
      pid_settings_position = elbow_pid_settings;
      physical_servo_values = elbow_physical_servo_values;
      listen_prev = can_perseus::shoulder_servo;
      break;
    case can_perseus::wrist_left:
      pid_settings_position = wrist_left_pid_settings;
      physical_servo_values = wrist_left_physical_servo_values;
      listen_prev = can_perseus::elbow_servo;
      break;
    case can_perseus::wrist_right:
      pid_settings_position = wrist_right_pid_settings;
      physical_servo_values = wrist_right_physical_servo_values;
      listen_prev = can_perseus::elbow_servo;
      break;
    case can_perseus::end_effector:
      pid_settings_position = end_effector_pid_settings;
      physical_servo_values = end_effector_physical_servo_values;
      break;
    default:
      hal::print(*console, "Address does not exist. Exiting.\n");
      return;
  }

  // create bldc_perseus
  auto bldc =
    bldc_perseus(resources::h_bridge(), resources::encoder(), resources::clock(), resources::homing_pin());
  bldc.update_pid_position(pid_settings_position);
  bldc.update_pid_velocity(pid_settings_velocity);
  bldc.set_physical_servo_values(physical_servo_values);
  bldc.get_actual_position();
  hal::print(*console, "pre-can\n");

  // create can_perseus
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::get_new_can_filter();
  auto can_group_filter = resources::get_new_can_filter();
  can_perseus servo_can(allowed_id,
                        listen_prev,
                        group_id,
                        1_MHz,
                        can_transceiver,
                        can_bus_manager,
                        can_id_filter,
                        can_group_filter);

  hal::print(*console, "Begin.\n");

  // start loop

  while (true) {

    // receive message
    std::optional<hal::can_message> msg =
      servo_can.check_for_individual_message();

    // react to message
    if (msg) {
      drivers::can_util::print_can_message(*console, *msg);
      servo_can.process_can_message(*msg, bldc);
      hal::print<64>(*console, "Mode: %x \n", bldc.get_active_mode());
    }

    bldc.periodic();

    // check group message message
    msg = servo_can.check_for_group_message();

    // react to message
    if (msg) {
      drivers::can_util::print_can_message(*console, *msg);
      servo_can.process_can_message(*msg, bldc);
      hal::print<64>(*console, "Mode: %x \n", bldc.get_active_mode());
    }
  }
}
}  // namespace sjsu::perseus
