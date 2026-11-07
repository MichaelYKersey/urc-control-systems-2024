#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo_settings.hpp>
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
  bldc_perseus::PID_settings pid_settings_position;
  bldc_perseus::PID_settings pid_settings_velocity = { .kp = 0.0001,
                                                       .ki = 0.000,
                                                       .kd = 0.01 };
  bldc_perseus::physical_servo_values physical_servo_values;
  // variables used to initialize can_perseus
  auto switches_ptr = resources::switches();
  hal::u16 allowed_id = switches_ptr->read_switch_value() + 0x120;
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
      break;
    case can_perseus::wrist_left:
      pid_settings_position = wrist_left_pid_settings;
      physical_servo_values = wrist_left_physical_servo_values;
      break;
    case can_perseus::wrist_right:
      pid_settings_position = wrist_right_pid_settings;
      physical_servo_values = wrist_right_physical_servo_values;
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
    bldc_perseus(resources::h_bridge(), resources::encoder(), allowed_id);
  bldc.update_pid_position(pid_settings_position);
  bldc.update_pid_velocity(pid_settings_velocity);
  bldc.set_physical_servo_values(physical_servo_values);
  bldc.get_actual_position();

  hal::print(*console, "Begin.\n");

  bldc.home_encoder();
  hal::print<64>(*console, "%f", bldc.get_actual_position());
}
}  // namespace sjsu::perseus
