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

  bldc_perseus::physical_servo_values physical_servo_values = {
    .gear_ratio = 188.0f / 1.0f,
    .angle_offset = 0.0,
    .fight_gravity = 0.0,
    .high_clamped_value = 0.5,
    .low_clamped_value = -0.5,
    .clockwise_positive = false,
    .position_min = -std::numeric_limits<float>::infinity(),
    .position_max = std::numeric_limits<float>::infinity(),
    .homing_limitswitch_position = NAN,
    .homing_power = 0.0f
  };
  bldc_perseus::pid_settings pid_settings = { .kp = 0.001, .ki = 0, .kd = 0 };
  auto servo_ptr = resources::servo();
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_physical_servo_values(physical_servo_values);
  servo_ptr->set_active_action(action::set_position_target);

  hal::print(*console, "start loop\n");

  uint64_t deadline;
  while (true) {
    // servo_ptr->set_power(0.3);
    hal::print(*console, "pos 0\n");
    servo_ptr->set_target_position(0);
    servo_ptr->periodic_action(true);
    deadline = hal::future_deadline(*clock, 10s);
    while (deadline > clock->uptime()) {
      servo_ptr->periodic_action(false);
    }

    // servo_ptr->set_power(-0.3);
    hal::print(*console, "pos 360\n");
    servo_ptr->set_target_position(360);
    servo_ptr->periodic_action(true);
    deadline = hal::future_deadline(*clock, 10s);
    while (deadline > clock->uptime()) {
      servo_ptr->periodic_action(false);
    }
  }
}

};  // namespace sjsu::perseus
