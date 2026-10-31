#pragma  once
#include <cstdint>
namespace sjsu::perseus {

enum class action : uint8_t
{
  // top priority
  power_off_reset = 0x0C,
  heartbeat = 0x0E,

  // actuators
  homing = 0x11,
  set_position_target = 0x12,
  set_position_reading = 0x13,
  set_velocity_target = 0x14,
  set_power = 0x16,
  set_pid_position_config = 0x17,
  set_pid_velocity_config = 0x18,

  // readers
  read_homing_status = 0x21,
  read_position_target = 0x22,
  read_position_reading = 0x23,
  read_velocity_target = 0x24,
  read_velocity_reading = 0x25,
  read_power = 0x26,
  read_pid_position_config = 0x27,
  read_pid_velocity_config = 0x28,

  // servo to servo
  prev_joint_actual_position = 0x41,
  prev_joint_position_response = 0x51,
};
}
