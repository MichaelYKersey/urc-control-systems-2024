#ifndef _CAN_CALLBACK_CPP
#define _CAN_CALLBACK_CPP
#include "can.cpp"
#include <cstdint>
#include <libhal/units.hpp>

enum class can_commands : hal::byte
{
  power = 0x00,
  set_velocity = 0x01,
};
can_commands requested_control_mode = can_commands::power;
bool command_change = false;
float requested_command_value = 0;

// CONFIG CAN HERE
void can2040_cb(struct can2040*, uint32_t notify, struct can2040_msg* msg)
{
  if (notify != CAN2040_NOTIFY_RX) {
    return;
  }
  if (msg->id != 0x15) {
    return;
  }
  if (msg->dlc != 8) {
    return;
  }
  uint32_t value_unsigned = (msg->data[4] << 24) | (msg->data[5] << 16) |
                            (msg->data[6] << 8) | msg->data[7];
  float value_float =
    static_cast<float>(static_cast<int32_t>(value_unsigned)) / 0x0f'ff'ff'ff;
  hal::byte command = msg->data[0];
  switch (command) {
    case static_cast<hal::byte>(can_commands::power):
    case static_cast<hal::byte>(can_commands::set_velocity):
      command_change =
        (requested_command_value != value_float) ||
        (static_cast<hal::byte>(requested_control_mode) != command);
      requested_control_mode = static_cast<can_commands>(command);
      break;
    default:
      break;
  }
}
#endif
