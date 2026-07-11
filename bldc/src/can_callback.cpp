#ifndef _CAN_CALLBACK_CPP
#define _CAN_CALLBACK_CPP
#include "can.cpp"
#include "can.hpp"
#include "consts.hpp"
#include "shared_resources.hpp"
#include <cstdint>
#include <libhal/units.hpp>

enum class can_commands : hal::byte
{
  power = 0x00,
  set_velocity = 0x01,
  get_velocity = 0x02,
};

can_commands requested_control_mode = can_commands::power;
bool command_change = false;
float requested_command_value = 0;

static int send_can_reply(hal::byte id, int32_t data)
{
  can2040_msg msg{ .id = can_id + 0x100,
                   .dlc = 5,
                   .data = { id,
                             static_cast<hal::byte>(data >> 24),
                             static_cast<hal::byte>(data >> 16),
                             static_cast<hal::byte>(data >> 8),
                             static_cast<hal::byte>(data) } };
  return can2040_transmit(&canbus, &msg);
}

// CONFIG CAN HERE
void can2040_cb(struct can2040*, uint32_t notify, struct can2040_msg* msg)
{
  if (notify != CAN2040_NOTIFY_RX) {
    return;
  }
  if (msg->id != can_id) {
    return;
  }
  if (msg->dlc != 8) {
    return;
  }
  int32_t value_int = (static_cast<int32_t>(msg->data[4]) << 24) |
                      (static_cast<int32_t>(msg->data[5]) << 16) |
                      (static_cast<int32_t>(msg->data[6]) << 8) |
                      static_cast<int32_t>(msg->data[7]);
  float value_float = static_cast<float>(value_int) / 0x0f'ff'ff'ff;
  hal::byte command = msg->data[0];

  switch (command) {
    case static_cast<hal::byte>(can_commands::power):
    case static_cast<hal::byte>(can_commands::set_velocity):
      command_change =
        (requested_command_value != value_float) ||
        (static_cast<hal::byte>(requested_control_mode) != command);
      requested_control_mode = static_cast<can_commands>(command);
      break;
    case static_cast<hal::byte>(can_commands::get_velocity):
      // TODO: Check that messages aren't missed while tring to send the reply
      send_can_reply(
        static_cast<hal::byte>(can_commands::get_velocity),
        static_cast<int32_t>(shared_angular_vel / max_vel * 0x0f'ff'ff'ff));
      break;
    default:
      break;
  }
}
#endif
