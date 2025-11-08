#include <array>
#include <bit>
#include <cstdint>
#include <libhal-util/bit.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <sys/_types.h>
#include <sys/types.h>

namespace {
enum class can_message_id : uint32_t
{
  set_chassis_velocities,
  set_velocities_Response,
  heartbeat,
  heartbeat_reply,
  homing_sequence,
  reply_homing_sequence,
  get_offset,
  return_offset,
  get_Estimated_velocities,
  return_estimated_chassis_velocities,
  config,
  Config_ack,
};
}

namespace sjsu::drive {

mission_control_manager::mission_control_manager(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver)
  : m_can_transceiver(p_can_transceiver)
  , m_heartbeat_message_finder(
      hal::can_message_finder(*m_can_transceiver, 0x00))
  , m_set_chassis_velocities_message_finder(
      hal::can_message_finder(*m_can_transceiver, 0x00))
  , m_get_chassis_velocities_message_finder(
      hal::can_message_finder(*m_can_transceiver, 0x00))
  , m_homing_request_message_finder(
      hal::can_message_finder(*m_can_transceiver, 0x00))
  , m_get_offset_message_finder(
      hal::can_message_finder(*m_can_transceiver, 0x00))
{
}

float fixed_point_to_float(int16_t p_fixed_point_num, int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  float floating_num = p_fixed_point_num;
  if (p_expo < 0) {
    return floating_num / shifted_expo;
  } else {
    return floating_num * shifted_expo;
  }
}
int16_t float_to_fixed_point(float p_floating_point_num, int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  if (p_expo > 0) {
    p_floating_point_num /= shifted_expo;
  } else {
    p_floating_point_num *= shifted_expo;
  }
  // TODO: consider implementing rounding
  return p_floating_point_num;
}

int16_t array_to_int16(std::array<hal::byte, 2> const& p_array);
std::array<hal::byte, 2> int16_to_byte_array(int16_t p_num)
{
  uint16_t unum = std::bit_cast<uint16_t>(p_num);
  std::array<hal::byte, 2> byte_array = { static_cast<uint8_t>(unum >> 8),
                                          static_cast<uint8_t>(unum) };
  return byte_array;
}

// returns most recent velocity request
// std::optional<chassis_velocities_request>
// mission_control_manager::read_set_velocity_request();
// void mission_control_manager::reply_set_velocity_reques?t(
//   chassis_velocities_request const& p_chassis_vel);

// return true if homing requested;
// bool mission_control_manager::read_homing_request();
// void mission_control_manager::reply_homing_request();

void mission_control_manager::fulfill_data_requests(
  drivetrain const& p_drivetrain)
{
  // handle offset requests
  hal::byte offsets_requested = 0;
  std::optional<hal::can_message> offset_request_message;
  do {
    offset_request_message = m_get_offset_message_finder.find();
    if (offset_request_message) {
      offsets_requested |= 1 << offset_request_message->payload[0];
    }
  } while (offset_request_message);
  hal::byte i = 0;
  while (offsets_requested >> i) {
    if ((offsets_requested >> i) & 1) {
      // send message
      std::array<hal::byte, 2> offset_bytes =
        int16_to_byte_array(p_drivetrain.get_steer_offset(i));
      hal::can_message
        offset_request_reply = { .id = static_cast<uint32_t>(
                                   can_message_id::heartbeat_reply),
                                 .length = 3,
                                 .payload = {
                                   offset_bytes[0],
                                   offset_bytes[1],
                                   i,
                                 } };
      m_can_transceiver->send(offset_request_reply);
    }
    i++;
  }
}

// void mission_control_manager::clear_heartbeat_requests();
// void mission_control_manager::reply_heartbeat();

}  // namespace sjsu::drive