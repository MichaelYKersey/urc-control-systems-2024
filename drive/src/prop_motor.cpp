#include <libhal/can.hpp>
#include <optional>
#include <prop_motor.hpp>

namespace sjsu::drive {
prop_motor::prop_motor(hal::strong_ptr<hal::v5::can_transceiver> p_can,
                       uint16_t p_id)
  : m_can(p_can)
  , m_can_message_finder(hal::can_message_finder(*p_can, p_id + 0x100))
  , m_id(p_id)
{
}
void prop_motor::driver_enable(bool p_state)
{
  if (!p_state) {
    hal::can_message message{
      .id = m_id,
      .length = 8,
      .payload = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    m_can->send(message);
  }
}
void prop_motor::driver_drive(hal::rpm p_velocity)
{
  int32_t int_velocity =
    static_cast<int32_t>(p_velocity / driver_velocity_range().max *
                         static_cast<float>(0x0f'ff'ff'ff));
  hal::can_message message{ .id = m_id,
                            .length = 8,
                            .payload = {
                              0x00,
                              0x00,
                              0x00,
                              0x00,
                              static_cast<hal::byte>(int_velocity >> 24),
                              static_cast<hal::byte>(int_velocity >> 16),
                              static_cast<hal::byte>(int_velocity >> 8),
                              static_cast<hal::byte>(int_velocity) } };
  m_can->send(message);
}
hal::velocity_motor::status_t prop_motor::driver_status()
{
  hal::can_message message{
    .id = m_id,
    .length = 8,
    .payload = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
  };
  m_can->send(message);
  // TODO: add timeout
  std::optional<hal::can_message> reply = std::nullopt;
  while (!reply) {
    reply = m_can_message_finder.find();
  }
  int32_t velocity_int =
    (static_cast<int32_t>(reply.value().payload[4]) << 24) |
    (static_cast<int32_t>(reply.value().payload[5]) << 16) |
    (static_cast<int32_t>(reply.value().payload[6]) << 8) |
    static_cast<int32_t>(reply.value().payload[7]);
  float velocity_float = static_cast<float>(velocity_int) /
                         static_cast<float>(0x0f'ff'ff'ff) *
                         velocity_range().max;
  return { .velocity = velocity_float };
}
hal::velocity_motor::range_t prop_motor::driver_velocity_range()
{
  return { .max = 0 };
}

}  // namespace sjsu::drive
