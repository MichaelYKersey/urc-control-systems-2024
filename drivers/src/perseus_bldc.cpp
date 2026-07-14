#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <optional>
#include <perseus_bldc.hpp>
namespace {
enum class perseus_command_ids : hal::byte
{
  shut_down = 0x0C,
  heartbeat,
  homing,
  set_target_position,
  get_target_position,
  set_encoder_position,
  get_encoder_position,
  set_target_velocity,
  get_target_velocity,
  set_power,
  get_power,
  set_position_pid_consts,
  get_position_pid_consts,
  set_velocity_pid_consts,
  get_velocity_pid_consts,
};
};

namespace sjsu::drivers {

// TODO: implement unimplemented functions

perseus_bldc::perseus_bldc(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::u32 p_can_id,
  hal::time_duration p_max_response_time)
  : m_can_transceiver(p_can_transceiver)
  , m_clock(p_clock)
  , m_can_id(p_can_id)
  , m_reply_message_finder(hal::can_message_finder(*p_can_transceiver, p_can_id))
  , m_max_response_time(p_max_response_time)
{
  // throw hal::operation_not_supported(this);
}

void perseus_bldc::kill_power()
{
  // throw hal::operation_not_supported(this);
}
void perseus_bldc::heart_beat()
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::home()
{
  // throw hal::operation_not_supported(this);
}
bool perseus_bldc::is_homing()
{
  // throw hal::operation_not_supported(this);
}
void perseus_bldc::set_target_position(hal::degrees p_target_position)
{
  // throw hal::operation_not_supported(this);
}
bool perseus_bldc::hit_target_position()
{
  // throw hal::operation_not_supported(this);
}
void perseus_bldc::set_encoder_position(hal::degrees p_position)
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_target_velocity(hal::rpm p_target_velocity)
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_power(float p_portion)
{
  // throw hal::operation_not_supported(this);
}
void perseus_bldc::set_position_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_velocity_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
hal::degrees perseus_bldc::get_target_position()
{
  // throw hal::operation_not_supported(this);
}
hal::degrees perseus_bldc::get_position()
{
  // throw hal::operation_not_supported(this);
}
hal::rpm perseus_bldc::get_velocity()
{
  throw hal::operation_not_supported(this);
}
float perseus_bldc::read_power()
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::get_position_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::get_velocity_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}

std::optional<hal::can_message> perseus_bldc::send(std::array<hal::byte, 8> const& p_payload,
                        hal::byte const& length)
{
  hal::can_message const payload{
    .id = m_can_id,
    .length = length,
    .payload = p_payload,
  };

  // Send payload
  m_can_transceiver->send(payload);
  // Wait for reply in case of time out
  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (deadline > m_clock->uptime()) {
    auto const message = m_reply_message_finder.find();
    if (message.has_value()) {
      return message;
    }
  }
  hal::safe_throw(hal::timed_out(this));
}

};  // namespace sjsu::drivers
