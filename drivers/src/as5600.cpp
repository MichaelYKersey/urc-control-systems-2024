#include <array>
#include <as5600.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

as5600::as5600(hal::v5::strong_ptr<hal::i2c> p_i2c)
  : m_i2c(p_i2c)
{
}

hal::degrees as5600::read_position()
{
  return read_i2c(register_address::raw_angle_high) * (360.0f / (1 << 12));
}

float as5600::read_magnitude()
{
  return read_i2c(register_address::magnitude_high) /
         static_cast<float>(1 << 12);
}

as5600::status as5600::read_status()
{
  hal::byte target_register = static_cast<hal::byte>(register_address::status);
  m_last_register = target_register;
  std::array<hal::byte, 1> write_data = {target_register};
  std::array<hal::byte, 1> read_data = {};
    hal::write_then_read(*m_i2c, device_address, write_data, read_data);
  status s = {
    .magnet_too_strong = static_cast<bool>(read_data[0] & (1 << 5)),
    .magnet_too_weak = static_cast<bool>(read_data[0] & (1 << 4)),
    .magnet_detected = static_cast<bool>(read_data[0] & (1 << 3)),
  };
  return s;
}

hal::u16 as5600::read_i2c(as5600::register_address p_register_address)
{
  hal::byte target_register = static_cast<hal::byte>(p_register_address);
  bool already_at_register =
    m_last_register == static_cast<hal::byte>(p_register_address);
  // update m_last_register (some registers will return after read transaction)
  bool re_readable = false;
  for (auto ra : re_readable_registers) {
    if (static_cast<hal::byte>(ra) == target_register) {
      re_readable = true;
      break;
    }
  }
  m_last_register = static_cast<hal::byte>(p_register_address);
  if (re_readable) {
    m_last_register += 2;
  }

  std::array<hal::byte, 2> read_data = {};
  // drop writeing target register address transaction if last same as last
  if (already_at_register) {
    hal::write_then_read(*m_i2c, device_address, {}, read_data);
  } else {
    std::array<hal::byte, 1> write_data = { target_register };
    hal::write_then_read(*m_i2c, device_address, write_data, read_data);
  }

  hal::u16 result = (static_cast<hal::u16>(read_data[0]) << 8) & read_data[1];
  return result;
}
}  // namespace sjsu::drivers
