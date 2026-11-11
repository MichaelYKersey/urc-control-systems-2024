#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

class as5600
{
public:
  struct status
  {
    bool magnet_too_strong;
    bool magnet_too_weak;
    bool magnet_detected;
  };

  as5600(hal::v5::strong_ptr<hal::i2c> p_i2c);
  hal::degrees read_position();
  float read_magnitude();
  status read_status();

private:
  static constexpr hal::byte device_address = 0x36;

  enum class register_address : hal::byte
  {
    status = 0x0C,
    raw_angle_high = 0x0C,
    raw_angle_low = 0x0D,
    angle_high = 0x0E,
    angle_low = 0x0F,
    magnitude_high = 0x1B,
    magnitude_low = 0x1C,
  };
  static constexpr std::array<register_address, 3> re_readable_registers = {
    register_address::raw_angle_high,
    register_address::magnitude_high,
    register_address::angle_high
  };
  hal::v5::strong_ptr<hal::i2c> m_i2c;
  // default is an invalid register so it must write on first transaction
  hal::byte m_last_register = 0x09;

  hal::u16 read_i2c(register_address p_register_address);
};
}  // namespace sjsu::drivers
