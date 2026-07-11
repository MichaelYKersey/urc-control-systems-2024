#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>

namespace sjsu::drive {

class prop_motor : hal::velocity_motor
{
public:
  prop_motor(hal::strong_ptr<hal::v5::can_transceiver> p_can, uint16_t p_id);

  virtual void driver_enable(bool p_state);
  virtual void driver_drive(hal::rpm p_velocity);
  virtual status_t driver_status();
  virtual range_t driver_velocity_range();
private:
  hal::strong_ptr<hal::v5::can_transceiver> m_can;
  hal::can_message_finder m_can_message_finder;
  uint16_t m_id;
};

class prop_servo : hal::velocity_servo
{
  // TODO
  // virtual void driver_enable(bool p_state);
  // virtual void driver_position(degrees p_target_position);
  // virtual position_range_t driver_position_range();
  // virtual void driver_configure(settings const& p_settings);
  // virtual status_t driver_status();
  // virtual range_t driver_velocity_range();
  // virtual degrees driver_get_position();
  // virtual bool driver_is_moving();
};
}  // namespace sjsu::drive
