#include <h_bridge.hpp>

namespace sjsu::drivers {
void h_bridge::driver_power(float p_power)
{
  if (power < 0) {
    m_pin_b_low.duty_cycle(0);
    m_pin_a_high.level(false);
    m_pin_b_high.level(true);
    m_pin_a_low.duty_cycle((-1) * power);
  } else {
    m_pin_a_low.duty_cycle(0);
    m_pin_b_high.level(false);
    m_pin_b_low.duty_cycle(power);
    m_pin_a_high.level(true);
  }
}
}  // namespace sjsu::drivers
