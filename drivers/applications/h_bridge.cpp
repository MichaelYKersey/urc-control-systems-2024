#include <h_bridge.hpp>

namespace sjsu::drivers {
void h_bridge::driver_power(float p_power)
{
  if (power > 0) {
    m_pin_a_low.duty_cycle(p_power);
    m_pin_a_high.level(False);
    m_pin_b_high.level(True);
    m_pin_b_low.duty_cycle(0);

  } else {
    m_pin_a_low.duty_cycle(0);
    m_pin_a_high.level(True);
    m_pin_b_high.level(False);
    m_pin_b_low.duty_cycle((-1) * p_power);
  }
}

}  // namespace sjsu::drivers
