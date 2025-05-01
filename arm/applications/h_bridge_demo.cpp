#include <application.hpp>
#include <h_bridge.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::arm {
void application(hardware_map_t& p_resources)
{

  auto& a_high = *p_resources.a_high;
  auto& b_high = *p_resources.b_high;
  auto& a_low = *p_resources.a_low;
  auto& b_low = *p_resources.b_low;
  auto& clock = *p_resources.clock;
  // auto& terminal = *p_resources.terminal;
  auto m_h_bridge = drivers::h_bridge(a_low, b_low, a_high, b_high);

  while (true) {
    m_h_bridge.power(0.1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(0.4);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(0.7);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.4);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.7);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-1);
    hal::delay(clock, 2000ms);

    m_h_bridge.power(0);
    hal::delay(clock, 2000ms);
  }
}
}  // namespace sjsu::arm