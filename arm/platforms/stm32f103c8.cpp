#include "../applications/application.hpp"
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
namespace sjsu::arm {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;

  static hal::stm32f1::uart uart1(hal::port<1>,
                                  hal::buffer<1024>,
                                  hal::serial::settings{
                                    .baud_rate = 9600,
                                  });
  static hal::stm32f1::output_pin p_a_high('B', 3);  // INHC
  static hal::stm32f1::output_pin p_b_high('B', 4);  // INHB
  static hal::stm32f1::pwm p_a_low('A', 8);          // INLC
  static hal::stm32f1::pwm p_b_low('A', 0);          // INLB
  hal::pwm16_channel* pwm_channel = nullptr;
  hal::pwm_group_manager* pwm_frequency = nullptr;
  static hal::stm32f1::output_pin led('C', 13);
  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();
  return {
    .led = &led,
    .console = &uart1,
    .clock = &steady_clock,
    .reset = +[]() { hal::cortex_m::reset(); },
    .a_low = &p_a_low,
    .b_low = &p_b_low,
    .a_high = &p_a_high,
    .b_high = &p_b_high,
  };
}
}  // namespace sjsu::arm