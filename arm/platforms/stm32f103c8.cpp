#include "../applications/application.hpp"
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-micromod/micromod.hpp>
namespace sjsu::arm {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();
  static hal::stm32f1::uart uart1(hal::port<1>,
                                  hal::buffer<1024>,
                                  hal::serial::settings{
                                    .baud_rate = 9600,
                                  });
  static hal::stm32f1::output_pin a_high('B', 3);  // INHC
  static hal::stm32f1::output_pin b_high('B', 4);  // INHB
  static hal::stm32f1::pwm a_low('A', 8);          // INLC
  static hal::stm32f1::pwm b_low('A', 0);          // INLB
  static hal::stm32f1::output_pin led('C', 13);

  return {
    .led = &led,
    .console = &uart1,
    .clock = &hal::stm32f1::v1::uptime_clock(),
    .reset = +[]() { hal::stm32f1::v1::reset(); },
    .a_low = &a_low,
    .b_low = &b_low,
    .a_high = &a_high,
    .b_high = &b_high,
  };
}
}  // namespace sjsu::arm