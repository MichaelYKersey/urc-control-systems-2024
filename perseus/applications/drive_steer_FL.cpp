// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
#include <resource_list.hpp>

#include "perseus_can_demo.cpp"


using namespace std::chrono_literals;
namespace sjsu::perseus {

void application()
{
  constexpr hal::u16 allowed_id = 0x03; 
  // pid
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.02, 
    .ki = 0.00, 
    .kd = 0.005,
  };
  // servo_values 
  bldc_perseus::servo_values servo_values = {
    .gear_ratio = 260/108, // 5281.1 * 2 / 2
    .high_clamped_value = 0.3, 
    .low_clamped_value = -0.3 
  };  
  can_app_bootstrap(allowed_id, pid_settings, servo_values);
}
}  // namespace sjsu::perseus