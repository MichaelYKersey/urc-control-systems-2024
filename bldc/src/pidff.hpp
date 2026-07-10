#ifndef _PIDFF_HPP
#define _PIDFF_HPP
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>

struct pidff_consts
{
  float kP = 0, kI = 0, kD = 0, kS = 0, kV = 0;
  float max_accumulator = std::numeric_limits<float>::infinity();
  float min_accumulator = -std::numeric_limits<float>::infinity();
  float max_out = 0, min_out = 0;
};

class velocity_pidff
{ 
public:
  velocity_pidff(pidff_consts p_consts, hal::strong_ptr<hal::steady_clock> p_clock);

  float set_target(float p_target_vel);
  float get_target();
  float get_last_output();
  float update(float p_vel_reading);
  float update(float p_vel_reading, hal::u64 p_clock_count);

private:
  pidff_consts m_consts;
  hal::strong_ptr<hal::steady_clock> m_clock;
  
  float m_target_vel = 0;
  float m_last_reading = 0;
  float m_error_accumulator = 0;
  hal::u64 m_last_clock_count = 0;
  float m_control_variable = 0;
};
#endif
