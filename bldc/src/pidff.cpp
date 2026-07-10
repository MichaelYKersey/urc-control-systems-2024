#ifndef _PIDFF_CPP
#define _PIDFF_CPP
#include "pidff.hpp"
#include <algorithm>
#include <cmath>

velocity_pidff::velocity_pidff(pidff_consts p_consts,
                               hal::strong_ptr<hal::steady_clock> p_clock)
  : m_consts(p_consts)
  , m_clock(p_clock)
{
}

float velocity_pidff::set_target(float p_target_vel)
{
  m_target_vel = p_target_vel;
  // TODO: looking behaviour when target velocity changes frequently (probably
  // don't want to completely reset it all the time)
  m_error_accumulator = 0;
  if (p_target_vel = 0) {
    m_control_variable = 0;
    return m_control_variable;
  }
  m_control_variable = std::signbit(p_target_vel) ? -m_consts.kS : m_consts.kS;
  m_control_variable += m_consts.kV * p_target_vel;
  m_control_variable =
    std::max(std::min(m_control_variable, m_consts.max_out), m_consts.min_out);
  return m_control_variable;
}

float velocity_pidff::get_target()
{
  return m_target_vel;
}
float velocity_pidff::get_last_output()
{
  return m_control_variable;
}
float velocity_pidff::update(float p_vel_reading)
{
  return update(p_vel_reading, m_clock->uptime());
}
float velocity_pidff::update(float p_vel_reading, hal::u64 p_clock_count)
{
  hal::i64 clock_count_diff = p_clock_count - m_last_clock_count;
  float dt = clock_count_diff / m_clock->frequency();
  m_last_clock_count = p_clock_count;
  // P term
  float error = m_target_vel - p_vel_reading;
  float p_term = m_consts.kP * error;
  // I term
  m_error_accumulator += dt * error;
  m_error_accumulator =
    std::max(std::min(m_error_accumulator, m_consts.max_accumulator),
             m_consts.min_accumulator);
  float i_term = m_consts.kI * m_error_accumulator;
  // D term
  float error_dif = m_last_reading - p_vel_reading;
  float d_term = m_consts.kD * error_dif;
  m_last_reading = p_vel_reading;

  m_control_variable += p_term + i_term + d_term;
  m_control_variable =
    std::max(std::min(m_control_variable, m_consts.max_out), m_consts.min_out);
  return m_control_variable;
}

#endif
