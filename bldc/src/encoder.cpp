#ifndef _ENCODER_CPP
#define _ENCODER_CPP
#include "encoder.hpp"
#include "shared_resources.hpp"
#include <array>
#include <cstdint>
#include <libhal/steady_clock.hpp>

absolute_encoder_accumulator::absolute_encoder_accumulator(
  hal::v5::strong_ptr<hal::i2c> p_i2c,
  hal::v5::strong_ptr<hal::steady_clock> p_clock)
  : m_i2c(p_i2c)
  , m_clock(p_clock)
{
  update_reading();
}

int16_t absolute_encoder_accumulator::read_raw_angle()
{
  std::array<uint8_t const, 1> reg{ 0x0e };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  m_i2c->transaction(i2c_address, reg, data);
  int16_t angle_data_merged = (data[0] << 8) | data[1];
  return angle_data_merged;
}

// TODO: move?
uint16_t get_magnitude(hal::i2c& i2c)
{
  uint8_t const addr = 0x40;
  std::array<uint8_t const, 1> reg{ 0x1b };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t mag_reading = (data[0] << 8) | data[1];
  return mag_reading;
}

void absolute_encoder_accumulator::update_reading()
{
  uint16_t last_raw_encoder_reading = m_encoder_ticks_raw;
  m_encoder_ticks_raw = read_raw_angle();
  m_last_read_timestamp = m_clock->uptime();
  uint16_t angle_dif = m_encoder_ticks_raw - last_raw_encoder_reading;
  if (angle_dif > ticks_per_rotation / 2) {
    angle_dif -= ticks_per_rotation / 2;
  } else if (angle_dif < -ticks_per_rotation / 2) {
    angle_dif += ticks_per_rotation / 2;
  }
  m_cumulative_encoder_tick += angle_dif;
  shared_angular_vel = angle_dif * (360.0f / ticks_per_rotation);
}

int16_t absolute_encoder_accumulator::get_encoder_ticks_raw()
{
  return m_encoder_ticks_raw;
}
long long absolute_encoder_accumulator::get_cumulative_encoder_ticks()
{
  return m_cumulative_encoder_tick;
}
long long absolute_encoder_accumulator::get_last_read_timestamp()
{
  return m_last_read_timestamp;
}
#endif
