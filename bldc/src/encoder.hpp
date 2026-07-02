#ifndef _ENCODER_HPP
#define _ENCODER_HPP
#include <cstdint>
#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
class absolute_encoder_accumulator
{
public:
  static constexpr uint16_t ticks_per_rotation = 0x1000;
  static constexpr float ticks_to_rad =
    (std::numbers::pi_v<float> * 2.0f) / ticks_per_rotation;

  absolute_encoder_accumulator(hal::v5::strong_ptr<hal::i2c> p_i2c,
                               hal::v5::strong_ptr<hal::steady_clock> p_clock);

  /**
   * @brief reads absolute encoder and updated accumulator
   *
   * if update_reading() is called after over 180 a rotation has been made
   */
  void update_reading();
  int16_t get_encoder_ticks_raw();
  long long get_cumulative_encoder_ticks();
  long long get_last_read_timestamp();
  int16_t read_raw_angle();

private:
  static constexpr uint8_t i2c_address = 0x40;

  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  int16_t m_encoder_ticks_raw = 0;
  long long m_cumulative_encoder_tick = 0;
  long long m_last_read_timestamp = 0;
};
#endif
