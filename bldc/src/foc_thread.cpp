#ifndef _FOC_THREAD
#define _FOD_THREAD
#include <algorithm>
#include <array>
#include <cmath>
#include <libhal-picosdk/i2c.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include "encoder.cpp"
#include "encoder.hpp"
#include "hbridge.hpp"
#include "micromod-rp2350.cpp"
#include "shared_resources.hpp"

struct timed_encoder_reading {
  long long reading = 0;
  hal::u64 timestamp = 0;
};

std::array<timed_encoder_reading, 256> reading_buffer;
int reading_buffer_index = 0;
// USB stack runs on core 1, so core 2 is picked to reduce jitter
void core2()
{
  triple_hbridge h;
  auto i2c = resources::i2c();
  auto clock = resources::clock();
  absolute_encoder_accumulator aea(i2c, clock);
  long long prev_vel_sample = 0;
  // we align the motor by running a phase for 500ms before reading the zero
  // angle
  h.set_duty(0.85, -0.85, -0.85);
  hal::rp::sleep(std::chrono::duration<hal::u32, std::milli>(500));
  aea.update_reading();
  uint16_t const zero_ticks = aea.get_encoder_ticks_raw();
  h.set_duty(0.0, 0.0, 0.0);
  
  int i = 0;
  float power = shared_power_portion;
  for (;;) {
    // we only check the dir variable once in a while to avoid too much memory
    // contention, although it might be entirely unnecessary
    if (i <= 0) {
      power = shared_power_portion;
      power = std::max(std::min(power, 1.0f), -1.0f);
      i = 1'000;
    }
    i--;

    aea.update_reading();
    float mechanical_angle = (aea.get_encoder_ticks_raw() - zero_ticks) *
                             absolute_encoder_accumulator::ticks_to_rad;
    float electrical_angle = mechanical_angle * 14.0f;
    constexpr float quad_offset = std::numbers::pi_v<float> / 2.f;
    float quadrature = electrical_angle + quad_offset;
    float const offset = std::numbers::pi_v<float> * 2.0f / 3.0f;
    float max_duty = power;
    float a = max_duty * cosf(quadrature),
          b = max_duty * cosf(quadrature - offset),
          c = max_duty * cosf(quadrature + offset);
    h.set_duty(a, b, c);

    // update vel
    timed_encoder_reading current_reading = {
      .reading = aea.get_cumulative_encoder_ticks(),
      .timestamp = aea.get_last_read_timestamp(),
    };
    shared_angle = current_reading.reading * absolute_encoder_accumulator::ticks_to_rad;
    timed_encoder_reading oldest_reading = reading_buffer[reading_buffer_index];
    float angle_diff = (current_reading.reading - prev_vel_sample) * absolute_encoder_accumulator::ticks_to_rad;
    float dt = (current_reading.timestamp - oldest_reading.timestamp) / clock->frequency();
    shared_angular_vel = angle_diff / dt;
    reading_buffer[reading_buffer_index] = current_reading;
    reading_buffer_index = (reading_buffer_index + 1) % reading_buffer.size();
  }
}
#endif
