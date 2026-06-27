#ifndef _FOC_THREAD
#define _FOD_THREAD
#include <algorithm>
#include <libhal-picosdk/i2c.hpp>
#include <libhal/units.hpp>

#include "hbridge.hpp"
#include "shared_resources.hpp"
#include "micromod-rp2350.cpp"

using namespace std::chrono_literals;

float prev_angle = 0;
long long prev_angle_clock_count = 0;
// AS5600L angle reading, not very much to comment on here
static float get_angle(hal::i2c& i2c)
{
  uint8_t const addr = 0x40;
  std::array<uint8_t const, 1> reg{ 0x0e };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t angle_data_merged = (data[0] << 8) | data[1];
  float angle = (float(angle_data_merged) / float(0xfff) * 2.0f * std::numbers::pi_v<float>);

  return angle;
}

uint16_t get_magnitude(hal::i2c& i2c) {
  uint8_t const addr = 0x40;
  std::array<uint8_t const, 1> reg{ 0x1b };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t mag_reading = (data[0] << 8) | data[1];
  return mag_reading;
}

// USB stack runs on core 1, so core 2 is picked to reduce jitter
void core2()
{
  triple_hbridge h;
  auto i2c = resources::i2c();

  // we align the motor by running a phase for 500ms before reading the zero
  // angle
  h.set_duty(0.85, -0.85, -0.85);
  hal::rp::sleep(std::chrono::duration<hal::u32, std::milli>(500));
  h.set_duty(0.0, 0.0, 0.0);
  float const zero_angle = get_angle(*i2c);
  shared_zero_angle = zero_angle;

  int i = 0;
  float power = shared_power_portion;
  for (;;) {
    // we only check the dir variable once in a while to avoid too much memory
    // contention, although it might be entirely unnecessary
    if (i <= 0) {
      power = shared_power_portion;
      power = std::max(std::min(power,0.7f),-0.7f);
      i = 1'000;
    }
    i--;
    // See Umeda et al.
    float mechanical_angle = (get_angle(*i2c) - zero_angle);
    float electrical_angle = mechanical_angle * 14.0f;
    float quad_offset = std::numbers::pi_v<float> / 2.f;
    float quadrature = electrical_angle + quad_offset;
    float const offset = std::numbers::pi_v<float> * 2.0f / 3.0f;
    float max_duty = power;
    float a = max_duty * cosf(quadrature),
          b = max_duty * cosf(quadrature - offset),
          c = max_duty * cosf(quadrature + offset);
    h.set_duty(a, b, c);
  }
}
#endif