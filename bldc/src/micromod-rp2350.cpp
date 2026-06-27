#ifndef _MICROMOD_RP2350
#define _MICROMOD_RP2350
#include <libhal-picosdk/i2c.hpp>
#include <libhal-picosdk/serial.hpp>
#include <libhal-picosdk/time.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

namespace resources {

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 4096> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

hal::v5::strong_ptr<hal::steady_clock> clock()
{
  static auto clock_ptr =
    hal::v5::make_strong_ptr<hal::rp::clock>(driver_allocator());
  return clock_ptr;
}

hal::strong_ptr<hal::serial> console()
{
  static auto console_ptr =
    hal::v5::make_strong_ptr<hal::rp::stdio_serial>(driver_allocator());
  return console_ptr;
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  // i2c needs to be run at very high speed or the FOC loop gets bogged down by i2c
  auto settings = hal::i2c::settings(1'000'000 );
  static auto i2c_ptr =
    hal::v5::make_strong_ptr<hal::rp::i2c>(driver_allocator(),
                                           hal::pin<16>,
                                           hal::pin<17>,
                                           hal::bus<0>,
                                           settings);
  return i2c_ptr;
}

}  // namespace resources
#endif
