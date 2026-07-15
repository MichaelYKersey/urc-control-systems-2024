#pragma once

#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <resource_list.hpp>

namespace sjsu::drivers::can_util {

// decode message from mission control
consteval float fixed_to_floating_point_32(hal::byte b0,
                                              hal::byte b1,
                                              hal::byte b2,
                                              hal::byte b3,
                                              int exponent)
{
  // auto console = resources::console();
  hal::i32 initial = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
  // hal::print<64>(*console, "pure_binary: %x -- ", initial);
  float shifted = initial / powf(2, exponent);
  // hal::print<64>(*console, "float cast: %f\n", shifted);
  return shifted;
}
consteval float fixed_to_floating_point_16(hal::byte b0,
                                              hal::byte b1,
                                              int exponent)
{
  // auto console = resources::console();
  hal::i32 initial = (b0 << 8) | b1;
  // hal::print<64>(*console, "pure_binary: %x -- ", initial);
  float shifted = initial / powf(2, exponent);
  // hal::print<64>(*console, "float cast: %f\n", shifted);
  return shifted;
}
// endcode message to mission control
consteval hal::i32 floating_to_fixed_point_32(float n, int exponent)
{
  // auto console = resources::console();
  float initial = n * powf(2, exponent);
  // hal::print<64>(*console, "moved: %f -- ", initial);
  hal::i32 shifted = static_cast<hal::i32>(initial);
  // hal::print<64>(*console, "int cast: %d\n", shifted);
  return (shifted);
}
consteval hal::i16 floating_to_fixed_point_16(float n, int exponent)
{
  // auto console = resources::console();
  float initial = n * powf(2, exponent);
  // hal::print<64>(*console, "moved: %f -- ", initial);
  hal::i16 shifted = static_cast<hal::i16>(initial);
  // hal::print<64>(*console, "int cast: %d\n", shifted);
  return (shifted);
  // return (static_cast<hal::i16>(n) << exponent);
}

// convert from fraction to
void print_can_message(hal::serial& p_console,
                                    hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}
}  // namespace sjsu::drivers
