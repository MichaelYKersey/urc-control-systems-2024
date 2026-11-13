#include <array>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <libhal-sensor/as5600.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  auto clock = resources::clock();
  auto i2c = resources::i2c();

  hal::sensor::as5600 encoder(i2c);

  while (true) {
    hal::delay(*clock, 750ms);
    auto raw_angle = encoder.raw_angle();
    auto magnitude = encoder.magnitude();
    auto s = encoder.magnet_status();
    hal::print(*console, "=====================================\n");
    hal::print<64>(*console, "Angle: %4.2f,\n", raw_angle);
    hal::print<64>(*console, "Magnitude: %4.2f,\n", magnitude);
    hal::print<64>(*console, "magnet_too_strong: %d\n", s.too_strong);
    hal::print<64>(*console, "magnet_too_weak: %d\n", s.too_weak);
    hal::print<64>(*console, "magnet_detected: %d\n", s.detected);
  }
}
}  // namespace sjsu::drivers
