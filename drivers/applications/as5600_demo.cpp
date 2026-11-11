#include <array>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <as5600.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  auto clock = resources::clock();
  auto i2c = resources::i2c();

  as5600 encoder(i2c);

  while (true) {
    hal::delay(*clock, 250ms);
    hal::print(*console, "=====================================\n");
    hal::print<64>(*console, "Angle: %4.2f,\n", encoder.read_position());
    hal::print<64>(*console, "Magnitude: %4.2f,\n", encoder.read_magnitude());
    as5600::status s = encoder.read_status();
    hal::print<64>(*console, "magnet_too_strong: %d\n", s.magnet_too_strong);
    hal::print<64>(*console, "magnet_too_weak: %d\n", s.magnet_too_weak);
    hal::print<64>(*console, "magnet_detected: %d\n", s.magnet_detected);
  }
}
}  // namespace sjsu::drivers
