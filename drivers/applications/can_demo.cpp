#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <can_util.hpp>


namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  auto can = resources::can_transceiver();
  auto current_pos = can->receive_cursor();
  hal::print(*console, "waiting for incoming messages");
  while (true) {
    if (current_pos != can->receive_cursor()) {
      can_util::print_can_message(*console,can->receive_buffer()[current_pos]);
      current_pos = (current_pos + 1) % can->receive_buffer().size();
    }
  }
}
}  // namespace sjsu::drivers