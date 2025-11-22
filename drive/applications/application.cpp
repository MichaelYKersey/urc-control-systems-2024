#include <swerve_module.hpp>
#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <resource_list.hpp>

namespace sjsu::drive {

void application()
{
  auto console = resources::console();
  auto clock = resources::clock();
  constexpr hal::time_duration cycle_time = 50ms;
  constexpr sec cycle_time_sec = hal_time_duration_to_sec(cycle_time);
  drivetrain dt(resources::swerve_modules(), cycle_time_sec);
  mission_control_manager mcm(resources::can_transceiver());

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, cycle_time);
    dt.periodic();
    bool home_req = mcm.read_homing_request();
    std::optional<chassis_velocities_request> cvr =
      mcm.read_set_velocity_request();
    if (cvr) {
      if (home_req) {
        cvr->chassis_vels = {{0,0},0};
      }
      bool resolved = dt.set_target_state(cvr->chassis_vels, cvr->module_conflicts);
      cvr->module_conflicts = resolved;
      mcm.reply_set_velocity_request(cvr.value());
    } else if (home_req) {
      dt.hard_home();//TODO: replace with interuptable homing later
      mcm.reply_homing_request();
    }
    mcm.fulfill_data_requests(dt);

    mcm.reply_heartbeat();

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::drive
