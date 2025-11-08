#pragma  once
#include <swerve_module.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <drivetrain.hpp>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <optional>

namespace sjsu::drive {

struct chassis_velocities_request
{
  chassis_velocities chassis_vels;
  /**
   * MC uses it as if you should resolve_module conflicts
   * Drive uses it as if it ran into a module conflict
   */
  bool module_conflicts;
};

class mission_control_manager
{
public:
  mission_control_manager(
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);

  float fixed_point_to_float(int16_t p_fixed_point_num, int p_expo);
  int16_t float_to_fixed_point(float p_float_num, int p_expo);
  
  int16_t array_to_int16(std::array<hal::byte, 4> const& p_array);
  std::array<hal::byte, 4> int16_to_array(int16_t p_num);

  // returns most recent velocity request
  std::optional<chassis_velocities_request> read_set_velocity_request();
  void reply_set_velocity_request(
    chassis_velocities_request const& p_chassis_vel);

  // return true if homing requested;
  bool read_homing_request();
  void reply_homing_request();

  void fulfill_data_requests(drivetrain const& p_drivetrain);

  void clear_heartbeat_requests();
  void reply_heartbeat();

  // Config
  // Config reply

  /*
  MC info needed each cycle
  - connected (heat beat or other message)
  - target vels (optional)
  - homing requested (and not overridden by more recent request)
  - module offsets requested (byte acts as bool array?)
  - if estimate requested
  - config requests (???)

  info needed to reply
  - if interpolate is needed
  - offsets (just a function call)
  - vel estimate
  */
private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::can_message_finder m_heartbeat_message_finder;
  hal::can_message_finder m_set_chassis_velocities_message_finder;
  hal::can_message_finder m_get_chassis_velocities_message_finder;
  hal::can_message_finder m_homing_request_message_finder;
  hal::can_message_finder m_get_offset_message_finder;
};

}  // namespace sjsu::drive