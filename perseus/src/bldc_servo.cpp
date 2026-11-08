#include <bldc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

#include <resource_list.hpp>
#include <time_util.hpp>

using namespace std::chrono_literals;

namespace sjsu::perseus {

bldc_perseus::bldc_perseus(
  hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
  hal::v5::strong_ptr<hal::rotation_sensor> p_encoder,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::v5::optional_ptr<hal::input_pin> p_homing_pin)
  : m_h_bridge(p_hbridge)
  , m_encoder(p_encoder)
  , m_clock(p_clock)
  , m_homing_pin(p_homing_pin)
{
}

void bldc_perseus::start_position_pid(hal::degrees p_target_position)
{
  m_active_mode = mode::position;
  m_target.position = p_target_position;
  m_pid_prev_position_values.last_error = 0;
  m_pid_prev_position_values.integral = 0;
  m_pid_prev_position_values.prev_timestamp = m_clock->uptime();
}

float bldc_perseus::get_target_position()
{
  return m_target.position;
}

void bldc_perseus::start_velocity_pid(float p_target_velocity)
{
  m_active_mode = mode::velocity;
  m_target.velocity = p_target_velocity;
  m_pid_prev_velocity_values.last_error = 0;
  m_pid_prev_velocity_values.integral = 0;
  m_pid_prev_velocity_values.prev_timestamp = m_clock->uptime();
}

float bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}
float bldc_perseus::get_velocity_reading()
{
  hal::u64 time_t0 = m_clock->uptime();
  hal::degrees pos_p0 = read_angle();
  hal::delay(*m_clock, 100ms);
  hal::u64 time_t1 = m_clock->uptime();
  hal::degrees pos_p1 = read_angle();
  float dt = static_cast<float>(time_t1 - time_t0) / m_clock->frequency();
  float dp_over_dt = (pos_p1 - pos_p0) / dt;
  return dp_over_dt;
}

float bldc_perseus::get_power()
{
  return m_active_power;
}

void bldc_perseus::set_h_bridge_power(float p_power)
{
  m_active_power = p_power;
  m_h_bridge->power(m_active_power);
}
void bldc_perseus::set_power(float p_power)
{
  m_active_mode = mode::power;
  set_h_bridge_power(p_power);
}

void bldc_perseus::stop()
{
  set_power(0.0f);
}

void bldc_perseus::update_pid_position(pid_settings p_settings)
{
  m_position_pid_settings = p_settings;
}
void bldc_perseus::update_pid_velocity(pid_settings p_settings)
{
  m_velocity_pid_settings = p_settings;
}

void bldc_perseus::start_home_encoder()
{
  m_active_mode = mode::homing;
  home_encoder_periodic();
}

void bldc_perseus::home_encoder_periodic()
{
  auto console = resources::console();
  // only spin track, shoulder, elbow
  // bc unknown how to deal with wrist, end-effector
  set_power(m_physical_servo_values.homing_power);
  if (m_homing_pin->level() == true) {
    return;
  }
  set_angle_offset_from_wake(read_angle() * -1);
  set_power(0.0f);
}

hal::degrees bldc_perseus::read_angle()
{
  return (m_encoder->read().angle / m_physical_servo_values.gear_ratio);
}

void bldc_perseus::velocity_pid_periodic()
{
  auto console = resources::console();
  // pid portion
  float error = m_target.velocity - get_velocity_reading();
  sec curr_time = drivers::time_util::hal_time_duration_to_sec(
    sjsu::drivers::time_util::get_clock_time(*m_clock));
  sec dt = curr_time - m_pid_prev_velocity_values.prev_timestamp;
  m_pid_prev_velocity_values.integral += error * dt;
  float derivative = (error - m_pid_prev_velocity_values.last_error) / dt;
  float p_term = m_velocity_pid_settings.kp * error;
  float i_term =
    m_velocity_pid_settings.ki * m_pid_prev_velocity_values.integral;
  float d_term = m_velocity_pid_settings.kd * derivative;
  m_pid_prev_velocity_values.last_error = error;
  m_pid_prev_velocity_values.prev_timestamp = curr_time;
  float pid_sum = p_term + i_term + d_term;
  float projected_power =
    std::clamp(pid_sum,
               m_physical_servo_values.low_clamped_value,
               m_physical_servo_values.high_clamped_value);
  hal::print<128>(
    *console,
    "Target: %f, Velocity: %f, Error: %f, pid: %f, projected: %f\n",
    m_target.velocity,
    get_velocity_reading(),
    error,
    pid_sum,
    projected_power);
  m_active_power = projected_power;
  m_h_bridge->power(m_active_power);
}

// position
void bldc_perseus::position_pid_periodic()
{
  auto console = resources::console();
  // pid portion
  float error = m_target.position - get_actual_position();
  sec curr_time = drivers::time_util::hal_time_duration_to_sec(
    sjsu::drivers::time_util::get_clock_time(*m_clock));
  sec dt = curr_time - m_pid_prev_position_values.prev_timestamp;
  m_pid_prev_position_values.integral += error * dt;
  float derivative = (error - m_pid_prev_position_values.last_error) / dt;
  float p_term = m_position_pid_settings.kp * error;
  float i_term =
    m_position_pid_settings.ki * m_pid_prev_position_values.integral;
  float d_term = m_position_pid_settings.kd * derivative;
  m_pid_prev_position_values.last_error = error;
  m_pid_prev_position_values.prev_timestamp = curr_time;
  float pid_sum = p_term + i_term + d_term;
  // feed forward
  float feedforward = bldc_perseus::position_feedforward();
  // apply
  float projected_power = pid_sum + feedforward;
  projected_power = std::clamp(projected_power,
                               m_physical_servo_values.low_clamped_value,
                               m_physical_servo_values.high_clamped_value);
  hal::print<128>(
    *console,
    "Target: %f, Position: %f, Error: %f, pid: %f, projected: %f\n",
    m_target.position,
    get_actual_position(),
    error,
    pid_sum,
    projected_power);
  m_active_power = projected_power;
  m_h_bridge->power(m_active_power);
}

// use actual position here once can be communicated/calculated via can
float bldc_perseus::position_feedforward()
{
  return std::sin(static_cast<float>(std::numbers::pi / 180) *
                  get_actual_position()) *
         m_physical_servo_values.fight_gravity;
}

void bldc_perseus::set_prev_joint_position(hal::degrees p_prev_joint_pos)
{
  m_prev_joint_position = p_prev_joint_pos;
}

float bldc_perseus::get_prev_joint_position()
{
  return m_prev_joint_position;
}

void bldc_perseus::set_angle_offset_from_gravity(hal::degrees p_angle_offset)
{
  m_physical_servo_values.angle_offset_from_gravity = p_angle_offset;
}

float bldc_perseus::get_angle_offset_from_gravity()
{
  return m_physical_servo_values.angle_offset_from_gravity;
}

void bldc_perseus::set_angle_offset_from_wake(hal::degrees p_angle_offset)
{
  m_physical_servo_values.angle_offset_from_wake = p_angle_offset;
}

float bldc_perseus::get_angle_offset_from_wake()
{
  return m_physical_servo_values.angle_offset_from_wake;
}

float bldc_perseus::get_actual_position()
{
  return read_angle() + m_physical_servo_values.angle_offset_from_gravity +
         m_physical_servo_values.angle_offset_from_wake + m_prev_joint_position;
}

void bldc_perseus::set_physical_servo_values(
  physical_servo_values p_physical_servo_values)
{
  m_physical_servo_values = p_physical_servo_values;
}

bldc_perseus::physical_servo_values bldc_perseus::get_physical_servo_values()
{
  return m_physical_servo_values;
}

void bldc_perseus::periodic()
{
  switch (m_active_mode) {
    case mode::homing: {
      home_encoder_periodic();
      break;
    }
    case mode::position: {
      position_pid_periodic();
      break;
    }
    case mode::velocity: {
      velocity_pid_periodic();
      break;
    }
    default:
      break;
  }
}
bldc_perseus::mode bldc_perseus::get_active_mode()
{
  return m_active_mode;
}

bldc_perseus::pid_settings bldc_perseus::get_position_pid_settings()
{
  return m_position_pid_settings;
}
bldc_perseus::pid_settings bldc_perseus::get_pid_velocity_settings()
{
  return m_velocity_pid_settings;
}

}  // namespace sjsu::perseus
