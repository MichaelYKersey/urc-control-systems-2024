#pragma once
#include <bldc_servo.hpp>
#include <can_commands.hpp>
#include <cmath>
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

#include <h_bridge.hpp>
#include <limits>

using sec = float;

namespace sjsu::perseus {

class bldc_perseus
{

public:
  enum class mode
  {
    power,
    homing,
    velocity,
    position,
  };
  /**
   * @brief Struct keeps position and velocity status of the servo.
   */
  struct status
  {
    hal::degrees position = 0.0;
    float power = 0.0;
    float velocity = 0.0;
  };

  /**
   * @brief Struct keeps PID settings for the servo.
   */
  struct pid_settings
  {
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
  };
  /**
   * @brief Struct keeps previous PID settings for the servo.
   * Obtained from update_velocity/position functions
   */
  struct pid_prev_values
  {
    float integral = 0.0;
    float last_error = 0.0;
    float prev_timestamp = 0.0;
  };
  /**
   * @brief Struct for the values that are individual to each servo.
   */
  struct physical_servo_values
  {
    // convert from ticks to degrees or mm
    float gear_ratio = 1.0;
    // offset of homed angle from ground (where limit switch should hit)
    float angle_offset_from_gravity = 0.0;
    // offset from wake-up zero (so homed angle is zero)
    float angle_offset_from_wake = 0.0;
    // power needed to keep servo in place when link parallel to ground
    float fight_gravity = 0.0;
    // limits on power for safety
    float high_clamped_value = 0.0;
    float low_clamped_value = 0.0;
    // motor spins clockwise when positive power applied
    bool clockwise_positive = false;
    hal::degrees position_min = -std::numeric_limits<float>::infinity();
    hal::degrees position_max = std::numeric_limits<float>::infinity();
    hal::degrees homing_limitswitch_position = NAN;
    float homing_power = 0.0f;
  };

  bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
               hal::v5::strong_ptr<hal::rotation_sensor> p_encoder,
               hal::v5::strong_ptr<hal::steady_clock> p_clock
               // hal::v5::optional_ptr<hal::input_pin> p_homing_pin);
  );

  /**
   * @brief Set servo values
   * @param p_physical_servo_values physical servo values
   * struct)
   */
  void set_physical_servo_values(physical_servo_values p_physical_servo_values);
  /**
   * @brief Get servo values object
   */
  physical_servo_values get_physical_servo_values();

  /**
   * @brief Returns the visible angle (ticks adjusted by gear ratio).
   * @return Angle in hal::degrees
   */
  hal::degrees read_angle();
  /**
   * @brief Get the current position of the servo.
   * @return Gets the position relative to the home position.
   */
  hal::degrees get_reading_position();
  /**
   * @brief Get the current velocity of the servo.
   * @return The current velocity of the servo as a float value representing
   * degrees per second.
   */
  float get_reading_velocity();

  /**
   * @brief Get the servo's current action
   * @return Returns the current action as a can_perseus::action
   */
  mode get_active_mode();

  void periodic();

  /**
   * @brief Sets the power directly to h_bridge (bypass modes)
   * @param p_power The power to set the motor to, as a float between -1.0
   * and 1.0 where -1 is the maximum in one direction and 1 is the maximum in
   * the other direction.
   */
  void set_h_bridge_power(float p_power);
  /**
   * @brief Sets the power and sets mode to power
   * @param p_power The power to set the motor to, as a float between -1.0
   * and 1.0 where -1 is the maximum in one direction and 1 is the maximum in
   * the other direction.
   */
  void set_power(float p_power);
  /**
   * @brief Get the power the servo is using.
   * @return The power as a float between -1.0 and 1.0, representing 0% to 100%
   * of maximum possible power.
   */
  float get_power();

  /**
   * @brief Update the PID settings of the servo.
   * @param p_settings The PID settings to update.
   */
  void update_pid_position(pid_settings p_settings);
  /**
   * @brief Update the velocity PID settings
   * @return the settings for position PID.
   */
  pid_settings get_position_pid_settings();
  /**
   * @brief Set the target position of the servo.
   * @param p_target_position The target position to set, it is float value in
   * degrees.
   */
  void start_position_pid(hal::degrees p_target_position);
  /**
   * @brief Get the target position of the servo.
   * @return Gets the position relative to the home position.
   */
  hal::degrees get_target_position();
  /**
   * @brief Update position to the target position using PID control and
   * feedforward.
   */
  void position_pid_periodic();

  /**
   * @brief Feedforward values to account for gravity/weight
   * @return Current feedforward value
   */
  float position_feedforward();

  /**
   * @brief Set the target velocity of the servo.
   * The servo will try to reach this velocity using acceleration limits.
   * This may change the clamped speed.
   * @param p_target_velocity The target velocity to set, it is a float value in
   * degrees per second.
   */
  void start_velocity_pid(float p_target_velocity);
  /**
   * @brief Get the target velocity of the servo.
   * @return The target velocity of the servo as a float value representing
   * degrees per second.
   */
  float get_target_velocity();
  /**
    * @brief Update velocity to the target velocity using PID control and
    feedforward.
    */
  void velocity_pid_periodic();
  /**
   * @brief Update the PID settings of the servo.
   * @param p_settings The PID settings to update.
   */
  void update_pid_velocity(pid_settings p_settings);
  /**
   * @brief Update the velocity PID settings
   * @return the settings for velocity PID.
   */
  pid_settings get_pid_velocity_settings();

  /**
   * @brief TURNS OFF (Power = 0)
   */
  void stop();

  void start_home_encoder();
  /**
   * @brief Remembers the current position of the encoder as the home position.
   * This should be called when the servo is homed.
   */
  void home_encoder_periodic();

  void set_angle_offset_from_gravity(float angle_offset);
  float get_angle_offset_from_gravity();
  void set_angle_offset_from_wake(float angle_offset);
  float get_angle_offset_from_wake();

  /**
   * @brief Set the previous joint's recorded position.
   * @param p_prev_joint_pos The previous joint's position (float).
   */
  void set_prev_joint_position(float p_prev_joint_pos);
  /**
   * @brief Get the previous joint's recorded position.
   * @return The previous joint's position (float).
   */
  float get_prev_joint_position();

  /**
   * @brief Get actual position of the servo (0 = perpendicular to ground).
   */
  float get_actual_position();

private:
  hal::v5::strong_ptr<sjsu::drivers::h_bridge> m_h_bridge;
  hal::v5::strong_ptr<hal::rotation_sensor> m_encoder;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::input_pin> m_homing_pin;
  hal::u16 m_servo_addr;
  hal::u64 m_last_clock_check;
  status m_target;
  pid_settings m_reading_position_settings;
  pid_settings m_reading_velocity_settings;
  pid_prev_values m_pid_prev_velocity_values;
  pid_prev_values m_pid_prev_position_values;
  physical_servo_values m_physical_servo_values;
  float m_prev_joint_position;
  float m_active_power;
  mode m_active_mode;
};

}  // namespace sjsu::perseus
