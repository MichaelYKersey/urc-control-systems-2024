cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    application
    sequential_homing_demo
    simple_state_set_demo
    prop_motor_test
    chassis_vel_demo
    drive_cycle_demo
    simple_can_reply_demo
    poll_connecitons
    serial_command_demo
INCLUDES
    include
    .
    ../drivers/include
SOURCES
    src/drivetrain_math.cpp
    src/drivetrain.cpp
    src/mission_control_manager.cpp
    src/swerve_module.cpp
    ../drivers/src/serial_commands.cpp

PACKAGES
#    libhal-__device__
    libhal-actuator

LINK_LIBRARIES
#    libhal::__device__
    libhal::actuator
)
