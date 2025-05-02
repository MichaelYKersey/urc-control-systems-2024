cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application
    h_bridge_demo

SOURCES
  temp_drivers/h_bridge.cpp
)