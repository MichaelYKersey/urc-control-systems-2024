#ifndef _SHARED_RESOURCES_HPP
#define _SHARED_RESOURCES_HPP
// Main thread write
float volatile shared_power_portion = 0;

// FOC thread write
float volatile shared_angle = -123456789.0;
float volatile shared_zero_angle = -123456789.0;
#endif