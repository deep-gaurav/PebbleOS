// Stub accel driver for banglejs2
#include "drivers/accel.h"

void accel_init(void) {}
void accel_power_up(void) {}
void accel_power_down(void) {}
void accel_set_rotated(bool rotated) {}
uint32_t accel_set_sampling_interval(uint32_t interval_us) { return interval_us; }
uint32_t accel_get_sampling_interval(void) { return 0; }
void accel_set_num_samples(uint32_t num_samples) {}
int accel_peek(AccelDriverSample *data) { return -1; }
void accel_enable_shake_detection(bool on) {}
void accel_disable_shake_detection(void) {}
void accel_enable_double_tap_detection(bool on) {}
void accel_disable_double_tap_detection(void) {}
void accel_set_shake_sensitivity_percent(uint8_t percent) {}
void accel_set_shake_sensitivity_high(bool sensitivity_high) {}
