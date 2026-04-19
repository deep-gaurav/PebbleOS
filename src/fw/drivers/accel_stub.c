// KX022 accelerometer driver for BangleJS2
#include "drivers/accel.h"
#include "drivers/imu/kx022/kx022.h"
#include "drivers/imu/kx022/kx022_private.h"

#include "board/board.h"
#include "system/logging.h"

static KX022State s_kx022_state_storage;

static KX022Config s_kx022_config;

void accel_init(void) {
  PBL_LOG_INFO("Accel: Initializing KX022 driver\n");
  
  s_kx022_config = (KX022Config){
    .state = &s_kx022_state_storage,
    .i2c = I2C_LSM6D,
    .axis_map = {0, 1, 2},
    .axis_dir = {-1, 1, -1},
    .scale_mg = 4000,
    .sampling_interval_us = 80000,
  };
  
  kx022_init(&s_kx022_config);
}

void accel_power_up(void) {
  kx022_power_up();
}

void accel_power_down(void) {
  kx022_power_down();
}

void accel_set_rotated(bool rotated) {
  (void)rotated;
}

uint32_t accel_set_sampling_interval(uint32_t interval_us) {
  return kx022_set_sampling_interval(interval_us);
}

uint32_t accel_get_sampling_interval(void) {
  return s_kx022_config.sampling_interval_us;
}

void accel_set_num_samples(uint32_t num_samples) {
  kx022_set_num_samples(num_samples);
}

int accel_peek(AccelDriverSample *data) {
  return kx022_peek(data);
}

void accel_enable_shake_detection(bool on) {
  kx022_enable_shake_detection(on);
}

bool accel_get_shake_detection_enabled(void) {
  return s_kx022_state_storage.shake_detection_enabled;
}

void accel_disable_shake_detection(void) {
  kx022_enable_shake_detection(false);
}

void accel_enable_double_tap_detection(bool on) {
  kx022_enable_double_tap_detection(on);
}

bool accel_get_double_tap_detection_enabled(void) {
  return s_kx022_state_storage.double_tap_enabled;
}

void accel_disable_double_tap_detection(void) {
  kx022_enable_double_tap_detection(false);
}

void accel_set_shake_sensitivity_percent(uint8_t percent) {
  (void)percent;
}

void accel_set_shake_sensitivity_high(bool sensitivity_high) {
  (void)sensitivity_high;
}
