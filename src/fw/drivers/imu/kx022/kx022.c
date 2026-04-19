/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kx022.h"
#include "kx022_private.h"
#include "kx022_regs.h"

#include "board/board.h"
#include "drivers/accel.h"
#include "drivers/i2c.h"
#include "drivers/rtc.h"
#include "kernel/util/delay.h"
#include "os/mutex.h"
#include "services/common/new_timer/new_timer.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/time/time.h"

#define MIN_ODR_INTERVAL_US 80000

#define TAP_DELTA_THRESHOLD_MG    1500
#define TAP_COOLDOWN_TICKS        8

static const KX022Config *s_kx022_config;
static PebbleMutex *s_kx022_mutex;
static uint32_t s_kx022_timer_id;
static uint32_t s_kx022_num_samples;

// Software tap detection state
static AccelDriverSample s_prev_sample;
static bool s_prev_sample_valid = false;
static uint32_t s_tap_cooldown = 0;
static uint32_t s_timer_tick_count = 0;

static void prv_timer_callback(void *data);

static void prv_write_register(uint8_t reg, uint8_t value) {
  i2c_use(s_kx022_config->i2c);
  i2c_write_register(s_kx022_config->i2c, reg, value);
  i2c_release(s_kx022_config->i2c);
}

static uint8_t prv_read_register(uint8_t reg) {
  uint8_t value;
  i2c_use(s_kx022_config->i2c);
  i2c_read_register(s_kx022_config->i2c, reg, &value);
  i2c_release(s_kx022_config->i2c);
  return value;
}

static void prv_read_registers(uint8_t reg, uint8_t *buffer, uint32_t length) {
  i2c_use(s_kx022_config->i2c);
  i2c_read_register_block(s_kx022_config->i2c, reg, length, buffer);
  i2c_release(s_kx022_config->i2c);
}

static void prv_read_sample(AccelDriverSample *sample) {
  uint8_t buffer[6];
  prv_read_registers(KX022_XOUT_L, buffer, 6);
  
  int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
  
  int16_t samples[3] = {raw_x, raw_y, raw_z};
  int16_t mapped[3];
  
  for (int i = 0; i < 3; i++) {
    int mapped_idx = s_kx022_config->axis_map[i];
    mapped[i] = samples[mapped_idx] * s_kx022_config->axis_dir[i];
  }
  
  uint16_t scale_mg = s_kx022_config->scale_mg;
  int32_t x_mg = ((int32_t)mapped[0] * scale_mg) / 32768;
  int32_t y_mg = ((int32_t)mapped[1] * scale_mg) / 32768;
  int32_t z_mg = ((int32_t)mapped[2] * scale_mg) / 32768;
  
  time_t time_s;
  uint16_t time_ms;
  rtc_get_time_ms(&time_s, &time_ms);
  uint64_t timestamp_us = ((uint64_t)time_s) * 1000000 + ((uint64_t)time_ms) * 1000;
  
  *sample = (AccelDriverSample){
    .x = (int16_t)x_mg,
    .y = (int16_t)y_mg,
    .z = (int16_t)z_mg,
    .timestamp_us = timestamp_us,
  };
}

static bool prv_needs_timer(void) {
  return (s_kx022_num_samples > 0 ||
          s_kx022_config->state->double_tap_enabled);
}

static uint32_t prv_get_timer_interval_ms(void) {
  if (s_kx022_num_samples > 0) {
    return s_kx022_config->state->sampling_interval_us / 1000;
  }
  return MIN_ODR_INTERVAL_US / 1000;
}

static void prv_update_timer(void) {
  if (prv_needs_timer()) {
    uint32_t interval_ms = prv_get_timer_interval_ms();
    PBL_LOG_DBG("KX022: starting timer interval=%lu ms", (unsigned long)interval_ms);
    new_timer_start(s_kx022_timer_id, interval_ms,
                    prv_timer_callback, NULL, TIMER_START_FLAG_REPEATING);
  } else {
    PBL_LOG_DBG("KX022: stopping timer (no subscribers)");
    new_timer_stop(s_kx022_timer_id);
  }
}

// Compute the absolute delta between two samples across all axes
static uint32_t prv_compute_sample_delta(const AccelDriverSample *a, const AccelDriverSample *b) {
  int32_t dx = (int32_t)a->x - (int32_t)b->x;
  int32_t dy = (int32_t)a->y - (int32_t)b->y;
  int32_t dz = (int32_t)a->z - (int32_t)b->z;
  uint32_t abs_dx = (dx < 0) ? -dx : dx;
  uint32_t abs_dy = (dy < 0) ? -dy : dy;
  uint32_t abs_dz = (dz < 0) ? -dz : dz;
  return abs_dx + abs_dy + abs_dz;
}

static void prv_timer_callback(void *data) {
  mutex_lock(s_kx022_mutex);
  
  s_timer_tick_count++;
  
  if (s_tap_cooldown > 0) s_tap_cooldown--;
  
  bool need_sample = (s_kx022_config->state->powered_up &&
                      (s_kx022_num_samples > 0 ||
                       s_kx022_config->state->double_tap_enabled));

  if (need_sample) {
    AccelDriverSample sample;
    prv_read_sample(&sample);
    s_kx022_config->state->last_sample = sample;
    s_kx022_config->state->last_sample_valid = true;
    
    if (s_kx022_num_samples > 0) {
      accel_cb_new_sample(&sample);
    }
    
    if (s_prev_sample_valid && s_kx022_config->state->double_tap_enabled) {
      uint32_t delta = prv_compute_sample_delta(&sample, &s_prev_sample);
      
      if (delta >= TAP_DELTA_THRESHOLD_MG && s_tap_cooldown == 0) {
        PBL_LOG_DBG("KX022: SW tap detected (delta=%u mg, tick=%u)", (unsigned)delta, (unsigned)s_timer_tick_count);
        s_tap_cooldown = TAP_COOLDOWN_TICKS;
        accel_cb_double_tap_detected(AXIS_Z, 0);
        accel_cb_shake_detected(AXIS_Z, 0);
      }
    }
    
    s_prev_sample = sample;
    s_prev_sample_valid = true;
  }
  
  if (s_kx022_config->state->double_tap_enabled) {
    uint8_t ins1 = prv_read_register(KX022_INS1);
    
    if ((ins1 & KX022_INS1_TDS) && s_tap_cooldown == 0) {
      PBL_LOG_DBG("KX022: HW tap detected (INS1=0x%02X, tick=%u)", ins1, (unsigned)s_timer_tick_count);
      s_tap_cooldown = TAP_COOLDOWN_TICKS;
      accel_cb_double_tap_detected(AXIS_Z, 0);
      accel_cb_shake_detected(AXIS_Z, 0);
    }
    
    prv_read_register(KX022_INT_REL);
  }
  
  if (s_timer_tick_count % 64 == 1) {
    uint8_t cntl1 = prv_read_register(KX022_CNTL1);
    PBL_LOG_DBG("KX022: tick=%u CNTL1=0x%02X tap=%d samples=%u interval=%luus peek_valid=%d",
                (unsigned)s_timer_tick_count, cntl1,
                s_kx022_config->state->double_tap_enabled,
                (unsigned)s_kx022_num_samples,
                (unsigned long)s_kx022_config->state->sampling_interval_us,
                s_kx022_config->state->last_sample_valid);
  }
  
  mutex_unlock(s_kx022_mutex);
}

void kx022_init(const KX022Config *config) {
  s_kx022_config = config;
  PBL_ASSERTN(s_kx022_config->state != NULL);
  *s_kx022_config->state = (KX022State){
    .config = config,
    .initialized = false,
    .powered_up = false,
  };
  s_kx022_num_samples = 0;
  s_kx022_mutex = mutex_create();
  s_kx022_timer_id = new_timer_create();

  PBL_LOG_INFO("KX022: Initializing accelerometer...\n");
  
  uint8_t whoami = prv_read_register(KX022_WHO_AM_I);
  PBL_LOG_INFO("KX022: WHO_AM_I = 0x%02X (expected 0x14)\n", whoami);
  
  if (whoami != KX022_WHO_AM_I_VALUE) {
    PBL_LOG_ERR("KX022: Wrong chip ID! Got 0x%02X, expected 0x%02X\n", 
                  whoami, KX022_WHO_AM_I_VALUE);
    return;
  }

  prv_write_register(KX022_CNTL1, 0x0A);
  prv_write_register(KX022_CNTL2, KX022_CNTL2_SRST);
  delay_us(10000);
  
  whoami = prv_read_register(KX022_WHO_AM_I);
  PBL_LOG_INFO("KX022: After reset WHO_AM_I = 0x%02X\n", whoami);

  prv_write_register(KX022_CNTL3, 0x98);
  
  prv_write_register(KX022_ODCNTL, KX022_ODCNTL_OSA_12P5);
  s_kx022_config->state->sampling_interval_us = 80000;

  prv_write_register(KX022_INC1, 0x00);
  prv_write_register(KX022_INC2, 0x00);
  prv_write_register(KX022_INC3, 0x00);
  prv_write_register(KX022_INC4, 0x00);
  prv_write_register(KX022_INC5, 0x00);
  prv_write_register(KX022_INC6, 0x00);

  prv_write_register(KX022_TDTRC, KX022_TDTRC_NTD | KX022_TDTRC_PTD |
                                   KX022_TDTRC_NSD | KX022_TDTRC_PSD);
  prv_write_register(KX022_TDTC, 0x78);
  prv_write_register(KX022_TTH, 0xCB);
  prv_write_register(KX022_TTL, 0x25);
  prv_write_register(KX022_LP_CNTL, KX022_LP_CNTL_AVER_4X);
  prv_write_register(KX022_BUF_CLEAR, 0x00);

  uint8_t cntl1 = KX022_CNTL1_RES | KX022_CNTL1_DRDYE | KX022_CNTL1_TDTE;
  
  if (config->scale_mg <= 2000) {
    cntl1 |= KX022_CNTL1_GSEL_2G;
  } else if (config->scale_mg <= 4000) {
    cntl1 |= KX022_CNTL1_GSEL_4G;
  } else if (config->scale_mg <= 8000) {
    cntl1 |= KX022_CNTL1_GSEL_8G;
  } else {
    cntl1 |= KX022_CNTL1_GSEL_16G;
  }
  
  cntl1 |= KX022_CNTL1_PC1;
  prv_write_register(KX022_CNTL1, cntl1);

  s_kx022_config->state->initialized = true;
  s_kx022_config->state->powered_up = true;
  s_kx022_config->state->shake_detection_enabled = false;
  s_kx022_config->state->double_tap_enabled = true;
  
  prv_update_timer();

  PBL_LOG_INFO("KX022: Initialization complete (CNTL1=0x%02X)\n", cntl1);
}

void kx022_power_up(void) {
}

void kx022_power_down(void) {
}

uint32_t kx022_set_sampling_interval(uint32_t interval_us) {
  if (!s_kx022_config->state->initialized) {
    return 0;
  }
  
  mutex_lock(s_kx022_mutex);
  
  uint32_t actual_interval_us;
  if (interval_us >= 80000) {
    prv_write_register(KX022_ODCNTL, KX022_ODCNTL_OSA_12P5);
    actual_interval_us = 80000;
  } else if (interval_us >= 40000) {
    prv_write_register(KX022_ODCNTL, KX022_ODCNTL_OSA_25);
    actual_interval_us = 40000;
  } else if (interval_us >= 20000) {
    prv_write_register(KX022_ODCNTL, KX022_ODCNTL_OSA_50);
    actual_interval_us = 20000;
  } else {
    prv_write_register(KX022_ODCNTL, KX022_ODCNTL_OSA_100);
    actual_interval_us = 10000;
  }
  
  s_kx022_config->state->sampling_interval_us = actual_interval_us;
  
  prv_update_timer();
  
  mutex_unlock(s_kx022_mutex);
  
  return actual_interval_us;
}

int kx022_peek(AccelDriverSample *data) {
  if (!s_kx022_config->state->initialized) {
    return -1;
  }
  
  mutex_lock(s_kx022_mutex);
  
  if (s_kx022_config->state->last_sample_valid) {
    *data = s_kx022_config->state->last_sample;
    mutex_unlock(s_kx022_mutex);
    return 0;
  }
  
  if (s_kx022_config->state->powered_up) {
    prv_read_sample(data);
    s_kx022_config->state->last_sample = *data;
    s_kx022_config->state->last_sample_valid = true;
    mutex_unlock(s_kx022_mutex);
    return 0;
  }
  
  mutex_unlock(s_kx022_mutex);
  return -1;
}

void kx022_set_num_samples(uint32_t num_samples) {
  if (!s_kx022_config->state->initialized) {
    return;
  }
  
  mutex_lock(s_kx022_mutex);
  
  s_kx022_num_samples = num_samples;
  
  prv_update_timer();
  
  mutex_unlock(s_kx022_mutex);
}

void kx022_enable_shake_detection(bool on) {
  s_kx022_config->state->shake_detection_enabled = on;

  if (!s_kx022_config->state->initialized) {
    return;
  }

  mutex_lock(s_kx022_mutex);
  prv_update_timer();
  mutex_unlock(s_kx022_mutex);

  PBL_LOG_INFO("KX022: Shake detection %s (no HW, tap-only driver)\n", on ? "enabled" : "disabled");
}

void kx022_enable_double_tap_detection(bool on) {
  s_kx022_config->state->double_tap_enabled = on;
  
  if (!s_kx022_config->state->initialized) {
    return;
  }
  
  mutex_lock(s_kx022_mutex);
  
  // KX022 requires standby mode (PC1=0) before modifying CNTL1 config bits
  uint8_t cntl1 = prv_read_register(KX022_CNTL1);
  prv_write_register(KX022_CNTL1, cntl1 & ~KX022_CNTL1_PC1);  // Enter standby
  
  if (on) {
    cntl1 |= KX022_CNTL1_TDTE;
  } else {
    cntl1 &= ~KX022_CNTL1_TDTE;
  }
  prv_write_register(KX022_CNTL1, cntl1);  // Write config + re-enable PC1
  
  prv_update_timer();
  
  mutex_unlock(s_kx022_mutex);
  
  PBL_LOG_INFO("KX022: Tap detection %s (CNTL1=0x%02X)\n", on ? "enabled" : "disabled", cntl1);
}