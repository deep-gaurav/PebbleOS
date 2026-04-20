/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "drivers/hrm.h"

#include "board/board.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "os/mutex.h"
#include "services/common/new_timer/new_timer.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum HRMEnabledState {
  HRMEnabledState_Uninitialized = 0,
  HRMEnabledState_Disabled,
  HRMEnabledState_PoweringOn,
  HRMEnabledState_Enabled,
} HRMEnabledState;

typedef enum {
  VC31Type_Unknown = 0,
  VC31Type_VC31A,
  VC31Type_VC31B,
} VC31Type;

typedef enum {
  AdjustDirection_Null = 0,
  AdjustDirection_Up = 1,
  AdjustDirection_Down = 2,
} VC31AdjustDirection;

typedef struct {
  VC31AdjustDirection direction;
  VC31AdjustDirection direction_last;
  uint16_t step;
} VC31AdjustInfo;

typedef struct HRMDeviceState {
  HRMEnabledState enabled_state;
  PebbleMutex *lock;
  TimerID timer;
  uint32_t handshake_count;

  VC31Type variant;
  bool is_wearing;
  int8_t is_wear_cnt;
  int8_t un_wear_cnt;

  // Raw metrics
  uint16_t ppg_value;
  uint16_t ppg_last_value;
  int16_t ppg_offset;
  uint8_t was_adjusted;
  uint16_t env_value;
  uint8_t irq_status;
  uint8_t raw[12];

  // IRQ/FIFO counters
  uint32_t irq_count;
  uint32_t fifo_count;
  uint32_t sample_count;

  // VC31A-specific
  uint8_t vc31a_ctrl;
  uint16_t vc31a_current;
  uint16_t vc31a_pre;
  uint16_t vc31a_ps;
  uint16_t vc31a_env;
  uint16_t vc31a_divisor;

  // VC31B-specific
  uint8_t vcb_reg_config[17];
  uint8_t vcb_fifo_read_index;
  uint8_t vcb_total_slots;
  uint8_t vcb_fifo_int_div;
  bool vcb_en_fifo;
  uint8_t vcb_led_current[3];
  uint8_t vcb_led_max_current[3];
  uint8_t vcb_pd_res[3];
  uint8_t vcb_pd_res_max[3];
  uint8_t vcb_pd_res_set[3];
  uint8_t vcb_ppg_gain[3];
  uint16_t vcb_divisor;
  bool vcb_slot0_env_exceed;
  bool vcb_slot1_env_exceed;
  uint8_t vcb_sample_rate;
  VC31AdjustInfo vcb_adjust_info[2];

  // Algorithm state
  RtcTicks last_sample_ticks;
  bool prev_is_wearing;
  uint8_t vcb_adjust_step_display;

  // App-facing debug state
  uint8_t fifo_write_index;
  uint8_t fifo_window[8];
  bool data_changed;
} HRMDeviceState;

typedef const struct HRMDevice {
  HRMDeviceState *state;
  ExtiConfig int_exti;
  InputConfig int_input;
  OutputConfig en_gpio;
  uint8_t sda_pin;
  uint8_t scl_pin;
} HRMDevice;

// Debug getters for bring-up app / prompt commands
VC31Type vc31_get_variant(HRMDevice *dev);
bool vc31_is_wearing(HRMDevice *dev);
void vc31_get_raw_metrics(HRMDevice *dev, uint16_t *ppg, uint16_t *env, uint16_t *ps, uint16_t *current);
void vc31_get_slot_config(HRMDevice *dev, uint8_t *led_current, uint8_t *pd_res, uint8_t *ppg_gain);
uint8_t vc31_get_saturation_streak(HRMDevice *dev);
void vc31_get_counters(HRMDevice *dev, uint32_t *irq_count, uint32_t *fifo_count, uint32_t *sample_count);

// Raw bring-up getters
void vc31_get_status_block(HRMDevice *dev, uint8_t *buf, size_t len);
uint8_t vc31_get_fifo_write_index(HRMDevice *dev);
void vc31_get_fifo_window(HRMDevice *dev, uint8_t *buf, size_t len);
bool vc31_data_changed(HRMDevice *dev);
