/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accel.h"
#include "drivers/i2c.h"

typedef struct KX022State KX022State;

typedef struct KX022Config {
  KX022State *state;
  I2CSlavePort *i2c;
  uint8_t axis_map[3];
  int8_t axis_dir[3];
  uint16_t scale_mg;
  uint32_t sampling_interval_us;
} KX022Config;

void kx022_init(const KX022Config *config);
void kx022_power_up(void);
void kx022_power_down(void);
uint32_t kx022_set_sampling_interval(uint32_t interval_us);
int kx022_peek(AccelDriverSample *data);
void kx022_set_num_samples(uint32_t num_samples);
void kx022_enable_shake_detection(bool on);
void kx022_enable_double_tap_detection(bool on);
