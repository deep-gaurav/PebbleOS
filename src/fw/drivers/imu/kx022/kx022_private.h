/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kx022.h"
#include "drivers/accel.h"

struct KX022State {
  const KX022Config *config;
  bool initialized;
  bool powered_up;
  bool shake_detection_enabled;
  bool double_tap_enabled;
  uint32_t sampling_interval_us;
  AccelDriverSample last_sample;
  bool last_sample_valid;
};
