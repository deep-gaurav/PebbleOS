/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/mcu.h"

#include <string.h>

#define NRF5_COMPATIBLE
#include <mcu.h>

StatusCode mcu_get_serial(void *buf, size_t *buf_sz) {
  const size_t serial_size = sizeof(NRF_FICR->DEVICEID);
  if (*buf_sz < serial_size) {
    return E_OUT_OF_MEMORY;
  }

  memcpy(buf, (const void *)NRF_FICR->DEVICEID, serial_size);
  *buf_sz = serial_size;

  return S_SUCCESS;
}

uint32_t mcu_cycles_to_milliseconds(uint64_t cpu_ticks) {
  return ((cpu_ticks * 1000) / SystemCoreClock);
}
