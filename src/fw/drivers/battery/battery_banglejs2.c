/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/battery.h"

void battery_init(void) {
  // No PMIC on banglejs2, battery reading is stubbed
}

int battery_get_millivolts(void) {
  // Stub: return 4000mV (80% charge) - FIXME: need proper ADC reading
  return 4000;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  // Stub: assume not charging
  return false;
}

bool battery_is_usb_connected_impl(void) {
  // Stub: assume USB connected for now
  return true;
}

void battery_set_charge_enable(bool charging_enabled) {
  // Stub: no-op
}

void battery_set_fast_charge(bool fast_charge_enabled) {
  // Stub: no-op
}
