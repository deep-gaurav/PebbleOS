/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/battery.h"

void battery_init(void) {
  // No PMIC on banglejs2, battery reading is stubbed for bring-up.
}

int battery_get_millivolts(void) {
  return 4000;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  return false;
}

bool battery_is_usb_connected_impl(void) {
  return false;
}

void battery_set_charge_enable(bool charging_enabled) {
  (void)charging_enabled;
}

void battery_set_fast_charge(bool fast_charge_enabled) {
  (void)fast_charge_enabled;
}
