/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/battery.h"
#include "drivers/voltage_monitor.h"
#include "board/boards/board_banglejs2.h"

#include <hal/nrf_saadc.h>
#include <hal/nrf_gpio.h>

//! Battery voltage pin is P0.03 (AIN1) which is D3 on Bangle.js v2
#define BATTERY_VOLTAGE_PIN NRF_GPIO_PIN_MAP(0, 3)
//! Charging detect pin is P0.23 which is D23 on Bangle.js v2 (active low)
#define CHARGE_DETECT_PIN NRF_GPIO_PIN_MAP(0, 23)

//! Battery voltage divider ratio (Vbatt -> ADC pin)
//! Battery voltage divider is approximately 13.36:1
//! This means at 4.2V battery, the ADC sees ~0.3144V
#define BATTERY_DIVIDER_NUM 1336
#define BATTERY_DIVIDER_DEN 100

//! Voltage monitor device for battery
static const VoltageMonitorDevice s_voltage_monitor_battery = {
  .adc = NULL,  // Not used on nRF52840
  .adc_channel = 0,
  .input = NRF_SAADC_INPUT_AIN1,  // P0.03 = AIN1 = D3
  .gpio_pin = BATTERY_VOLTAGE_PIN,
};

void battery_init(void) {
  // Initialize voltage monitor
  voltage_monitor_init();
  voltage_monitor_device_init(&s_voltage_monitor_battery);

  // Configure charge detect pin (P0.23 / D23) as input with pull-up
  // Active low - goes low when charger is connected
  nrf_gpio_cfg_input(CHARGE_DETECT_PIN, NRF_GPIO_PIN_PULLUP);
}

int battery_get_millivolts(void) {
  VoltageReading reading;
  voltage_monitor_read(&s_voltage_monitor_battery, &reading);

  // With VDD/4 reference and 1/4 gain on nRF52840 SAADC (14-bit):
  // - Full-scale (16384) = VDD (input range 0 to VDD with these settings)
  // - result / 16384 = Vbatt_adc / VDD
  //
  // The voltage divider (R1=1MΩ, R2=330kΩ) gives:
  //   Vbatt_adc = Vbatt * R2 / (R1 + R2) = Vbatt * 0.248
  //
  // At 4.2V battery, ADC reading = 4.2 * 0.248 / VDD * 16384
  // If VDD ≈ 3.74V, this is approximately 4580 (our measured full-scale)
  //
  // Using ratiometric approach:
  //   Vbatt = 4200 * avg_reading / ADC_full_scale
  //
  // Where ADC_full_scale = ADC reading at 4.2V battery

  uint32_t avg_reading = reading.vmon_total / NUM_CONVERSIONS;

  // ADC reading at 4.2V battery (calibrated full-scale)
  // This is approximately 4580 for VDD ≈ 3.74V
  const uint32_t kAdcFullScale = 4580;
  const uint32_t kBatteryFullMv = 4200;  // 4.2V in mV

  // Vbatt_mV = kBatteryFullMv * avg_reading / kAdcFullScale
  uint32_t vbatt_mv = (uint64_t)kBatteryFullMv * avg_reading / kAdcFullScale;

  return (int)vbatt_mv;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  // Charge detect pin (P0.23) is active LOW
  // It goes LOW when charger is connected
  return nrf_gpio_pin_read(CHARGE_DETECT_PIN) == 0;
}

bool battery_is_usb_connected_impl(void) {
  // USB connection is detected the same way as charging
  return battery_charge_controller_thinks_we_are_charging_impl();
}

void battery_set_charge_enable(bool charging_enabled) {
  (void)charging_enabled;
  // No charge control on banglejs2 - charging is handled by hardware
}

void battery_set_fast_charge(bool fast_charge_enabled) {
  (void)fast_charge_enabled;
  // No fast charge control on banglejs2
}
