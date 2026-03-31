/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/voltage_monitor.h"
#include "os/mutex.h"
#include "system/passert.h"
#include "system/logging.h"

#define NRF5_COMPATIBLE
#include <mcu.h>

#include <hal/nrf_saadc.h>

static PebbleMutex *s_adc_mutex;

void voltage_monitor_init(void) {
  s_adc_mutex = mutex_create();
}

void voltage_monitor_device_init(const VoltageMonitorDevice *device) {
  (void)device;
}

void voltage_monitor_read(const VoltageMonitorDevice *device, VoltageReading *reading_out) {
  PBL_ASSERTN(device != NULL);
  PBL_ASSERTN(reading_out != NULL);

  mutex_lock(s_adc_mutex);

  // Configure channel for single-ended read on AIN1 (P0.03)
  // Using VDD/4 as reference and 1/4 gain - this makes the result
  // ratiometric: result/16384 = Vbatt/VDD
  nrf_saadc_channel_config_t channel_config = {
#if NRF_SAADC_HAS_CH_CONFIG_RES
      .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
      .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
#endif
      .gain = NRF_SAADC_GAIN1_4,            // 1/4 input gain
      .reference = NRF_SAADC_REFERENCE_VDD4, // VDD/4 as reference
      .acq_time = NRF_SAADC_ACQTIME_3US,
      .mode = NRF_SAADC_MODE_SINGLE_ENDED,
      .burst = NRF_SAADC_BURST_DISABLED,
  };

  // Accumulate readings
  *reading_out = (VoltageReading){};

  // Function-static buffer in DataRAM, properly aligned for EasyDMA
  // Using static ensures the buffer is in DataRAM (not stack)
  static uint16_t __attribute__((aligned(4))) buffer[NUM_CONVERSIONS];

  for (int i = 0; i < NUM_CONVERSIONS; i++) {
    // Initialize buffer for this sample
    nrf_saadc_buffer_init(NRF_SAADC, &buffer[i], 1);

    // Initialize channel 0 with our config
    nrf_saadc_channel_init(NRF_SAADC, 0, &channel_config);

    // Set pin positive input to AIN1, negative disabled (single-ended)
    nrf_saadc_channel_input_set(NRF_SAADC, 0, device->input, NRF_SAADC_INPUT_DISABLED);

    // Enable SAADC
    nrf_saadc_enable(NRF_SAADC);

    // Set 14-bit resolution
    nrf_saadc_resolution_set(NRF_SAADC, NRF_SAADC_RESOLUTION_14BIT);

    // Trigger conversion
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);

    // Wait for END event (polling loop)
    while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_END)) {
      // Could add timeout here if needed
    }
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);

    // Read via pointer - key fix for when nrf_saadc_value_t is void
    // When nrf_saadc_value_t is void, we can't directly read from the buffer
    // because the compiler doesn't know the size. Use buffer_pointer_get
    // to retrieve the pointer where EasyDMA wrote, then explicitly cast.
    nrf_saadc_value_t *result_ptr = nrf_saadc_buffer_pointer_get(NRF_SAADC);
    uint16_t sample = *(volatile uint16_t *)result_ptr;

    // DEBUG: Log raw sample value
    PBL_LOG_DBG("ADC sample[%d] ptr=%p value=%u",
                i, (void *)result_ptr, sample);

    reading_out->vmon_total += sample;

    // Stop and disable for next sample
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_STOP);
    nrf_saadc_disable(NRF_SAADC);
  }

  // For ratiometric reading with VDD/4 reference, vref_total represents
  // the full-scale value (16384) per sample, scaled by NUM_CONVERSIONS.
  // This maintains compatibility with the STM32 voltage_monitor interface
  // where vref_total is used to compute the ratio.
  reading_out->vref_total = 16384 * NUM_CONVERSIONS;

  PBL_LOG_DBG("ADC vmon_total=%"PRIu32" vref_total=%"PRIu32,
              reading_out->vmon_total, reading_out->vref_total);

  mutex_unlock(s_adc_mutex);
}
