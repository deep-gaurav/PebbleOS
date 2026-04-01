/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/vibe.h"

#include "board/board.h"
#include "console/prompt.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "drivers/pwm.h"
#include "drivers/timer.h"
#include "kernel/util/stop.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/math.h"

#include <string.h>

#define PWM_TIMER_UPDATE_PERIOD (125)
#define PWM_OUTPUT_FREQUENCY_HZ (1000)

#define PWM_TIMER_FREQUENCY_HZ (125000)
#define PWM_DUTY_CYCLE_OFF (0)
#define PWM_DUTY_CYCLE_FULL (PWM_TIMER_UPDATE_PERIOD)

static uint8_t s_vibe_duty_cycle = PWM_DUTY_CYCLE_OFF;
static bool s_initialized = false;

void vibe_init(void) {
  if (s_initialized) {
    return;
  }

  periph_config_acquire_lock();

  if (BOARD_CONFIG_VIBE.options & ActuatorOptions_Ctl) {
    gpio_output_init(&BOARD_CONFIG_VIBE.ctl, GPIO_OType_PP, GPIO_Speed_2MHz);
    gpio_output_set(&BOARD_CONFIG_VIBE.ctl, false);
  }

  if (BOARD_CONFIG_VIBE.options & ActuatorOptions_Pwm) {
    pwm_init(&BOARD_CONFIG_VIBE.pwm, PWM_TIMER_UPDATE_PERIOD, PWM_TIMER_FREQUENCY_HZ);
  }

  s_initialized = true;
  periph_config_release_lock();
}

static void prv_vibe_pwm_enable(bool on) {
  pwm_enable(&BOARD_CONFIG_VIBE.pwm, on);

  static bool stop_mode_disabled = false;
  if (stop_mode_disabled != on) {
    if (on) {
      stop_mode_disable(InhibitorVibes);
    } else {
      stop_mode_enable(InhibitorVibes);
    }
    stop_mode_disabled = on;
  }
}

static uint16_t prv_get_vsys_mv(void) {
  return BOARD_CONFIG_VIBE.vsys_scale;
}

static uint32_t prv_vibe_get_pwm_duty_cycle(int8_t strength) {
  uint32_t duty_cycle = ABS(strength);

  if (BOARD_CONFIG_VIBE.vsys_scale > 0) {
    const uint16_t vsys_mv = prv_get_vsys_mv();
    PBL_ASSERTN(vsys_mv > 0);
    duty_cycle = (vsys_mv * duty_cycle) / BOARD_CONFIG_VIBE.vsys_scale;
  }
  return MIN(duty_cycle, PWM_DUTY_CYCLE_FULL);
}

static void prv_vibe_raw_ctl(bool on) {
  if (BOARD_CONFIG_VIBE.options & ActuatorOptions_Pwm) {
    const uint32_t duty_cycle = (on) ? s_vibe_duty_cycle : PWM_DUTY_CYCLE_OFF;
    prv_vibe_pwm_enable(on);
    pwm_set_duty_cycle(&BOARD_CONFIG_VIBE.pwm, duty_cycle);
  }

  if (BOARD_CONFIG_VIBE.options & ActuatorOptions_Ctl) {
    gpio_output_set(&BOARD_CONFIG_VIBE.ctl, on);
  }
}

void vibe_set_strength(int8_t strength) {
  uint8_t duty_cycle = prv_vibe_get_pwm_duty_cycle(strength);
  s_vibe_duty_cycle = MIN(duty_cycle, PWM_DUTY_CYCLE_FULL);
}

void vibe_ctl(bool on) {
  if (!s_initialized) {
    return;
  }

  PBL_LOG_DBG("Vibe status <%s>", on ? "on" : "off");

  prv_vibe_raw_ctl(on);
}

void vibe_force_off(void) {
  if (!s_initialized) {
    return;
  }

  prv_vibe_raw_ctl(false);
}

int8_t vibe_get_braking_strength(void) {
  return VIBE_STRENGTH_OFF;
}

status_t vibe_calibrate(void) {
  return E_INVALID_OPERATION;
}

void command_vibe_ctl(const char *arg) {
  int strength = atoi(arg);

  const bool out_of_bounds = ((strength < 0) || (strength > VIBE_STRENGTH_MAX));
  const bool not_a_number = (strength == 0 && arg[0] != '0');
  if (out_of_bounds || not_a_number) {
    prompt_send_response("Invalid argument");
    return;
  }

  vibe_set_strength(strength);

  const bool turn_on = strength != 0;
  vibe_ctl(turn_on);
  prompt_send_response("OK");
}
