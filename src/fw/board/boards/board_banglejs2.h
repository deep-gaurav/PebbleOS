#pragma once

#include "services/imu/units.h"
#include "util/size.h"

#define BT_VENDOR_ID 0x0EEA
#define BT_VENDOR_NAME "Core Devices LLC"

#define BOARD_RTC_INST NRF_RTC1
#define BOARD_RTC_IRQN RTC1_IRQn

static const BoardConfig BOARD_CONFIG = {
  .ambient_light_dark_threshold = 100,
  .ambient_k_delta_threshold = 30,
  .als_always_on = true,

  .backlight_on_percent = 25,
  .backlight_max_duty_cycle_percent = 67,

  .dbgserial_int = {
    .peripheral = NRFX_GPIOTE_INSTANCE(0),
    .channel = 0,
    .gpio_pin = NRF_GPIO_PIN_MAP(1, 10),
  },

  .has_mic = true,
  .mic_config = {
    .gain = 40,
  },

  .lcd_com = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 6), true },
};

static const BoardConfigButton BOARD_CONFIG_BUTTON = {
  .buttons = {
    [BUTTON_ID_BACK] =
        { "Back",   { NRFX_GPIOTE_INSTANCE(0), 1, NRF_GPIO_PIN_MAP(0, 17) }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_UP] =
        { "Up",     { NRFX_GPIOTE_INSTANCE(0), 0, GPIO_Pin_NULL }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_SELECT] =
        { "Select", { NRFX_GPIOTE_INSTANCE(0), 0, GPIO_Pin_NULL }, NRF_GPIO_PIN_PULLUP },
    [BUTTON_ID_DOWN] =
        { "Down",   { NRFX_GPIOTE_INSTANCE(0), 0, GPIO_Pin_NULL }, NRF_GPIO_PIN_PULLUP },
  },
  .active_high = false,
  .timer = NRFX_TIMER_INSTANCE(1),
};

static const BoardConfigPower BOARD_CONFIG_POWER = {
  .pmic_int = { NRFX_GPIOTE_INSTANCE(0), 1, NRF_GPIO_PIN_MAP(1, 12) },
  .pmic_int_gpio = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(1, 12) },
  .low_power_threshold = 2,

  // Memfault is currently estimates a bit above 400 hours as the median
  .battery_capacity_hours = 400,
};

static const BoardConfigActuator BOARD_CONFIG_VIBE = {
  .ctl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 2), true }, // LRA_EN
  .vsys_scale = 3300,
};

static const BoardConfigAccel BOARD_CONFIG_ACCEL = {
  .accel_config = {
    .axes_offsets[AXIS_X] = 1,
    .axes_offsets[AXIS_Y] = 0,
    .axes_offsets[AXIS_Z] = 2,
    .axes_inverts[AXIS_X] = false,
    .axes_inverts[AXIS_Y] = false,
    .axes_inverts[AXIS_Z] = false,
    // This will need calibration.
    .shake_thresholds[AccelThresholdHigh] = 64,
    .shake_thresholds[AccelThresholdLow] = 0xf,
    .double_tap_threshold = 12500, //This already is a good value and does not need calibration.
  // LSM6DSO tap timing register values tuned for reliable double-tap:
  // tap_shock (0-3): maximum duration (in ODR steps) where an over-threshold event is still
  //   considered a tap. Higher tolerates longer impacts. 3 = ~max;
  // tap_quiet (0-3): quiet time after first tap during which accel must stay below threshold
  //   before second tap; balances rejection of long impacts vs responsiveness. 2 is moderate.
  // tap_dur (0-15): maximum interval (in ODR steps) between first and second tap. 8 chosen to
  //   allow natural user double taps without allowing widely spaced taps.
  .tap_shock = 0x03,
  .tap_quiet = 0x02,
  .tap_dur = 0x08,
  .default_motion_sensitivity = 85, // High
  },
  // Ideally we would configure both interrupt pins, but we have run out of GPIOTE channels.
  // We will use INT1 (connected to pin 13) for accelerometer interrupts, and leave INT2 (pin 11) unused.
  .accel_int_gpios = {
    [0] = { .gpio = NRF5_GPIO_RESOURCE_EXISTS, .gpio_pin = NRF_GPIO_PIN_MAP(1, 13) },
  },
  .accel_ints = {
    [0] = { .peripheral = NRFX_GPIOTE_INSTANCE(0), .channel = 7, .gpio_pin = NRF_GPIO_PIN_MAP(1, 13) },
  },
};

static const BoardConfigMag BOARD_CONFIG_MAG = {
  .mag_config = {
    .axes_offsets[AXIS_X] = 1,
    .axes_offsets[AXIS_Y] = 0,
    .axes_offsets[AXIS_Z] = 2,
    .axes_inverts[AXIS_X] = false,
    .axes_inverts[AXIS_Y] = true,
    .axes_inverts[AXIS_Z] = false,
  },
};

extern UARTDevice * const DBG_UART;

extern PwmState BACKLIGHT_PWM_STATE;
static const BoardConfigBacklight BOARD_CONFIG_BACKLIGHT = {
  .options = ActuatorOptions_Pwm,
  .pwm = {
    .state = &BACKLIGHT_PWM_STATE,
    .output = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 8), true },
    .peripheral = NRFX_PWM_INSTANCE(0)
  },
};

static const BoardConfigSharpDisplay BOARD_CONFIG_DISPLAY = {
  .spi = NRFX_SPIM_INSTANCE(3),

  .clk = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 26), true },
  .mosi = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 27), true },
  .cs = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 5), true },

  .on_ctrl = { NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 7), true },
};

#define TOUCH_PIN_RST NRF_GPIO_PIN_MAP(1, 3)
#define TOUCH_PIN_IRQ NRF_GPIO_PIN_MAP(1, 4)
#define TOUCH_PIN_SDA NRF_GPIO_PIN_MAP(1, 1)
#define TOUCH_PIN_SCL NRF_GPIO_PIN_MAP(1, 2)

static const ExtiConfig BOARD_CONFIG_TOUCH_EXTI = {
  .peripheral = NRFX_GPIOTE_INSTANCE(0),
  .channel = 2,
  .gpio_pin = TOUCH_PIN_IRQ,
};

extern QSPIPort * const QSPI;
extern QSPIFlash * const QSPI_FLASH;

extern MicDevice * const MIC;

extern I2CSlavePort * const I2C_DRV2604;
extern I2CSlavePort * const I2C_OPT3001;
extern I2CSlavePort * const I2C_DA7212;
extern I2CSlavePort * const I2C_MMC5603NJ;
extern I2CSlavePort * const I2C_BMP390;
extern I2CSlavePort * const I2C_LSM6D;
