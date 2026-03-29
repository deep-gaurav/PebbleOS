#include <nrfx_i2s.h>

#include "board/board.h"
#include "lib/log_buffer.h"
#include "drivers/flash/qspi_flash_definitions.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/i2c_definitions.h"
#include "drivers/mic.h"
#include "drivers/mic/nrf5/pdm_definitions.h"
#include "drivers/nrf5/i2c_hal_definitions.h"
#include "drivers/nrf5/spi_definitions.h"
#include "drivers/nrf5/uart_definitions.h"
#include "drivers/pmic/npm1300.h"
#include "drivers/pwm.h"
#include "drivers/qspi_definitions.h"
#include "drivers/button_id.h"
#include "drivers/exti.h"
#include "drivers/rtc.h"
#include "flash_region/flash_region.h"
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "services/common/system_task.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/units.h"

// QSPI
#include <hal/nrf_clock.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_qspi.h>
#include <nrfx_spim.h>
#include <nrfx_twim.h>
#include <nrfx_pdm.h>

static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .clk_freq_hz = 32000000UL,
    .cs_gpio = NRF_GPIO_PIN_MAP(0, 14),
    .clk_gpio = NRF_GPIO_PIN_MAP(0, 16),
    .data_gpio =
        {
            NRF_GPIO_PIN_MAP(0, 15),
            NRF_GPIO_PIN_MAP(0, 13),
            NRF_GPIO_PIN_MAP(1, 10),
            NRF_GPIO_PIN_MAP(1, 11),
        },
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
    .default_fast_read_ddr_enabled = false,
    .read_mode = QSPI_FLASH_READ_READ2IO,
    .write_mode = QSPI_FLASH_WRITE_PP,
    .reset_gpio = { GPIO_Port_NULL },
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;
/* PERIPHERAL ID 43 */

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx_gpio = NRF_GPIO_PIN_MAP(1, 11),
    .rx_gpio = NRF_GPIO_PIN_MAP(1, 10),
    .rts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .cts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .periph = NRFX_UARTE_INSTANCE(0),
    .counter = NRFX_TIMER_INSTANCE(2),
};
UARTDevice *const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP_NRFX(UART0_UARTE0, nrfx_uarte_0_irq_handler);
/* PERIPHERAL ID 8 */

/* buttons */
IRQ_MAP_NRFX(TIMER1, nrfx_timer_1_irq_handler);
IRQ_MAP_NRFX(TIMER2, nrfx_timer_2_irq_handler);

/* display */
PwmState DISPLAY_EXTCOMIN_STATE;
IRQ_MAP_NRFX(SPIM3, nrfx_spim_3_irq_handler);

/* PERIPHERAL ID 10 */

/* EXTI */
IRQ_MAP_NRFX(GPIOTE, nrfx_gpiote_0_irq_handler);

/* nPM1300 */
static I2CBusState I2C_NPMC_IIC1_BUS_STATE = {};

static const I2CBusHal I2C_NPMC_IIC1_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(1),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_NPMC_IIC1_BUS = {
    .state = &I2C_NPMC_IIC1_BUS_STATE,
    .hal = &I2C_NPMC_IIC1_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 14),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 15),
        },
    .name = "I2C_NPMC_IIC1",
};
IRQ_MAP_NRFX(SPI1_SPIM1_SPIS1_TWI1_TWIM1_TWIS1, nrfx_twim_1_irq_handler);
/* PERIPHERAL ID 9 */

static const I2CSlavePort I2C_SLAVE_NPM1300 = {
    .bus = &I2C_NPMC_IIC1_BUS,
    .address = 0x6B << 1,
};

I2CSlavePort *const I2C_NPM1300 = &I2C_SLAVE_NPM1300;

/* peripheral I2C bus */
static I2CBusState I2C_IIC2_BUS_STATE = {};

static const I2CBusHal I2C_IIC2_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(0),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_IIC2_BUS = {
    .state = &I2C_IIC2_BUS_STATE,
    .hal = &I2C_IIC2_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 25),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 11),
        },
    .name = "I2C_IIC2",
};
IRQ_MAP_NRFX(SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0, nrfx_twim_0_irq_handler);

static const I2CSlavePort I2C_SLAVE_DRV2604 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x5A << 1,
};

I2CSlavePort *const I2C_DRV2604 = &I2C_SLAVE_DRV2604;

static const I2CSlavePort I2C_SLAVE_OPT3001 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x44 << 1,
};

I2CSlavePort *const I2C_OPT3001 = &I2C_SLAVE_OPT3001;

static const I2CSlavePort I2C_SLAVE_DA7212 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x1A << 1,
};

I2CSlavePort *const I2C_DA7212 = &I2C_SLAVE_DA7212;

static const I2CSlavePort I2C_SLAVE_MMC5603NJ = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x30 << 1,
};

I2CSlavePort *const I2C_MMC5603NJ = &I2C_SLAVE_MMC5603NJ;

static const I2CSlavePort I2C_SLAVE_BMP390 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x76 << 1,
};

I2CSlavePort *const I2C_BMP390 = &I2C_SLAVE_BMP390;

static const I2CSlavePort I2C_SLAVE_LSM6D = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x6A << 1,
};

I2CSlavePort *const I2C_LSM6D = &I2C_SLAVE_LSM6D;

IRQ_MAP_NRFX(I2S, nrfx_i2s_0_irq_handler);

IRQ_MAP_NRFX(PDM, NRFX_PDM_INST_HANDLER_GET(0));

/* PERIPHERAL ID 11 */

/* Microphone */
static MicDeviceState s_mic_state_storage;
static MicDevice s_mic_device = {
  .state = &s_mic_state_storage,
  .pdm_instance = NRFX_PDM_INSTANCE(0),
  .clk_pin = NRF_GPIO_PIN_MAP(1, 0),   // P1.00 - PDM CLK
  .data_pin = NRF_GPIO_PIN_MAP(0, 24), // P0.24 - PDM DATA
  .channels = 1,
};
MicDevice * const MIC = &s_mic_device;

/* sensor SPI bus */

/* asterix shares SPI with flash, which we don't support */

PwmState BACKLIGHT_PWM_STATE;
IRQ_MAP_NRFX(PWM0, nrfx_pwm_0_irq_handler);

IRQ_MAP_NRFX(RTC1, rtc_irq_handler);

const Npm1300Config NPM1300_CONFIG = {
  // 128mA = ~1C (rapid charge)
  .chg_current_ma = 128,
  .dischg_limit_ma = 200,
  .term_current_pct = 10,
  .thermistor_beta = 3380,
};

// Touch controller CST816S with software I2C
#define TOUCH_I2C_TIMEOUT 100000
#define TOUCH_I2C_ADDR 0x15

static bool s_touch_i2c_started = false;

static void touch_wr_pin(int pin, bool state) {
  if (state) {
    nrf_gpio_pin_set(pin);
    nrf_gpio_cfg_output(pin);
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
  } else {
    nrf_gpio_pin_clear(pin);
    nrf_gpio_cfg_output(pin);
  }
}

static bool touch_rd_pin(int pin) {
  return nrf_gpio_pin_read(pin);
}

static void touch_i2c_start(void) {
  if (s_touch_i2c_started) {
    touch_wr_pin(TOUCH_PIN_SDA, true);
    touch_wr_pin(TOUCH_PIN_SCL, true);
    int timeout = TOUCH_I2C_TIMEOUT;
    while (!touch_rd_pin(TOUCH_PIN_SCL) && --timeout) {
    }
  }
  touch_wr_pin(TOUCH_PIN_SDA, false);
  touch_wr_pin(TOUCH_PIN_SCL, false);
  s_touch_i2c_started = true;
}

static void touch_i2c_stop(void) {
  touch_wr_pin(TOUCH_PIN_SDA, false);
  touch_wr_pin(TOUCH_PIN_SCL, true);
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!touch_rd_pin(TOUCH_PIN_SCL) && --timeout) {
  }
  touch_wr_pin(TOUCH_PIN_SDA, true);
  s_touch_i2c_started = false;
}

static void touch_i2c_wr_bit(bool bit) {
  touch_wr_pin(TOUCH_PIN_SDA, bit);
  touch_wr_pin(TOUCH_PIN_SCL, true);
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!touch_rd_pin(TOUCH_PIN_SCL) && --timeout) {
  }
  touch_wr_pin(TOUCH_PIN_SCL, false);
  touch_wr_pin(TOUCH_PIN_SDA, true);
}

static bool touch_i2c_rd_bit(void) {
  touch_wr_pin(TOUCH_PIN_SDA, true);
  touch_wr_pin(TOUCH_PIN_SCL, true);
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!touch_rd_pin(TOUCH_PIN_SCL) && --timeout) {
  }
  bool bit = touch_rd_pin(TOUCH_PIN_SDA);
  touch_wr_pin(TOUCH_PIN_SCL, false);
  return bit;
}

static bool touch_i2c_wr(uint8_t data) {
  for (int i = 0; i < 8; i++) {
    touch_i2c_wr_bit(data & 128);
    data <<= 1;
  }
  return !touch_i2c_rd_bit();
}

static uint8_t touch_i2c_rd(bool nack) {
  int data = 0;
  for (int i = 0; i < 8; i++) {
    data = (data << 1) | (touch_i2c_rd_bit() ? 1 : 0);
  }
  touch_i2c_wr_bit(nack);
  return data;
}

static void touch_read(uint8_t addr, uint8_t cnt, uint8_t *data) {
  touch_i2c_start();
  touch_i2c_wr(TOUCH_I2C_ADDR << 1);
  touch_i2c_wr(addr);
  touch_i2c_start();
  touch_i2c_wr(1 | (TOUCH_I2C_ADDR << 1));
  for (int i = 0; i < cnt; i++) {
    data[i] = touch_i2c_rd(i == (cnt - 1));
  }
  touch_i2c_stop();
  touch_wr_pin(TOUCH_PIN_SDA, true);
  touch_wr_pin(TOUCH_PIN_SCL, true);
}

static void prv_button_press_short(ButtonId button) {
  PebbleEvent e = {
    .type = PEBBLE_BUTTON_DOWN_EVENT,
    .button.button_id = button,
  };
  event_put(&e);
  e = (PebbleEvent){
    .type = PEBBLE_BUTTON_UP_EVENT,
    .button.button_id = button,
  };
  event_put(&e);
}

static void prv_touch_sys_task_callback(void *data) {
  uint8_t buf[6];
  touch_read(1, 6, buf);
  int gesture = buf[0];
  PBL_LOG_INFO("Touch IRQ: %d %d %d %d %d %d", buf[0], buf[1], buf[2],
               buf[3], buf[4], buf[5]);
  static int last_gesture = 0;
  if (gesture != last_gesture) {
    last_gesture = gesture;
    PBL_LOG_INFO("Gesture: %d", gesture);
    switch (gesture) {
      case 1:
        prv_button_press_short(BUTTON_ID_DOWN);
        break;
      case 2:
        prv_button_press_short(BUTTON_ID_UP);
        break;
      case 3:
        prv_button_press_short(BUTTON_ID_BACK);
        break;
      case 4:
        prv_button_press_short(BUTTON_ID_SELECT);
        break;
    }
  }
  (void)data;
}

static void touch_interrupt_handler(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_touch_sys_task_callback, NULL, should_context_switch);
}

void board_early_init(void) {
  log_init();
  log_write("board_early_init: log buffer initialized\r\n");

  NRF_NVMC->ICACHECNF |= NVMC_ICACHECNF_CACHEEN_Msk;

  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
  /* TODO: Add timeout, report failure if LFCLK does not start. For now,
   * WDT should trigger a reboot. Calibrated RC may be used as a fallback,
   * provided we can adjust BLE SCA settings at runtime.
   */
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
}

void board_init(void) {
  nrf_gpio_pin_set(TOUCH_PIN_SDA);
  nrf_gpio_pin_set(TOUCH_PIN_SCL);
  nrf_gpio_cfg_output(TOUCH_PIN_SDA);
  nrf_gpio_cfg_output(TOUCH_PIN_SCL);
  nrf_gpio_cfg_input(TOUCH_PIN_IRQ, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_pin_clear(TOUCH_PIN_RST);
  nrf_gpio_cfg_output(TOUCH_PIN_RST);

  for (volatile int i = 0; i < 48000; i++)
    ;
  nrf_gpio_pin_set(TOUCH_PIN_RST);
  for (volatile int i = 0; i < 480000; i++)
    ;

  uint8_t chip_id = 0;
  touch_read(0xA7, 1, &chip_id);
  PBL_LOG_INFO("Touch chip ID: 0x%02X", chip_id);

  uint8_t fw_ver = 0;
  touch_read(0xA9, 1, &fw_ver);
  PBL_LOG_INFO("Touch fw version: 0x%02X", fw_ver);

  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
  touch_read(1, 6, buf);
  PBL_LOG_INFO("Touch init: %d %d %d %d %d %d", buf[0], buf[1], buf[2],
               buf[3], buf[4], buf[5]);

  exti_configure_pin(BOARD_CONFIG_TOUCH_EXTI, ExtiTrigger_Falling, touch_interrupt_handler);
  exti_enable(BOARD_CONFIG_TOUCH_EXTI);
  PBL_LOG_INFO("Touch IRQ enabled");
}
