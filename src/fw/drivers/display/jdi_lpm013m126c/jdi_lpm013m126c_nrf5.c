#include "jdi_lpm013m126c.h"

#include "applib/graphics/gtypes.h"
#include "board/board.h"
#include "debug/power_tracking.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "drivers/spi.h"
#include "kernel/util/sleep.h"
#include "kernel/util/stop.h"
#include "os/tick.h"
#include "services/common/analytics/analytics.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/bitset.h"
#include "util/net.h"
#include "util/reverse.h"
#include "util/units.h"


#define NRF5_COMPATIBLE
#include <mcu.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// GPIO constants
static const unsigned int DISP_MODE_STATIC = 0x00;
static const unsigned int DISP_MODE_WRITE = 0b10001000; // 1 bit data mode
static const unsigned int DISP_MODE_CLEAR = 0x20;

// 4-bit color mode: M0=1, M2=1 → 0x90. VCOM polarity toggle on bit 6 (0xD0)
#define DISP_MODE_COLOR 0x90

// We want the SPI clock to run at 2MHz by default
static uint32_t s_spi_clock_hz;

static bool s_initialized = false;
static bool s_display_enabled = false;

static volatile int s_spidma_waiting = 0;
static volatile int s_spidma_immediate = 0;

// DMA state
static DisplayContext s_display_context;
// DMA buffer: 4-bit needs 92B (88 data + 2 header + 2 dummy), 1-bit needs 26B
static uint8_t s_dma_line_buffer[DISP_4BIT_DMA_BUFFER_SIZE_BYTES];

// Frame counter for VCOM polarity toggling (toggle every other frame)
static uint32_t s_frame_count;

static SemaphoreHandle_t s_dma_update_in_progress_semaphore;

static void prv_display_context_init(DisplayContext* context);
static bool prv_do_dma_update(void);

static void prv_enable_chip_select(void) {
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, true);
  // setup time > 3us
  // this produces a setup time of ~7us
  for (volatile int i = 0; i < 32; i++);
}

static void prv_disable_chip_select(void) {
  // delay while last byte is emitted by the SPI peripheral (~7us)
  for (volatile int i = 0; i < 48; i++);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, false);
  // hold time > 1us
  // this produces a delay of ~3.5us
  for (volatile int i = 0; i < 16; i++);
}

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx) {
  s_spidma_waiting = 0;
  if (!s_spidma_immediate) {
    bool needs_switch = prv_do_dma_update();
    portEND_SWITCHING_ISR(needs_switch);
  }
}

static void prv_display_write_sync(const uint8_t *buf, size_t len);

static void prv_display_start(void) {
  periph_config_acquire_lock();

  if (s_initialized) {
    nrfx_spim_uninit(&BOARD_CONFIG_DISPLAY.spi);
  }

  // PBL_LOG(LOG_LEVEL_ALWAYS, "LCD prv_display_start");

  gpio_output_init(&BOARD_CONFIG_DISPLAY.cs, GPIO_OType_PP, GPIO_Speed_50MHz);

  nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
    BOARD_CONFIG_DISPLAY.clk.gpio_pin,
    BOARD_CONFIG_DISPLAY.mosi.gpio_pin,
    NRF_SPIM_PIN_NOT_CONNECTED,
    NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = NRFX_MHZ_TO_HZ(2);

  /* spim4 has hardware SS but it is tricky to convince NRFX to expose it to
   * us; for now, we use the classic enable chip select mechanism */
#if 0
  config.use_hw_ss = 1;
  config.ss_duration = 256; /* 4 us * 64MHz */
#endif

  nrfx_err_t err = nrfx_spim_init(&BOARD_CONFIG_DISPLAY.spi, &config, prv_spim_evt_handler, NULL);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  gpio_output_init(&BOARD_CONFIG_DISPLAY.on_ctrl,
                   BOARD_CONFIG_DISPLAY.on_ctrl_otype,
                   GPIO_Speed_50MHz);

  if (BOARD_CONFIG.lcd_com.gpio) {
    gpio_output_init(&BOARD_CONFIG.lcd_com, GPIO_OType_PP, GPIO_Speed_50MHz);
  }

  // +5V to LCD_DISP pin (Set this pin low to turn off the display)
  gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, true);


  // debug
#if 0
  gpio_output_init(&BOARD_CONFIG_DISPLAY.clk, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_init(&BOARD_CONFIG_DISPLAY.mosi, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.clk, true);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.mosi, true);
  prv_enable_chip_select();
  uint8_t buf[24] = { DISP_MODE_WRITE, 0x00 };
  for (int i=0;i<176;i++) {
    buf[1] = i+1;
    buf[2+(i>>3)] |= 128 >> (i&7);
    prv_display_write_sync(buf, sizeof(buf));
  }
  buf[0]=0;
  buf[1]=0;
  prv_display_write_sync(buf, 2);
  prv_disable_chip_select();
#endif

  periph_config_release_lock();
}

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  // Take the semaphore so that we can be sure that we are not interrupting a transfer
  xSemaphoreTake(s_dma_update_in_progress_semaphore, portMAX_DELAY);

  uint32_t old_spi_clock_hz = s_spi_clock_hz;
  s_spi_clock_hz = new_frequency_hz;
  prv_display_start();

  xSemaphoreGive(s_dma_update_in_progress_semaphore);
  return old_spi_clock_hz;
}

void display_init(void) {

  if (s_initialized) {
    return;
  }

  s_spi_clock_hz = MHZ_TO_HZ(2);

  prv_display_context_init(&s_display_context);

  vSemaphoreCreateBinary(s_dma_update_in_progress_semaphore);

  prv_display_start();

  s_initialized = true;
}

static void prv_display_context_init(DisplayContext* context) {
  context->state = DISPLAY_STATE_IDLE;
  context->get_next_row = NULL;
  context->complete = NULL;
}

//! Convert a row of 8-bit GColor8 pixels to 4-bit packed format for the JDI display.
//! Input: 176 bytes (GColor8, 2 bits per R/G/B/A)
//! Output: 88 bytes (2 pixels per byte, nibble: bit3=R, bit2=G, bit1=B, bit0=unused)
static void prv_convert_row_to_4bit(const uint8_t *src, uint8_t *dst) {
  for (uint16_t i = 0; i < DISP_COLS; i += 2) {
    uint8_t color1 = src[i];
    uint8_t color2 = src[i + 1];
    // GColor8: AA_RR_GG_BB. Extract MSB of each channel.
    uint8_t hi = ((color1 >> 5) & 1) << 3 |  // R
                 ((color1 >> 3) & 1) << 2 |  // G
                 ((color1 >> 1) & 1) << 1;   // B
    uint8_t lo = ((color2 >> 5) & 1) << 3 |
                 ((color2 >> 3) & 1) << 2 |
                 ((color2 >> 1) & 1) << 1;
    dst[i / 2] = (hi << 4) | lo;
  }
}

static void prv_display_write_async(const uint8_t *buf, size_t len) {
  nrfx_spim_xfer_desc_t desc = {
    .p_tx_buffer = buf,
    .tx_length = len
  };

  PBL_ASSERTN(!s_spidma_waiting);

  s_spidma_waiting = 1;
  s_spidma_immediate = 0;

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
}

static void prv_display_write_sync(const uint8_t *buf, size_t len) {
#if 0 // debug bit-banging (requires SPI to be deinited)
  for (size_t i=0;i<len;i++) {
    for (int b=7;b>=0;b--) {
      gpio_output_set(&BOARD_CONFIG_DISPLAY.mosi, (buf[i]>>b)&1);
      for (volatile int i = 0; i < 32; i++);
      gpio_output_set(&BOARD_CONFIG_DISPLAY.clk, true);
      for (volatile int i = 0; i < 32; i++);
      gpio_output_set(&BOARD_CONFIG_DISPLAY.clk, false);
      for (volatile int i = 0; i < 32; i++);
    }
  }
#else
  nrfx_spim_xfer_desc_t desc = {
    .p_tx_buffer = buf,
    .tx_length = len
  };

  PBL_ASSERTN(!s_spidma_waiting);

  s_spidma_waiting = 1;
  s_spidma_immediate = 1;

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  while (s_spidma_waiting)
    /* XXX: ... yield, or something.  maybe a semaphore would be nicer here.  it should be fast, though */;
  s_spidma_immediate = 0;
#endif
}

// Clear-all mode is entered by sending 0x04 to the panel
void display_clear(void) {
  uint8_t buf[] = { DISP_MODE_CLEAR, 0x00 };
  prv_display_write_sync(buf, sizeof(buf));
}

bool display_update_in_progress(void) {
  if (xSemaphoreTake(s_dma_update_in_progress_semaphore, 0) == pdPASS) {
    xSemaphoreGive(s_dma_update_in_progress_semaphore);
    return false;
  }
  return true;
}

// Static mode is entered by sending 0x00 to the panel
static void prv_display_enter_static(void) {
  uint8_t buf[] = { DISP_MODE_STATIC, 0x00, 0x00 };
  prv_enable_chip_select();
  prv_display_write_sync(buf, sizeof(buf));
  prv_disable_chip_select();
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  PBL_ASSERTN(nrcb != NULL);
  PBL_ASSERTN(uccb != NULL);
  stop_mode_disable(InhibitorDisplay);
  xSemaphoreTake(s_dma_update_in_progress_semaphore, portMAX_DELAY);

  prv_display_context_init(&s_display_context);
  s_display_context.get_next_row = nrcb;
  s_display_context.complete = uccb;

  /* // SYNC update
  prv_enable_chip_select();
  uint8_t buf[24] = { DISP_MODE_WRITE, 0x00 };
  for (int i=0;i<176;i++) {
    DisplayRow r;
    s_display_context.get_next_row(&r);
    buf[1] = r.address;
    for (int p=0;p<22;p++)
      buf[2+p] = reverse_byte(r.data[p]);

    prv_display_write_sync(buf, sizeof(buf));
  }
  buf[0]=0;
  buf[1]=0;
  prv_display_write_sync(buf, 2);
  prv_disable_chip_select();
  s_display_context.complete();*/

  prv_do_dma_update();

  // Block while we wait for the update to finish.
  TickType_t ticks = milliseconds_to_ticks(4000); // DMA should be fast
  if (xSemaphoreTake(s_dma_update_in_progress_semaphore, ticks) != pdTRUE) {
    uint32_t pri_mask = __get_PRIMASK();
    PBL_CROAK("display DMA failed: 0x%" PRIx32, pri_mask);
  }

  // needs to not happen from the ISR, because write_sync depends on the ISR to be called again
  uint8_t buf[] = { 0x00 };
  prv_display_write_sync(buf, sizeof(buf));
  prv_disable_chip_select();

  xSemaphoreGive(s_dma_update_in_progress_semaphore);
  stop_mode_enable(InhibitorDisplay);
}

void display_pulse_vcom(void) {
  // Skip VCOM pulse if display is powered off (memory LCD retains state without power)
  if (!s_display_enabled) {
    return;
  }
  PBL_ASSERTN(BOARD_CONFIG.lcd_com.gpio != 0);
  gpio_output_set(&BOARD_CONFIG.lcd_com, true);
  // the spec requires at least 1us; this provides ~2 so should be safe
  for (volatile int i = 0; i < 8; i++);
  gpio_output_set(&BOARD_CONFIG.lcd_com, false);
}

#if DISPLAY_ORIENTATION_ROTATED_180
//!
//! memcpy the src buffer to dst and reverse the bits
//! to match the display order
//!
static void prv_memcpy_reverse_bytes(uint8_t* dst, uint8_t* src, int bytes) {
    // Skip the mode selection and column address bytes
    dst+=2;
    while (bytes--) {
        *dst++ = reverse_byte(*src++);
    }
}
#else
//!
//! memcpy the src buffer to dst backwards (i.e. the highest src byte
//! is the lowest byte in dst.
//!
static void prv_memcpy_backwards(uint32_t* dst, uint32_t* src, int length) {
  dst += length - 1;
  while (length--) {
    *dst-- = ntohl(*src++);
  }
}
#endif


static bool prv_do_dma_update(void) {
  DisplayRow r;

  PBL_ASSERTN(s_display_context.get_next_row != NULL);
  bool is_end_of_buffer = !s_display_context.get_next_row(&r);

#if PBL_COLOR
  // 4-bit color path
  switch (s_display_context.state) {
  case DISPLAY_STATE_IDLE:
  {
    if (is_end_of_buffer) {
      return false;
    }

    prv_enable_chip_select();
    s_display_context.state = DISPLAY_STATE_WRITING;
    s_frame_count = 0;

    // 4-bit color mode command with VCOM polarity
    uint8_t polarity = (s_frame_count & 1) ? 0x40 : 0;
    s_dma_line_buffer[0] = DISP_MODE_COLOR | polarity;
    s_dma_line_buffer[1] = r.address + 1;

    prv_convert_row_to_4bit(r.data, &s_dma_line_buffer[2]);

    // Send: 2 header + 88 data = 90 bytes (no dummies between rows)
    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES_4BIT + 2);

    break;
  }
  case DISPLAY_STATE_WRITING:
  {
    if (is_end_of_buffer) {
      s_display_context.complete();
      s_frame_count++;

      signed portBASE_TYPE was_higher_priority_task_woken = pdFALSE;
      xSemaphoreGiveFromISR(s_dma_update_in_progress_semaphore, &was_higher_priority_task_woken);

      return was_higher_priority_task_woken != pdFALSE;
    }

    uint8_t polarity = (s_frame_count & 1) ? 0x40 : 0;
    s_dma_line_buffer[0] = DISP_MODE_COLOR | polarity;
    s_dma_line_buffer[1] = r.address + 1;

    prv_convert_row_to_4bit(r.data, &s_dma_line_buffer[2]);

    // Last row gets 2 trailing zeros; all others get just header + data
    bool lastLine = (r.address == (DISP_ROWS - 1));
    if (lastLine) {
      s_dma_line_buffer[DISP_LINE_BYTES_4BIT + 2] = 0;
      s_dma_line_buffer[DISP_LINE_BYTES_4BIT + 3] = 0;
    }
    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES_4BIT + (lastLine ? 4 : 2));
    break;
  }
  default:
    WTF;
  }

#else
  // 1-bit BW path (original code)
  switch (s_display_context.state) {
  case DISPLAY_STATE_IDLE:
  {
    if (is_end_of_buffer) {
      // If nothing has been modified, bail out early
      return false;
    }

    prv_enable_chip_select();

    s_display_context.state = DISPLAY_STATE_WRITING;

#if DISPLAY_ORIENTATION_ROTATED_180
    prv_memcpy_reverse_bytes(&s_dma_line_buffer[0], r.data, DISP_LINE_BYTES);
    s_dma_line_buffer[1] = r.address + 1;
#else
    prv_memcpy_backwards((uint32_t*)&s_dma_line_buffer[0], (uint32_t*)r.data, DISP_LINE_WORDS);
    s_dma_line_buffer[1] = 175 - r.address + 1;
#endif
    s_dma_line_buffer[0] = DISP_MODE_WRITE;
    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES+2);

    break;
  }
  case DISPLAY_STATE_WRITING:
  {
    if (is_end_of_buffer) {
      s_display_context.complete();

      signed portBASE_TYPE was_higher_priority_task_woken = pdFALSE;
      xSemaphoreGiveFromISR(s_dma_update_in_progress_semaphore, &was_higher_priority_task_woken);

      return was_higher_priority_task_woken != pdFALSE;
    }

#if DISPLAY_ORIENTATION_ROTATED_180
    prv_memcpy_reverse_bytes(&s_dma_line_buffer[0], r.data, DISP_LINE_BYTES);
    s_dma_line_buffer[1] = r.address + 1;
#else
    prv_memcpy_backwards((uint32_t*)&s_dma_line_buffer[0], (uint32_t*)r.data, DISP_LINE_WORDS);
    s_dma_line_buffer[1] = 175 - r.address + 1;
#endif
    s_dma_line_buffer[0] = DISP_MODE_WRITE;
    bool lastLine = r.address==(DISP_ROWS-1);
    if (lastLine) {
      s_dma_line_buffer[DISP_LINE_BYTES+2] = 0; // send two final bytes to flush last time
      s_dma_line_buffer[DISP_LINE_BYTES+3] = 0;
    }
    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES+(lastLine?4:2));
    break;
  }
  default:
    WTF;
  }
#endif // PBL_COLOR
  return false;
}

void display_show_splash_screen(void) {
  // The bootloader has already drawn the splash screen for us; nothing to do!
}

void display_set_enabled(bool enabled) {
  // LCD_DISP pin: HIGH = on, LOW = off (per comment in prv_display_start)
  gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, enabled);
  s_display_enabled = enabled;
}

void display_set_rotated(bool rotated) {
}

void display_update_boot_frame(uint8_t *framebuffer) {
}

void display_show_panic_screen(uint32_t error_code) {
}

void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) { return GPointZero; }
