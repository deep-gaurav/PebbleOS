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

// 3-bit color mode constants
// 3-bit mode header (6 bits sent MSB first): M0=1, M1=toggle, M2=0, M3=0, M4=0, M5=0
// 1-bit mode is 0x88 (M0=1, M1=0, M2=0, M3=0, M4=1).
// So for 3-bit: M0=1, M4=0 gives 0b10000000 = 0x80 base, plus M1 in bit 6.
static const unsigned int DISP_MODE_3BIT_BASE = 0x80; // M1=0, M2-M4=0

// We want the SPI clock to run at 2MHz by default
static uint32_t s_spi_clock_hz;

static bool s_initialized = false;

static volatile int s_spidma_waiting = 0;
static volatile int s_spidma_immediate = 0;

// DMA state
static DisplayContext s_display_context;
// DMA buffer: 1-bit needs 26B (22 data + 4 header+dummy), 3-bit needs 88B (66 data + 4 header + 16 dummy)
// Use the larger size always
static uint8_t s_dma_line_buffer[96];

// Frame counter for 3-bit M1 COM inversion toggling (toggle every other frame)
static uint32_t s_3bit_frame_count;

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

//! Convert a row of 8-bit GColor8 pixels to 3-bit packed format for the JDI display.
//! Input: 176 bytes (GColor8, 2 bits per R/G/B/A)
//! Output: 66 bytes packed as RGB triplets (R=MSB, B=LSB per pixel)
static void prv_convert_row_to_3bit(const uint8_t *src, uint8_t *dst);

//! Diagnostic: Convert GColor8 to 1-bit using red channel MSB.
//! Uses the WORKING 1-bit display protocol to test if GColor8 parsing is correct.
//! Input: 176 bytes (GColor8)
//! Output: 22 bytes (1-bit, 8 pixels per byte, bit 0 = leftmost pixel)
//! Note: reverse_byte() is applied separately (like the 1-bit path)
static void prv_convert_row_to_1bit_from_8bit(const uint8_t *src, uint8_t *dst) {
  // 176 pixels / 8 = 22 bytes
  for (uint16_t i = 0; i < DISP_LINE_BYTES; i++) {
    uint8_t byte = 0;
    // Pack 8 GColor8 pixels into 8 bits (using red channel MSB)
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t color = src[i * 8 + j];
      // GColor8: AA_RR_GG_BB, red is bits 5-4, MSB is bit 5
      uint8_t bit = (color >> 5) & 1;
      // Put pixel j's bit into bit j of the output byte
      // (no reversal here - reverse_byte() is called in the 1-bit path)
      byte |= (bit << j);
    }
    dst[i] = byte;
  }
}

static void prv_convert_row_to_3bit(const uint8_t *src, uint8_t *dst) {
  // Clear output (important since we OR bits)
  memset(dst, 0, DISP_LINE_BYTES_3BIT);

  for (uint16_t i = 0; i < DISP_COLS; i++) {
    uint8_t color = src[i];
    // GColor8 format: AA_RR_GG_BB (bits 7-6=A, 5-4=R, 3-2=G, 1-0=B)
    // 3-bit mode: 1 bit per channel, 0 = off, 1 = on
    uint8_t r = (color >> 5) & 1; // R occupies bits 5-4, MSB is bit 5
    uint8_t g = (color >> 3) & 1; // G occupies bits 3-2, MSB is bit 3
    uint8_t b = (color >> 1) & 1; // B occupies bits 1-0, MSB is bit 1

    // Pack into output: pixel i occupies bits 7-5 of byte (3*i/8), 3*i%8, 3*i%8-1
    uint16_t byte_index = (3 * i) / 8;
    uint8_t bit_base = 7 - ((3 * i) % 8);

    dst[byte_index] |= (r << bit_base);
    dst[byte_index] |= (g << (bit_base - 1));
    dst[byte_index] |= (b << (bit_base - 2));
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
  // Diagnostic: 8-bit GColor8 -> 1-bit conversion using working 1-bit display protocol
  // This tests if the GColor8 parsing and row addressing works correctly
  switch (s_display_context.state) {
  case DISPLAY_STATE_IDLE:
  {
    if (is_end_of_buffer) {
      return false;
    }

    prv_enable_chip_select();
    s_display_context.state = DISPLAY_STATE_WRITING;

    // Convert 8-bit GColor8 to 1-bit (using red channel MSB as the B/W value)
    uint8_t converted_data[DISP_LINE_BYTES];
    prv_convert_row_to_1bit_from_8bit(r.data, converted_data);

    // Apply reverse_byte() to each byte (same as 1-bit path does)
    for (int i = 0; i < DISP_LINE_BYTES; i++) {
      s_dma_line_buffer[2 + i] = reverse_byte(converted_data[i]);
    }

    // Use 1-bit mode header but with direct row address (no inversion like 1-bit path)
    // The 8-bit framebuffer has rows in opposite orientation vs 1-bit
    s_dma_line_buffer[0] = DISP_MODE_WRITE;
    s_dma_line_buffer[1] = r.address + 1; // Direct mapping for 8-bit framebuffer

    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES + 2);

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

    // Convert 8-bit GColor8 to 1-bit and apply reverse_byte
    uint8_t converted_data[DISP_LINE_BYTES];
    prv_convert_row_to_1bit_from_8bit(r.data, converted_data);
    for (int i = 0; i < DISP_LINE_BYTES; i++) {
      s_dma_line_buffer[2 + i] = reverse_byte(converted_data[i]);
    }

    s_dma_line_buffer[0] = DISP_MODE_WRITE;
    s_dma_line_buffer[1] = r.address + 1; // Direct mapping for 8-bit framebuffer

    bool lastLine = r.address == (DISP_ROWS - 1);
    if (lastLine) {
      s_dma_line_buffer[DISP_LINE_BYTES + 2] = 0;
      s_dma_line_buffer[DISP_LINE_BYTES + 3] = 0;
    }
    prv_display_write_async(s_dma_line_buffer, DISP_LINE_BYTES + (lastLine ? 4 : 2));
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
}

void display_set_rotated(bool rotated) {
}

void display_update_boot_frame(uint8_t *framebuffer) {
}

void display_show_panic_screen(uint32_t error_code) {
}

void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) { return GPointZero; }
