/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "vc31.h"

#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/rtc.h"
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "services/common/hrm/hrm_manager.h"
#include "services/common/system_task.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/attributes.h"

#include "drivers/hrm/vc31_binary/vc31_algo_adapter.h"

#include <hal/nrf_gpio.h>
#include <string.h>

#define VC31_I2C_ADDR (0x33)
#define VC31_I2C_ADDR_WRITE ((VC31_I2C_ADDR << 1) | 0)
#define VC31_I2C_ADDR_READ  ((VC31_I2C_ADDR << 1) | 1)

// VC31A registers
#define VC31A_REG_DEV_ID     0x00
#define VC31A_REG_STATUS     0x01
#define VC31A_REG_PPG        0x02
#define VC31A_REG_CURRENT    0x04
#define VC31A_REG_PRE        0x06
#define VC31A_REG_PS         0x08
#define VC31A_REG_ENV        0x0A
#define VC31A_REG_CTRL       0x20
#define VC31A_REG_PPG_DIV    0x21
#define VC31A_REG_PS_DIV     0x23
#define VC31A_REG_TIA_WAIT   0x24
#define VC31A_REG_AMP_WAIT   0x25
#define VC31A_REG_GREEN_WAIT 0x26
#define VC31A_REG_GREEN_IR_GAP 0x27
#define VC31A_REG_IR_WAIT    0x28
#define VC31A_REG_GREEN_ADJ  0x29

// VC31A CTRL bits
#define VC31A_CTRL_WORK_MODE      0x80
#define VC31A_CTRL_ENABLE_PPG     0x40
#define VC31A_CTRL_ENABLE_PRE     0x20
#define VC31A_CTRL_LDO_EXTERN     0x10
#define VC31A_CTRL_INT_DIR_RISING 0x03
#define VC31A_CTRL_OPA_GAIN_25    0x01

// VC31A STATUS bits
#define VC31A_STATUS_D_ENV_OK 0x10
#define VC31A_STATUS_D_PS_OK  0x08
#define VC31A_STATUS_D_PRE_OK 0x04
#define VC31A_STATUS_D_CUR_OK 0x02
#define VC31A_STATUS_D_PPG_OK 0x01

// VC31A GREEN_ADJ bits
#define VC31A_GREEN_ADJ_ENABLE 0x8000
#define VC31A_GREEN_ADJ_UP     0x4000

// VC31A constants
#define VC31A_PPG_DIV_50_HZ  0x0287
#define VC31A_PPG_DIV_100_HZ 0x013F
#define VC31A_ENV_LIMIT      2500
#define VC31A_PS_LIMIT       350
#define VC31A_PPG_LIMIT_L    200
#define VC31A_PPG_LIMIT_H    3900
#define VC31A_CURRENT_LIMIT_L  12
#define VC31A_CURRENT_LIMIT_H  1000
#define VC31A_UNWEAR_CNT     3
#define VC31A_ISWEAR_CNT     1

// VC31B registers
#define VC31B_REG_STATUS     0x01
#define VC31B_REG_IRQ        0x02
#define VC31B_REG_FIFO_WR    0x03
#define VC31B_REG_SOFT_RESET 0x3B
#define VC31B_REG_CTRL       0x10
#define VC31B_REG_INT_EN     0x11
#define VC31B_REG13          0x12
#define VC31B_REG_FIFO_CFG   0x13
#define VC31B_REG_TIME_CAL   0x14
#define VC31B_REG_ENV_RATE   0x16
#define VC31B_REG_SLOT0_LED  0x17
#define VC31B_REG_SLOT1_LED  0x18
#define VC31B_REG_SLOT2_LED  0x19
#define VC31B_REG_SLOT0_ENV  0x1A
#define VC31B_REG_SLOT1_ENV  0x1B
#define VC31B_REG22          0x1C
#define VC31B_FIFO_BASE      0x80

// VC31B STATUS bits
#define VC31B_STATUS_OVERLOAD_MASK 0x07
#define VC31B_STATUS_INSAMPLE      0x08

// VC31B IRQ bits
#define VC31B_INT_PS   0x10
#define VC31B_INT_OV   0x08
#define VC31B_INT_FIFO 0x04
#define VC31B_INT_ENV  0x02
#define VC31B_INT_PPG  0x01

// VC31B constants
#define VC31B_PS_TH    6
#define VC31B_PPG_TH   10
#define VC31B_ADJUST_INCREASE  22   // 1.4 << 4
#define VC31B_ADJUST_DECREASE  11   // 0.7 << 4
#define VC31B_ADJUST_STEP_MAX  32
#define VC31B_ADJUST_STEP_MIN  1

#define PPG_DEBUG 1

#if PPG_DEBUG
#define PPG_DBG(...) PBL_LOG_DBG(__VA_ARGS__)
#else
#define PPG_DBG(...)
#endif

#define NORMAL_BOOT_DELAY_MS (10)

//------------------------------------------------------------------------------
// Forward declarations
//------------------------------------------------------------------------------

static void prv_enable_system_task_cb(void *context);
static void prv_enable_timer_cb(void *context);

//------------------------------------------------------------------------------
// Software I2C
//------------------------------------------------------------------------------

static void prv_i2c_delay(void) {
  for (volatile int i = 0; i < 8; i++) {
    __NOP();
  }
}

static void prv_sda_out(HRMDevice *dev, bool high) {
  if (high) {
    nrf_gpio_cfg_input(dev->sda_pin, NRF_GPIO_PIN_PULLUP);
  } else {
    nrf_gpio_pin_clear(dev->sda_pin);
    nrf_gpio_cfg_output(dev->sda_pin);
  }
}

static void prv_scl_out(HRMDevice *dev, bool high) {
  if (high) {
    nrf_gpio_cfg_input(dev->scl_pin, NRF_GPIO_PIN_PULLUP);
  } else {
    nrf_gpio_pin_clear(dev->scl_pin);
    nrf_gpio_cfg_output(dev->scl_pin);
  }
}

static bool prv_sda_rd(HRMDevice *dev) {
  return nrf_gpio_pin_read(dev->sda_pin);
}

static void prv_i2c_start(HRMDevice *dev) {
  prv_sda_out(dev, true);
  prv_scl_out(dev, true);
  prv_i2c_delay();
  prv_sda_out(dev, false);
  prv_i2c_delay();
  prv_scl_out(dev, false);
  prv_i2c_delay();
}

static void prv_i2c_stop(HRMDevice *dev) {
  prv_sda_out(dev, false);
  prv_scl_out(dev, true);
  prv_i2c_delay();
  prv_sda_out(dev, true);
  prv_i2c_delay();
}

static bool prv_i2c_write_byte(HRMDevice *dev, uint8_t data) {
  for (int i = 0; i < 8; i++) {
    prv_sda_out(dev, data & 0x80);
    data <<= 1;
    prv_i2c_delay();
    prv_scl_out(dev, true);
    prv_i2c_delay();
    prv_scl_out(dev, false);
    prv_i2c_delay();
  }
  prv_sda_out(dev, true); // release for ACK
  prv_i2c_delay();
  prv_scl_out(dev, true);
  prv_i2c_delay();
  bool ack = !prv_sda_rd(dev);
  prv_scl_out(dev, false);
  prv_i2c_delay();
  return ack;
}

static uint8_t prv_i2c_read_byte(HRMDevice *dev, bool nack) {
  uint8_t data = 0;
  prv_sda_out(dev, true); // release
  for (int i = 0; i < 8; i++) {
    prv_i2c_delay();
    prv_scl_out(dev, true);
    prv_i2c_delay();
    data = (data << 1) | (prv_sda_rd(dev) ? 1 : 0);
    prv_scl_out(dev, false);
  }
  prv_sda_out(dev, nack);
  prv_i2c_delay();
  prv_scl_out(dev, true);
  prv_i2c_delay();
  prv_scl_out(dev, false);
  prv_i2c_delay();
  prv_sda_out(dev, true); // release
  return data;
}

static bool prv_write_register(HRMDevice *dev, uint8_t reg, uint8_t value) {
  prv_i2c_start(dev);
  bool ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_WRITE);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  ack = prv_i2c_write_byte(dev, reg);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  ack = prv_i2c_write_byte(dev, value);
  prv_i2c_stop(dev);
  return ack;
}

static bool prv_read_register(HRMDevice *dev, uint8_t reg, uint8_t *value) {
  prv_i2c_start(dev);
  bool ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_WRITE);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  ack = prv_i2c_write_byte(dev, reg);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  prv_i2c_start(dev);
  ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_READ);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  *value = prv_i2c_read_byte(dev, true);
  prv_i2c_stop(dev);
  return true;
}

static bool prv_write_register_block(HRMDevice *dev, uint8_t reg,
                                      const uint8_t *data, uint32_t len) {
  prv_i2c_start(dev);
  bool ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_WRITE);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  ack = prv_i2c_write_byte(dev, reg);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  for (uint32_t i = 0; i < len; i++) {
    ack = prv_i2c_write_byte(dev, data[i]);
    if (!ack) {
      prv_i2c_stop(dev);
      return false;
    }
  }
  prv_i2c_stop(dev);
  return true;
}

static bool prv_read_register_block(HRMDevice *dev, uint8_t reg,
                                     uint8_t *data, uint32_t len) {
  prv_i2c_start(dev);
  bool ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_WRITE);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  ack = prv_i2c_write_byte(dev, reg);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  prv_i2c_start(dev);
  ack = prv_i2c_write_byte(dev, VC31_I2C_ADDR_READ);
  if (!ack) {
    prv_i2c_stop(dev);
    return false;
  }
  for (uint32_t i = 0; i < len; i++) {
    data[i] = prv_i2c_read_byte(dev, (i == len - 1));
  }
  prv_i2c_stop(dev);
  return true;
}

//------------------------------------------------------------------------------
// VC31A helpers
//------------------------------------------------------------------------------

static void prv_vc31a_adjust(HRMDevice *dev) {
  HRMDeviceState *st = dev->state;
  uint16_t adjust_param = 0;
  uint32_t adjust_step = 0;

  st->vc31a_current += 10;

  if (st->ppg_value > VC31A_PPG_LIMIT_H) {
    if (st->vc31a_current < VC31A_CURRENT_LIMIT_H) {
      adjust_step = 100; // simplified
      adjust_param = (uint16_t)adjust_step | VC31A_GREEN_ADJ_ENABLE | VC31A_GREEN_ADJ_UP;
      uint8_t adj_lo = adjust_param & 0xFF;
      uint8_t adj_hi = (adjust_param >> 8) & 0xFF;
      prv_write_register(dev, VC31A_REG_GREEN_ADJ, adj_lo);
      prv_write_register(dev, VC31A_REG_GREEN_ADJ + 1, adj_hi);
      st->was_adjusted = 2;
    }
  } else if (st->ppg_value < VC31A_PPG_LIMIT_L) {
    if (st->vc31a_current > VC31A_CURRENT_LIMIT_L) {
      adjust_step = 50; // simplified
      adjust_param = (uint16_t)adjust_step | VC31A_GREEN_ADJ_ENABLE;
      uint8_t adj_lo = adjust_param & 0xFF;
      uint8_t adj_hi = (adjust_param >> 8) & 0xFF;
      prv_write_register(dev, VC31A_REG_GREEN_ADJ, adj_lo);
      prv_write_register(dev, VC31A_REG_GREEN_ADJ + 1, adj_hi);
      st->was_adjusted = 2;
    }
  }
}

static void prv_vc31a_wearstatus(HRMDevice *dev) {
  HRMDeviceState *st = dev->state;
  if (st->is_wearing) {
    if ((st->vc31a_env >= VC31A_ENV_LIMIT) ||
        (st->vc31a_ps < st->vc31a_env + VC31A_PS_LIMIT)) {
      if (--st->un_wear_cnt <= 0) {
        st->is_wearing = false;
        st->un_wear_cnt = VC31A_UNWEAR_CNT;
        st->is_wear_cnt = VC31A_ISWEAR_CNT;
        st->vc31a_ctrl &= ~VC31A_CTRL_ENABLE_PPG;
        prv_write_register(dev, VC31A_REG_CTRL, st->vc31a_ctrl);
      }
    } else {
      st->un_wear_cnt = VC31A_UNWEAR_CNT;
    }
  } else {
    if (st->vc31a_ps >= st->vc31a_env + VC31A_PS_LIMIT) {
      if (--st->is_wear_cnt <= 0) {
        st->is_wearing = true;
        st->un_wear_cnt = VC31A_UNWEAR_CNT;
        st->is_wear_cnt = VC31A_ISWEAR_CNT;
        st->vc31a_ctrl |= VC31A_CTRL_ENABLE_PPG;
        prv_write_register(dev, VC31A_REG_CTRL, st->vc31a_ctrl);
      }
    } else {
      st->is_wear_cnt = VC31A_ISWEAR_CNT;
    }
  }
}

//------------------------------------------------------------------------------
// VC31B helpers
//------------------------------------------------------------------------------

static void prv_vc31b_wearstatus(HRMDevice *dev) {
  HRMDeviceState *st = dev->state;
  uint8_t env2 = st->raw[5] >> 4;
  uint8_t ps = st->raw[5] & 0x0F;
  (void)ps;

  if (st->is_wearing) {
    if ((env2 > VC31B_PS_TH) || ((st->raw[5] & 0x0F) < VC31B_PS_TH) ||
        st->vcb_slot0_env_exceed || st->vcb_slot1_env_exceed) {
      if (--st->un_wear_cnt <= 0) {
        st->is_wearing = false;
        st->un_wear_cnt = VC31A_UNWEAR_CNT;
        st->is_wear_cnt = VC31A_ISWEAR_CNT;
        // Disable SLOT0+1, enable SLOT2
        st->vcb_reg_config[0] = (st->vcb_reg_config[0] & 0xF8) | 0x04;
        prv_write_register(dev, VC31B_REG_CTRL, st->vcb_reg_config[0]);
      }
    } else {
      st->un_wear_cnt = VC31A_UNWEAR_CNT;
    }
  } else {
    if ((ps >= VC31B_PS_TH) && (env2 < 3)) {
      if (--st->is_wear_cnt <= 0) {
        st->is_wearing = true;
        st->un_wear_cnt = VC31A_UNWEAR_CNT;
        st->is_wear_cnt = VC31A_ISWEAR_CNT;
        // Re-enable SLOT2 and SLOT0
        st->vcb_reg_config[0] = (st->vcb_reg_config[0] & 0xF8) | 0x05;
        prv_write_register(dev, VC31B_REG_CTRL, st->vcb_reg_config[0]);
      }
    } else {
      st->is_wear_cnt = VC31A_ISWEAR_CNT;
    }
  }
}

static void prv_vc31b_adjust(HRMDevice *dev, int slot_num,
                            uint8_t led_max, uint8_t pd_res_max) {
  HRMDeviceState *st = dev->state;
  VC31AdjustInfo *ai = &st->vcb_adjust_info[slot_num];
  uint8_t old_led = st->vcb_led_current[slot_num];
  uint8_t old_pd = st->vcb_pd_res[slot_num];
  uint8_t new_led = old_led;
  uint8_t new_pd = old_pd;

  st->vcb_slot0_env_exceed = (slot_num == 0) ? false : st->vcb_slot0_env_exceed;
  st->vcb_slot1_env_exceed = (slot_num == 1) ? false : st->vcb_slot1_env_exceed;

  if (old_led == led_max) {
    if (old_pd == pd_res_max) {
      if (slot_num == 0) st->vcb_slot0_env_exceed = true;
      if (slot_num == 1) st->vcb_slot1_env_exceed = true;
      return;
    }
    if (ai->direction == AdjustDirection_Up) {
      new_pd = (old_pd >= 7) ? 7 : (old_pd + 1);
    } else {
      new_pd = (old_pd < 1) ? 0 : (old_pd - 1);
    }
    st->was_adjusted = 2;
  } else {
    if (ai->direction_last == AdjustDirection_Null) {
      ai->step *= 16;
    } else if (ai->direction == ai->direction_last) {
      if (ai->step == 1 || ai->step == 2) {
        ai->step = (ai->step + 1) * 16;
      } else {
        ai->step *= VC31B_ADJUST_INCREASE;
      }
    } else {
      ai->step *= VC31B_ADJUST_DECREASE;
    }
    ai->step >>= 4;
    if (ai->step <= VC31B_ADJUST_STEP_MIN) ai->step = VC31B_ADJUST_STEP_MIN;
    if (ai->step >= VC31B_ADJUST_STEP_MAX) ai->step = VC31B_ADJUST_STEP_MAX;

    if (ai->direction == AdjustDirection_Up) {
      new_led = ((old_led + ai->step) > led_max) ? led_max : old_led + ai->step;
    } else {
      new_led = (old_led < ai->step) ? led_max : old_led - ai->step;
    }
    st->was_adjusted = 2;
    new_pd = old_pd;
    ai->direction_last = ai->direction;
  }

  st->vcb_led_current[slot_num] = new_led;
  st->vcb_pd_res[slot_num] = new_pd;

  st->vcb_reg_config[slot_num + 7] = new_led | st->vcb_ppg_gain[slot_num];
  prv_write_register(dev, VC31B_REG_SLOT0_LED + slot_num,
                     st->vcb_reg_config[slot_num + 7]);

  if (new_pd != old_pd) {
    PBL_LOG_INFO("VC31B S%d PD_RES: %u->%u", slot_num, old_pd, new_pd);
    st->vcb_reg_config[slot_num + 10] = (new_pd << 4) | st->vcb_pd_res_set[slot_num];
    prv_write_register(dev, VC31B_REG_SLOT0_ENV + slot_num,
                       st->vcb_reg_config[slot_num + 10]);
  }
}

static void prv_vc31b_slot_adjust(HRMDevice *dev, int slot_num) {
  HRMDeviceState *st = dev->state;
  int slot_mask = 1 << slot_num;
  if (!(st->vcb_reg_config[0] & slot_mask)) return;

  st->vcb_led_current[slot_num] = st->vcb_reg_config[7 + slot_num] & 0x7F;
  st->vcb_ppg_gain[slot_num] = st->vcb_reg_config[7 + slot_num] & 0x80;
  st->vcb_pd_res[slot_num] = (st->vcb_reg_config[10 + slot_num] & 0x70) >> 4;
  st->vcb_pd_res_set[slot_num] = st->vcb_reg_config[10 + slot_num] & 0x8F;

  if (st->irq_status & VC31B_INT_OV) {
    uint8_t overload = st->raw[0] & VC31B_STATUS_OVERLOAD_MASK;
    if ((overload & slot_mask) && st->vcb_led_current[slot_num] != 0) {
      st->vcb_led_current[slot_num]--;
      st->vcb_led_max_current[slot_num] = st->vcb_led_current[slot_num];
      st->was_adjusted = 1;
      st->vcb_reg_config[slot_num + 7] = st->vcb_led_current[slot_num] | st->vcb_ppg_gain[slot_num];
      prv_write_register(dev, VC31B_REG_SLOT0_LED + slot_num, st->vcb_reg_config[slot_num + 7]);
    }
  }

  if (slot_num > 1) return;

  VC31AdjustInfo *ai = &st->vcb_adjust_info[slot_num];

  if (st->ppg_value > (4095 - VC31B_PPG_TH * 32)) {
    ai->direction = AdjustDirection_Up;
    prv_vc31b_adjust(dev, slot_num,
                     st->vcb_led_max_current[slot_num],
                     st->vcb_pd_res_max[slot_num]);
  } else if (st->ppg_value < (VC31B_PPG_TH * 32)) {
    ai->direction = AdjustDirection_Down;
    prv_vc31b_adjust(dev, slot_num, 0, 0);
  } else {
    ai->direction = ai->direction_last;
    ai->direction_last = AdjustDirection_Null;
  }
}

static int prv_vc31b_readfifo(HRMDevice *dev, uint16_t start_addr, uint16_t end_addr) {
  HRMDeviceState *st = dev->state;
  int data_length = end_addr - start_addr;
  if (data_length <= 0 || data_length > 256) return 0;

  uint8_t sample_data[128];
  if (!prv_read_register_block(dev, start_addr, sample_data, data_length)) {
    return 0;
  }

  int samples = 0;
  for (int i = 0; i < data_length; i += 2) {
    uint16_t ppg = ((sample_data[i] << 8) | sample_data[i + 1]);
    st->ppg_last_value = st->ppg_value;
    st->ppg_value = ppg;
    st->sample_count++;
    samples++;
  }
  return samples;
}

//------------------------------------------------------------------------------
// Interrupt handling
//------------------------------------------------------------------------------

static void prv_handle_vc31a_irq(HRMDevice *dev) {
  HRMDeviceState *st = dev->state;
  uint8_t buf[11];
  if (!prv_read_register_block(dev, VC31A_REG_STATUS, buf, 11)) {
    PBL_LOG_ERR("VC31A: failed to read status block");
    return;
  }

  st->irq_status = buf[0];
  st->ppg_last_value = st->ppg_value;
  st->ppg_value = (buf[2] << 8) | buf[1];
  st->vc31a_current = ((buf[4] << 8) | buf[3]) + 10;
  st->vc31a_pre = (buf[6] << 8) | buf[5];
  st->vc31a_ps = (buf[8] << 8) | buf[7];
  st->vc31a_env = (buf[10] << 8) | buf[9];
  st->env_value = st->vc31a_env;
  memcpy(st->raw, buf, sizeof(buf));

  PPG_DBG("VC31A IRQ stat=0x%02x PPG=%u Env=%u PS=%u Cur=%u Wear=%s",
          st->irq_status, st->ppg_value, st->vc31a_env, st->vc31a_ps, st->vc31a_current,
          st->is_wearing ? "Y" : "N");

  st->fifo_write_index = 0;
  if (prv_read_register_block(dev, VC31A_REG_PPG, st->fifo_window, 2)) {
    st->fifo_write_index = st->fifo_window[1];
  }

  if (st->irq_status & VC31A_STATUS_D_PPG_OK) {
    st->sample_count++;
    if (st->was_adjusted > 0) st->was_adjusted--;
    prv_vc31a_adjust(dev);
  }
  if (st->irq_status & VC31A_STATUS_D_PS_OK) {
    prv_vc31a_wearstatus(dev);
  }
}

static void prv_handle_vc31b_irq(HRMDevice *dev) {
  HRMDeviceState *st = dev->state;
  uint8_t buf[6];
  if (!prv_read_register_block(dev, VC31B_REG_STATUS, buf, 6)) {
    PBL_LOG_ERR("VC31B: failed to read status block");
    return;
  }

  st->raw[0] = buf[0]; // status
  st->irq_status = buf[1];
  st->raw[1] = buf[1];
  st->raw[2] = buf[2];
  st->raw[3] = buf[3];
  st->raw[4] = buf[4];
  st->raw[5] = buf[5];

  // env/pre/ps from status regs
  uint8_t env0 = buf[3] >> 4;
  uint8_t pre0 = buf[3] & 0x0F;
  uint8_t env1 = buf[4] >> 4;
  uint8_t pre1 = buf[4] & 0x0F;
  uint8_t env2 = buf[5] >> 4;
  uint8_t ps = buf[5] & 0x0F;
  (void)env0; (void)pre0; (void)env1; (void)pre1;
  st->env_value = env2;

  // Read current config state
  uint8_t cfg_buf[6];
  if (prv_read_register_block(dev, VC31B_REG_SLOT0_LED, cfg_buf, 6)) {
    st->raw[6] = cfg_buf[0];
    st->raw[7] = cfg_buf[1];
    st->raw[8] = cfg_buf[2];
    st->raw[9] = cfg_buf[3];
    st->raw[10] = cfg_buf[4];
    st->raw[11] = cfg_buf[5];
  }

  // Wear detection from PS/ENV
  if (st->irq_status & VC31B_INT_PS) {
    prv_vc31b_wearstatus(dev);
  }

  // Cache FIFO write index and window for app
  st->fifo_write_index = 0;
  prv_read_register(dev, VC31B_REG_FIFO_WR, &st->fifo_write_index);
  prv_read_register_block(dev, VC31B_FIFO_BASE, st->fifo_window, 8);

  // FIFO read
  if (st->irq_status & VC31B_INT_FIFO) {
    uint8_t fifo_wr = st->fifo_write_index;
    int samples = 0;
    if (st->vcb_en_fifo) {
      if (fifo_wr != st->vcb_fifo_read_index) {
        if (fifo_wr > st->vcb_fifo_read_index) {
          samples += prv_vc31b_readfifo(dev, st->vcb_fifo_read_index, fifo_wr);
        } else {
          samples += prv_vc31b_readfifo(dev, st->vcb_fifo_read_index, 256);
          if (fifo_wr != VC31B_FIFO_BASE) {
            samples += prv_vc31b_readfifo(dev, VC31B_FIFO_BASE, fifo_wr);
          }
        }
        st->vcb_fifo_read_index = fifo_wr;
      } else {
        // FIFO interrupt fired but indices match — read current slot data anyway
        samples += prv_vc31b_readfifo(dev, VC31B_FIFO_BASE,
                                      VC31B_FIFO_BASE + st->vcb_total_slots * 2);
      }
    } else {
      // FIFO disabled - read direct from 0x80
      st->vcb_fifo_read_index = VC31B_FIFO_BASE;
      samples += prv_vc31b_readfifo(dev, VC31B_FIFO_BASE,
                                    VC31B_FIFO_BASE + st->vcb_total_slots * 2);
    }
    st->fifo_count++;

    if (st->was_adjusted > 0) st->was_adjusted--;
    for (int slot = 0; slot < 3; slot++) {
      prv_vc31b_slot_adjust(dev, slot);
    }
  }

  PPG_DBG("VC31B IRQ stat=0x%02x PPG=%u Env=%u PS=%d Wear=%s",
          st->irq_status, st->ppg_value, st->env_value, st->raw[5] & 0x0F,
          st->is_wearing ? "Y" : "N");
}

static HRMQuality prv_map_reliability_to_quality(uint8_t reliability) {
  if (reliability == 0) return HRMQuality_NoSignal;
  if (reliability <= 24) return HRMQuality_Worst;
  if (reliability <= 49) return HRMQuality_Poor;
  if (reliability <= 69) return HRMQuality_Acceptable;
  if (reliability <= 89) return HRMQuality_Good;
  return HRMQuality_Excellent;
}

static void prv_irq_system_cb(void *context) {
  HRMDevice *dev = (HRMDevice *)context;
  mutex_lock(dev->state->lock);
  if (!hrm_is_enabled(dev)) {
    mutex_unlock(dev->state->lock);
    return;
  }

  dev->state->irq_count++;

  if (dev->state->variant == VC31Type_VC31A) {
    prv_handle_vc31a_irq(dev);
  } else if (dev->state->variant == VC31Type_VC31B) {
    prv_handle_vc31b_irq(dev);
  }

  dev->state->data_changed = true;

  // Detect off-wrist -> on-wrist transition and reset algorithm
  if (dev->state->is_wearing && !dev->state->prev_is_wearing) {
    vc31_algo_reset();
  }
  dev->state->prev_is_wearing = dev->state->is_wearing;

  HRMData hrm_data = (HRMData) {};
  hrm_data.features = HRMFeature_BPM;

  if (!dev->state->is_wearing) {
    vc31_algo_reset();
    hrm_data.hrm_bpm = 0;
    hrm_data.hrm_quality = HRMQuality_OffWrist;
    mutex_unlock(dev->state->lock);
    PPG_DBG("VC31 data: off-wrist");
    hrm_manager_new_data_cb(&hrm_data);
    return;
  }

  // Compute sample gap in milliseconds
  RtcTicks now = rtc_get_ticks();
  uint32_t gap_ms = 40; // default for VC31B ~25Hz
  if (dev->state->last_sample_ticks != 0) {
    gap_ms = (uint32_t)((now - dev->state->last_sample_ticks) * 1000 / RTC_TICKS_HZ);
    if (gap_ms == 0) gap_ms = 1;
    if (gap_ms > 500) gap_ms = 40; // clamp after long gaps
  }
  dev->state->last_sample_ticks = now;

  uint8_t bpm = 0;
  uint8_t reliability = 0;
  bool had_update = vc31_algo_feed_sample(
      (int32_t)dev->state->ppg_value,
      (int32_t)dev->state->env_value,
      gap_ms,
      dev->state->was_adjusted > 0,
      &bpm, &reliability);

  mutex_unlock(dev->state->lock);

  if (had_update) {
    hrm_data.hrm_bpm = bpm;
    hrm_data.hrm_quality = prv_map_reliability_to_quality(reliability);
    if (hrm_data.hrm_quality >= HRMQuality_Acceptable) {
      PPG_DBG("VC31 data: bpm=%u rel=%u qual=%d",
              bpm, reliability, hrm_data.hrm_quality);
      hrm_manager_new_data_cb(&hrm_data);
    } else {
      PPG_DBG("VC31 data: suppressed bpm=%u rel=%u qual=%d (below threshold)",
              bpm, reliability, hrm_data.hrm_quality);
    }
  }
}

static void prv_vc31_interrupt_handler(bool *should_context_switch) {
  PPG_DBG("VC31 IRQ");
  *should_context_switch = new_timer_add_work_callback_from_isr(prv_irq_system_cb, (void *)HRM);
}

static void prv_interrupts_enable(HRMDevice *dev, bool enable) {
  mutex_assert_held_by_curr_task(dev->state->lock, true);
  if (enable) {
    nrf_gpio_cfg_input(dev->int_input.gpio_pin, NRF_GPIO_PIN_PULLUP);
    exti_configure_pin(dev->int_exti, ExtiTrigger_Falling, prv_vc31_interrupt_handler);
    exti_enable(dev->int_exti);
  } else {
    exti_disable(dev->int_exti);
    nrf_gpio_cfg_default(dev->int_input.gpio_pin);
  }
}

//------------------------------------------------------------------------------
// Enable / Disable
//------------------------------------------------------------------------------

static void prv_disable(HRMDevice *dev) {
  mutex_assert_held_by_curr_task(dev->state->lock, true);

  prv_interrupts_enable(dev, false);

  if (dev->state->enabled_state == HRMEnabledState_PoweringOn) {
    new_timer_stop(dev->state->timer);
  }

  if (dev->state->enabled_state == HRMEnabledState_Enabled ||
      dev->state->enabled_state == HRMEnabledState_PoweringOn) {
    // Power off
    gpio_output_set(&dev->en_gpio, false);
    // Release I2C pins to avoid parasitic power
    nrf_gpio_cfg_input(dev->sda_pin, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(dev->scl_pin, NRF_GPIO_PIN_NOPULL);
    dev->state->enabled_state = HRMEnabledState_Disabled;
    vc31_algo_reset();
  }
}

static void prv_enable(HRMDevice *dev) {
  mutex_assert_held_by_curr_task(dev->state->lock, true);
  if (dev->state->enabled_state == HRMEnabledState_Uninitialized) {
    PBL_LOG_ERR("Trying to enable HRM before initialization.");
    return;
  }
  if (dev->state->enabled_state != HRMEnabledState_Disabled) {
    return;
  }

  dev->state->handshake_count = 0;
  dev->state->irq_count = 0;
  dev->state->fifo_count = 0;
  dev->state->sample_count = 0;
  dev->state->is_wearing = true;
  dev->state->prev_is_wearing = false;
  dev->state->is_wear_cnt = VC31A_ISWEAR_CNT;
  dev->state->un_wear_cnt = VC31A_UNWEAR_CNT;
  dev->state->ppg_offset = 0;
  dev->state->was_adjusted = 0;
  dev->state->last_sample_ticks = 0;
  dev->state->vcb_adjust_step_display = 0;
  vc31_algo_init();

  // Power on
  gpio_output_set(&dev->en_gpio, true);
  dev->state->enabled_state = HRMEnabledState_PoweringOn;

  new_timer_start(dev->state->timer, NORMAL_BOOT_DELAY_MS,
                  prv_enable_timer_cb, (void *)dev, 0);

  PBL_LOG_DBG("Enabling VC31...");
}

static void prv_enable_system_task_cb(void *context) {
  HRMDevice *dev = (HRMDevice *)context;
  mutex_lock(dev->state->lock);
  if (dev->state->enabled_state == HRMEnabledState_Disabled) {
    goto done;
  }
  if (dev->state->enabled_state != HRMEnabledState_PoweringOn) {
    PBL_LOG_ERR("Enable callback fired in unexpected state: %u",
                (unsigned int)dev->state->enabled_state);
    goto done;
  }

  // Verify we can talk to the sensor
  uint8_t id = 0;
  if (!prv_read_register(dev, VC31A_REG_DEV_ID, &id)) {
    PBL_LOG_ERR("VC31: failed to read device ID");
    goto failed;
  }

  if (id == 17) {
    dev->state->variant = VC31Type_VC31A;
    PBL_LOG_INFO("VC31 PROBE: VC31A detected (ID=%d)", id);
  } else if (id == 33) {
    dev->state->variant = VC31Type_VC31B;
    PBL_LOG_INFO("VC31 PROBE: VC31B detected (ID=%d)", id);
  } else {
    PBL_LOG_ERR("VC31: unexpected device ID %d", id);
    goto failed;
  }

  // Configure sensor based on variant
  if (dev->state->variant == VC31Type_VC31A) {
    // Timing registers
    prv_write_register(dev, VC31A_REG_GREEN_WAIT, 0xB4);
    prv_write_register(dev, VC31A_REG_TIA_WAIT, 0x54);
    prv_write_register(dev, VC31A_REG_PS_DIV, 0x09);
    prv_write_register(dev, VC31A_REG_IR_WAIT, 0x5F);
    prv_write_register(dev, VC31A_REG_GREEN_IR_GAP, 0x20);
    prv_write_register(dev, VC31A_REG_AMP_WAIT, 0x14);

    // PPG divisor for ~50Hz (VC31A needs 100Hz divider to get 50Hz actual)
    dev->state->vc31a_divisor = VC31A_PPG_DIV_100_HZ;
    prv_write_register(dev, VC31A_REG_PPG_DIV, dev->state->vc31a_divisor & 0xFF);
    prv_write_register(dev, VC31A_REG_PPG_DIV + 1, (dev->state->vc31a_divisor >> 8) & 0xFF);

    // CTRL: work mode, enable PPG + PRE, rising edge int
    dev->state->vc31a_ctrl = VC31A_CTRL_OPA_GAIN_25 |
                              VC31A_CTRL_ENABLE_PPG |
                              VC31A_CTRL_ENABLE_PRE |
                              VC31A_CTRL_WORK_MODE |
                              VC31A_CTRL_INT_DIR_RISING;
    prv_write_register(dev, VC31A_REG_CTRL, dev->state->vc31a_ctrl);
  } else if (dev->state->variant == VC31Type_VC31B) {
    // Soft reset
    prv_write_register(dev, VC31B_REG_SOFT_RESET, 0x5A);
    psleep(5);

    dev->state->vcb_sample_rate = 25; // Hz (BangleJS2 default)
    uint8_t reg_config[17] = {
      0x45,       // VC31B_REG_CTRL: SLOT2(env) + SLOT0(hr) + bit6
      VC31B_INT_OV | VC31B_INT_FIFO | VC31B_INT_ENV | VC31B_INT_PS, // INT
      0x8A,       // REG13
      0x41,       // FIFO cfg: enable + int_div=1
      0x03, 0x1F, // time cal (default)
      0x00,       // env rate
      0x00,       // slot0 led
      0x80,       // slot1 led
      0xE0,       // slot2 led = 80mA
      0x57, 0x37, // slot0/1 env: PD_res=5/3
      0x67, 0x16, // REG22=0x67, REG23=0x16
      0x56, 0x16, 0x00
    };
    memcpy(dev->state->vcb_reg_config, reg_config, sizeof(reg_config));

    // Set sample rate divisor: 20 * interval_ms
    dev->state->vcb_divisor = 20 * 40; // 40ms = 25Hz
    dev->state->vcb_reg_config[4] = (dev->state->vcb_divisor >> 8) & 0xFF;
    dev->state->vcb_reg_config[5] = dev->state->vcb_divisor & 0xFF;
    dev->state->vcb_reg_config[6] = dev->state->vcb_sample_rate - 6;

    // Write config block
    prv_write_register_block(dev, VC31B_REG_CTRL, dev->state->vcb_reg_config, 17);

    // Enable
    dev->state->vcb_reg_config[0] |= 0x80;
    prv_write_register(dev, VC31B_REG_CTRL, dev->state->vcb_reg_config[0]);

    // Verify key registers
    uint8_t verify[5];
    if (prv_read_register_block(dev, VC31B_REG_CTRL, verify, 5)) {
      PBL_LOG_INFO("VC31B CFG: CTRL=0x%02x INT=0x%02x FIFO=0x%02x TIME=0x%02x%02x",
                   verify[0], verify[1], verify[2], verify[3], verify[4]);
    }
    uint8_t verify2[4];
    if (prv_read_register_block(dev, VC31B_REG_SLOT0_LED, verify2, 4)) {
      PBL_LOG_INFO("VC31B LED: S0=0x%02x S1=0x%02x S2=0x%02x REG22=0x%02x",
                   verify2[0], verify2[1], verify2[2], verify2[3]);
    }

    // Init state
    dev->state->vcb_fifo_read_index = VC31B_FIFO_BASE;
    for (int slot = 0; slot < 3; slot++) {
      dev->state->vcb_led_max_current[slot] = 0x6F;
      dev->state->vcb_pd_res_max[slot] = 7;
      dev->state->vcb_led_current[slot] = 0;
      dev->state->vcb_pd_res[slot] = 0;
      dev->state->vcb_pd_res_set[slot] = 0;
      dev->state->vcb_ppg_gain[slot] = 0;
    }
    for (int slot = 0; slot < 2; slot++) {
      dev->state->vcb_adjust_info[slot].direction = AdjustDirection_Null;
      dev->state->vcb_adjust_info[slot].direction_last = AdjustDirection_Null;
      dev->state->vcb_adjust_info[slot].step = VC31B_ADJUST_STEP_MIN;
    }
    dev->state->vcb_total_slots =
        ((dev->state->vcb_reg_config[0] & 0x02) ? 1 : 0) +
        ((dev->state->vcb_reg_config[0] & 0x01) ? 1 : 0);
    dev->state->vcb_fifo_int_div = dev->state->vcb_reg_config[3] & 0x3F;
    dev->state->vcb_en_fifo = dev->state->vcb_fifo_int_div != 0;
    PBL_LOG_INFO("VC31B INIT: slots=%d fifo=%d div=%u",
                 dev->state->vcb_total_slots,
                 dev->state->vcb_en_fifo,
                 dev->state->vcb_divisor);
  }

  // Configure INT pin and enable interrupts
  prv_interrupts_enable(dev, true);
  dev->state->enabled_state = HRMEnabledState_Enabled;
  PBL_LOG_DBG("VC31 enabled");
  goto done;

failed:
  prv_disable(dev);
done:
  mutex_unlock(dev->state->lock);
}

static void prv_enable_timer_cb(void *context) {
  system_task_add_callback(prv_enable_system_task_cb, context);
}

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

void hrm_init(HRMDevice *dev) {
  PBL_ASSERTN(dev->state->enabled_state == HRMEnabledState_Uninitialized);

  dev->state->lock = mutex_create();
  dev->state->timer = new_timer_create();
  dev->state->enabled_state = HRMEnabledState_Disabled;
  dev->state->variant = VC31Type_Unknown;

  gpio_output_init(&dev->en_gpio, GPIO_OType_PP, GPIO_Speed_2MHz);
  gpio_output_set(&dev->en_gpio, false);

  // Ensure I2C pins are in input/no-pull when idle
  nrf_gpio_cfg_input(dev->sda_pin, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(dev->scl_pin, NRF_GPIO_PIN_NOPULL);

  PBL_LOG_DBG("VC31 driver initialized");
}

bool hrm_enable(HRMDevice *dev) {
  if (!dev->state->lock) {
    PBL_LOG_DBG("Not an HRM device.");
    return false;
  }
  mutex_lock(dev->state->lock);
  prv_enable(dev);
  mutex_unlock(dev->state->lock);
  return true;
}

void hrm_disable(HRMDevice *dev) {
  if (!dev->state->lock) {
    PBL_LOG_DBG("Not an HRM device.");
    return;
  }
  mutex_lock(dev->state->lock);
  prv_disable(dev);
  mutex_unlock(dev->state->lock);
}

bool hrm_is_enabled(HRMDevice *dev) {
  return (dev->state->enabled_state == HRMEnabledState_Enabled ||
          dev->state->enabled_state == HRMEnabledState_PoweringOn);
}

//------------------------------------------------------------------------------
// Debug getters
//------------------------------------------------------------------------------

VC31Type vc31_get_variant(HRMDevice *dev) {
  return dev->state->variant;
}

bool vc31_is_wearing(HRMDevice *dev) {
  return dev->state->is_wearing;
}

void vc31_get_raw_metrics(HRMDevice *dev, uint16_t *ppg, uint16_t *env,
                           uint16_t *ps, uint16_t *current) {
  mutex_lock(dev->state->lock);
  *ppg = dev->state->ppg_value;
  *env = dev->state->env_value;
  if (dev->state->variant == VC31Type_VC31A) {
    *ps = dev->state->vc31a_ps;
    *current = dev->state->vc31a_current;
  } else {
    *ps = dev->state->raw[5] & 0x0F;
    *current = dev->state->vcb_led_current[0];
  }
  mutex_unlock(dev->state->lock);
}

void vc31_get_slot_config(HRMDevice *dev, uint8_t *led_current,
                          uint8_t *pd_res, uint8_t *ppg_gain) {
  mutex_lock(dev->state->lock);
  if (dev->state->variant == VC31Type_VC31B) {
    *led_current = dev->state->vcb_led_current[0];
    *pd_res = dev->state->vcb_pd_res[0];
    *ppg_gain = dev->state->vcb_ppg_gain[0];
  } else {
    *led_current = 0;
    *pd_res = 0;
    *ppg_gain = 0;
  }
  mutex_unlock(dev->state->lock);
}

uint8_t vc31_get_saturation_streak(HRMDevice *dev) {
  mutex_lock(dev->state->lock);
  uint8_t step = (uint8_t)dev->state->vcb_adjust_info[0].step;
  mutex_unlock(dev->state->lock);
  return step;
}

void vc31_get_counters(HRMDevice *dev, uint32_t *irq_count,
                        uint32_t *fifo_count, uint32_t *sample_count) {
  mutex_lock(dev->state->lock);
  *irq_count = dev->state->irq_count;
  *fifo_count = dev->state->fifo_count;
  *sample_count = dev->state->sample_count;
  mutex_unlock(dev->state->lock);
}

void vc31_get_status_block(HRMDevice *dev, uint8_t *buf, size_t len) {
  mutex_lock(dev->state->lock);
  size_t copy = len < sizeof(dev->state->raw) ? len : sizeof(dev->state->raw);
  memcpy(buf, dev->state->raw, copy);
  mutex_unlock(dev->state->lock);
}

uint8_t vc31_get_fifo_write_index(HRMDevice *dev) {
  mutex_lock(dev->state->lock);
  uint8_t idx = dev->state->fifo_write_index;
  mutex_unlock(dev->state->lock);
  return idx;
}

void vc31_get_fifo_window(HRMDevice *dev, uint8_t *buf, size_t len) {
  mutex_lock(dev->state->lock);
  size_t copy = len < sizeof(dev->state->fifo_window) ? len : sizeof(dev->state->fifo_window);
  memcpy(buf, dev->state->fifo_window, copy);
  mutex_unlock(dev->state->lock);
}

bool vc31_data_changed(HRMDevice *dev) {
  mutex_lock(dev->state->lock);
  bool changed = dev->state->data_changed;
  dev->state->data_changed = false;
  mutex_unlock(dev->state->lock);
  return changed;
}

//------------------------------------------------------------------------------
// Prompt Commands
//------------------------------------------------------------------------------

#include "console/prompt.h"

void command_hrm_vc31_probe(void) {
  HRMDevice *dev = HRM;
  if (!dev->state->lock) {
    prompt_send_response("No HRM device");
    return;
  }

  mutex_lock(dev->state->lock);
  gpio_output_set(&dev->en_gpio, true);
  psleep(NORMAL_BOOT_DELAY_MS);

  uint8_t id = 0;
  bool ok = prv_read_register(dev, VC31A_REG_DEV_ID, &id);
  gpio_output_set(&dev->en_gpio, false);
  nrf_gpio_cfg_input(dev->sda_pin, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(dev->scl_pin, NRF_GPIO_PIN_NOPULL);
  mutex_unlock(dev->state->lock);

  if (!ok) {
    prompt_send_response("Probe failed (no ACK)");
    return;
  }

  char buf[64];
  if (id == 17) {
    prompt_send_response_fmt(buf, sizeof(buf), "VC31A detected (ID=%d)", id);
  } else if (id == 33) {
    prompt_send_response_fmt(buf, sizeof(buf), "VC31B detected (ID=%d)", id);
  } else {
    prompt_send_response_fmt(buf, sizeof(buf), "Unknown device (ID=%d)", id);
  }
}

void command_hrm_vc31_enable(void) {
  hrm_enable(HRM);
  prompt_send_response("HRM enabled");
}

void command_hrm_vc31_disable(void) {
  hrm_disable(HRM);
  prompt_send_response("HRM disabled");
}

void command_hrm_vc31_status(void) {
  HRMDevice *dev = HRM;
  if (!dev->state->lock) {
    prompt_send_response("No HRM device");
    return;
  }

  mutex_lock(dev->state->lock);
  char buf[128];
  const char *variant_str = "Unknown";
  if (dev->state->variant == VC31Type_VC31A) variant_str = "VC31A";
  if (dev->state->variant == VC31Type_VC31B) variant_str = "VC31B";

  prompt_send_response_fmt(buf, sizeof(buf),
    "Variant: %s | Wear: %s | IRQ: %lu | FIFO: %lu | Samples: %lu | PPG: %u | Env: %u",
    variant_str,
    dev->state->is_wearing ? "Yes" : "No",
    (unsigned long)dev->state->irq_count,
    (unsigned long)dev->state->fifo_count,
    (unsigned long)dev->state->sample_count,
    dev->state->ppg_value,
    dev->state->env_value);
  mutex_unlock(dev->state->lock);
}
