/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <string.h>

#include "applib/app.h"
#include "applib/app_message/app_message.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "apps/system_app_ids.h"
#if PLATFORM_SILK
#include "drivers/hrm/as7000/as7000.h"
#endif
#if PLATFORM_BANGLEJS2
#include "drivers/hrm/vc31/vc31.h"
#include "kernel/util/sleep.h"
#endif
#include "kernel/pbl_malloc.h"
#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"
#include "process_state/app_state/app_state.h"
#include "services/common/hrm/hrm_manager.h"
#include "system/passert.h"

#define BPM_STRING_LEN 10

#if PLATFORM_BANGLEJS2

typedef enum {
  Page_Decoded = 0,
  Page_Raw,
  Page_FIFO,
  Page_Count,
} DiagnosticPage;

#define NUM_TEXT_LINES 8
#define LINE_BUF_LEN 48

typedef struct {
  HRMSessionRef session;
  EventServiceInfo hrm_event_info;

  Window window;
  TextLayer title_text_layer;
  TextLayer text_layers[NUM_TEXT_LINES];
  char line_buffers[NUM_TEXT_LINES][LINE_BUF_LEN];

  DiagnosticPage page;

  // Previous values for delta tracking
  uint16_t prev_ppg;
  uint16_t prev_env;
  uint16_t prev_ps;
  uint8_t prev_raw[12];
  uint8_t prev_fifo[8];
  uint32_t prev_irq_count;
  uint32_t prev_sample_count;

  // HRM event cache
  uint8_t last_bpm;
  uint8_t last_reliability;
  HRMQuality last_quality;
  bool has_hrm_data;

  AppTimer *refresh_timer;
} AppData;

static const char *prv_variant_string(void) {
  VC31Type variant = vc31_get_variant(HRM);
  switch (variant) {
    case VC31Type_VC31A: return "VC31A";
    case VC31Type_VC31B: return "VC31B";
    default: return "Unknown";
  }
}

static char prv_delta_char(uint16_t current, uint16_t previous) {
  if (current == previous) return '=';
  if (current > previous) return '+';
  return '-';
}

static void prv_format_decoded_page(AppData *app_data) {
  uint16_t ppg = 0, env = 0, ps = 0, current = 0;
  uint32_t irq_count = 0, fifo_count = 0, sample_count = 0;
  vc31_get_raw_metrics(HRM, &ppg, &env, &ps, &current);
  vc31_get_counters(HRM, &irq_count, &fifo_count, &sample_count);
  uint8_t fifo_wr = vc31_get_fifo_write_index(HRM);

  char ppg_delta = prv_delta_char(ppg, app_data->prev_ppg);
  char env_delta = prv_delta_char(env, app_data->prev_env);
  char ps_delta = prv_delta_char(ps, app_data->prev_ps);

  snprintf(app_data->line_buffers[0], LINE_BUF_LEN,
           "%s %s %s",
           prv_variant_string(),
           hrm_is_enabled(HRM) ? "ON" : "OFF",
           vc31_is_wearing(HRM) ? "Worn" : "Off");

  snprintf(app_data->line_buffers[1], LINE_BUF_LEN,
           "PPG:%4u %c Env:%3u %c",
           ppg, ppg_delta, env, env_delta);

  snprintf(app_data->line_buffers[2], LINE_BUF_LEN,
           "PS:%2u %c Cur:%2u",
           ps, ps_delta, current);

  snprintf(app_data->line_buffers[3], LINE_BUF_LEN,
           "IRQ:%4lu FIFO:%2lu S:%4lu",
           (unsigned long)irq_count,
           (unsigned long)fifo_count,
           (unsigned long)sample_count);

  snprintf(app_data->line_buffers[4], LINE_BUF_LEN,
           "FIFO_WR:0x%02X", fifo_wr);

  uint8_t led_cur = 0, pd_res = 0, ppg_gain = 0;
  vc31_get_slot_config(HRM, &led_cur, &pd_res, &ppg_gain);
  uint8_t sat_streak = vc31_get_saturation_streak(HRM);
  snprintf(app_data->line_buffers[5], LINE_BUF_LEN,
           "Cur:%2u PD:%u G:%u Stp:%u", led_cur, pd_res, ppg_gain, sat_streak);

  if (app_data->has_hrm_data) {
    snprintf(app_data->line_buffers[6], LINE_BUF_LEN,
             "BPM:%3u", app_data->last_bpm);
  } else {
    snprintf(app_data->line_buffers[6], LINE_BUF_LEN,
             "BPM: --");
  }

  const char *quality_str = "--";
  switch (app_data->last_quality) {
    case HRMQuality_OffWrist: quality_str = "OffWrist"; break;
    case HRMQuality_NoSignal: quality_str = "NoSignal"; break;
    case HRMQuality_Worst: quality_str = "Worst"; break;
    case HRMQuality_Poor: quality_str = "Poor"; break;
    case HRMQuality_Acceptable: quality_str = "Acceptable"; break;
    case HRMQuality_Good: quality_str = "Good"; break;
    case HRMQuality_Excellent: quality_str = "Excellent"; break;
    default: break;
  }
  snprintf(app_data->line_buffers[7], LINE_BUF_LEN,
           "Qual: %s", quality_str);

  // Clear unused lines
  for (int i = 8; i < NUM_TEXT_LINES; i++) {
    app_data->line_buffers[i][0] = '\0';
  }

  app_data->prev_ppg = ppg;
  app_data->prev_env = env;
  app_data->prev_ps = ps;
  app_data->prev_irq_count = irq_count;
  app_data->prev_sample_count = sample_count;
}

static void prv_format_raw_page(AppData *app_data) {
  uint8_t raw[12];
  vc31_get_status_block(HRM, raw, sizeof(raw));

  for (int row = 0; row < 3; row++) {
    char *buf = app_data->line_buffers[row];
    int offset = row * 4;
    int len = 0;
    for (int col = 0; col < 4 && (offset + col) < 12; col++) {
      int idx = offset + col;
      bool changed = (raw[idx] != app_data->prev_raw[idx]);
      len += snprintf(buf + len, LINE_BUF_LEN - len,
                      "%s%02X:%02X ", changed ? "*" : "", idx + 1, raw[idx]);
    }
    if (len > 0 && buf[len - 1] == ' ') {
      buf[len - 1] = '\0';
    }
  }

  // Clear unused lines
  for (int i = 3; i < NUM_TEXT_LINES; i++) {
    app_data->line_buffers[i][0] = '\0';
  }

  memcpy(app_data->prev_raw, raw, sizeof(raw));
}

static void prv_format_fifo_page(AppData *app_data) {
  uint8_t fifo[8];
  vc31_get_fifo_window(HRM, fifo, sizeof(fifo));

  int len = 0;
  char *buf = app_data->line_buffers[0];
  for (int i = 0; i < 8; i++) {
    bool changed = (fifo[i] != app_data->prev_fifo[i]);
    len += snprintf(buf + len, LINE_BUF_LEN - len,
                    "%s%02X ", changed ? "*" : "", fifo[i]);
  }
  if (len > 0 && buf[len - 1] == ' ') {
    buf[len - 1] = '\0';
  }

  // Clear unused lines
  for (int i = 1; i < NUM_TEXT_LINES; i++) {
    app_data->line_buffers[i][0] = '\0';
  }

  memcpy(app_data->prev_fifo, fifo, sizeof(fifo));
}

static void prv_update_display(AppData *app_data) {
  const char *page_titles[Page_Count] = {
    [Page_Decoded] = "VC31 Decoded",
    [Page_Raw] = "VC31 Raw Regs",
    [Page_FIFO] = "VC31 FIFO",
  };
  text_layer_set_text(&app_data->title_text_layer, page_titles[app_data->page]);

  switch (app_data->page) {
    case Page_Decoded:
      prv_format_decoded_page(app_data);
      break;
    case Page_Raw:
      prv_format_raw_page(app_data);
      break;
    case Page_FIFO:
      prv_format_fifo_page(app_data);
      break;
    default:
      break;
  }

  layer_mark_dirty(&app_data->window.layer);
}

static void prv_refresh_timer_cb(void *data) {
  AppData *app_data = app_state_get_user_data();
  prv_update_display(app_data);
  // Re-schedule for next 100ms
  app_data->refresh_timer = app_timer_register(100, prv_refresh_timer_cb, NULL);
}

static void prv_toggle_hrm(void) {
  if (hrm_is_enabled(HRM)) {
    hrm_disable(HRM);
  } else {
    hrm_enable(HRM);
  }
}

static void prv_reprobe_hrm(void) {
  hrm_disable(HRM);
  psleep(100);
  hrm_enable(HRM);
}

static void prv_button_handler(ClickRecognizerRef recognizer, void *context) {
  (void)context;
  AppData *app_data = app_state_get_user_data();
  ButtonId button = click_recognizer_get_button_id(recognizer);

  if (button == BUTTON_ID_SELECT) {
    prv_toggle_hrm();
  } else if (button == BUTTON_ID_UP) {
    if (app_data->page > 0) {
      app_data->page--;
    } else {
      app_data->page = Page_Count - 1;
    }
  } else if (button == BUTTON_ID_DOWN) {
    app_data->page++;
    if (app_data->page >= Page_Count) {
      app_data->page = 0;
    }
  } else if (button == BUTTON_ID_BACK) {
    prv_reprobe_hrm();
  }

  prv_update_display(app_data);
}

static void prv_click_config_provider(void *context) {
  (void)context;
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_button_handler);
  window_single_click_subscribe(BUTTON_ID_UP, prv_button_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_button_handler);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_button_handler);
}

static void prv_handle_hrm_data(PebbleEvent *e, void *context) {
  (void)context;
  AppData *app_data = app_state_get_user_data();

  if (e->type == PEBBLE_HRM_EVENT && e->hrm.event_type == HRMEvent_BPM) {
    app_data->last_bpm = e->hrm.bpm.bpm;
    app_data->last_quality = e->hrm.bpm.quality;
    app_data->has_hrm_data = true;
  }
}

static void prv_init(void) {
  AppData *app_data = app_malloc_check(sizeof(*app_data));
  memset(app_data, 0, sizeof(*app_data));
  app_data->page = Page_Decoded;
  app_data->last_quality = HRMQuality_NoSignal;
  app_state_set_user_data(app_data);

  Window *window = &app_data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_click_config_provider);

  // Title layer
  TextLayer *title = &app_data->title_text_layer;
  text_layer_init(title, &GRect(0, 0, window->layer.bounds.size.w, 20));
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "VC31 Decoded");
  layer_add_child(&window->layer, &title->layer);

  // Content lines
  int line_height = 18;
  int start_y = 22;
  for (int i = 0; i < NUM_TEXT_LINES; i++) {
    TextLayer *tl = &app_data->text_layers[i];
    text_layer_init(tl, &GRect(2, start_y + i * line_height,
                               window->layer.bounds.size.w - 4,
                               line_height));
    text_layer_set_font(tl, fonts_get_system_font(FONT_KEY_GOTHIC_14));
    text_layer_set_text_alignment(tl, GTextAlignmentLeft);
    text_layer_set_text(tl, app_data->line_buffers[i]);
    layer_add_child(&window->layer, &tl->layer);
  }

  // Subscribe to HRM events (kept for compatibility)
  app_data->hrm_event_info = (EventServiceInfo) {
    .type = PEBBLE_HRM_EVENT,
    .handler = prv_handle_hrm_data,
  };
  event_service_client_subscribe(&app_data->hrm_event_info);

  // Start repeating refresh timer
  app_data->refresh_timer = app_timer_register(100, prv_refresh_timer_cb, NULL);

  if (sys_hrm_manager_is_hrm_present()) {
    // Auto-enable HRM on launch
    if (!hrm_is_enabled(HRM)) {
      hrm_enable(HRM);
    }

    // Subscribe through manager so it keeps the sensor powered
    app_data->session = sys_hrm_manager_app_subscribe(
        APP_ID_HRM_DEMO, 1, SECONDS_PER_HOUR, HRMFeature_BPM);
  }

  app_window_stack_push(window, true);
}

static void prv_deinit(void) {
  AppData *app_data = app_state_get_user_data();
  if (app_data->refresh_timer) {
    app_timer_cancel(app_data->refresh_timer);
  }
  event_service_client_unsubscribe(&app_data->hrm_event_info);
  sys_hrm_manager_unsubscribe(app_data->session);
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

#else // !PLATFORM_BANGLEJS2

// ------------------------------------------------------------------------------
// Standard Pebble HRM Demo (non-BangleJS2)
// ------------------------------------------------------------------------------

typedef enum {
  AppMessageKey_Status = 1,

  AppMessageKey_HeartRate = 10,
  AppMessageKey_Confidence = 11,
  AppMessageKey_Current = 12,
  AppMessageKey_TIA = 13,
  AppMessageKey_PPG = 14,
  AppMessageKey_AccelData = 15,
  AppMessageKey_SerialNumber = 16,
  AppMessageKey_Model = 17,
  AppMessageKey_HRMProtocolVersionMajor = 18,
  AppMessageKey_HRMProtocolVersionMinor = 19,
  AppMessageKey_HRMSoftwareVersionMajor = 20,
  AppMessageKey_HRMSoftwareVersionMinor = 21,
  AppMessageKey_HRMApplicationID = 22,
  AppMessageKey_HRMHardwareRevision = 23,
} AppMessageKey;

typedef enum {
  AppStatus_Stopped = 0,
  AppStatus_Enabled_1HZ = 1,
} AppStatus;

typedef struct {
  HRMSessionRef session;
  EventServiceInfo hrm_event_info;

  Window window;
  TextLayer bpm_text_layer;
  TextLayer quality_text_layer;

  char bpm_string[BPM_STRING_LEN];

  bool ready_to_send;
  DictionaryIterator *out_iter;
} AppData;

static char *prv_get_quality_string(HRMQuality quality) {
  switch (quality) {
    case HRMQuality_NoAccel:
      return "No Accel Data";
    case HRMQuality_OffWrist:
      return "Off Wrist";
    case HRMQuality_NoSignal:
      return "No Signal";
    case HRMQuality_Worst:
      return "Worst";
    case HRMQuality_Poor:
      return "Poor";
    case HRMQuality_Acceptable:
      return "Acceptable";
    case HRMQuality_Good:
      return "Good";
    case HRMQuality_Excellent:
      return "Excellent";
  }
  WTF;
}

static char *prv_translate_error(AppMessageResult result) {
  switch (result) {
    case APP_MSG_OK: return "APP_MSG_OK";
    case APP_MSG_SEND_TIMEOUT: return "APP_MSG_SEND_TIMEOUT";
    case APP_MSG_SEND_REJECTED: return "APP_MSG_SEND_REJECTED";
    case APP_MSG_NOT_CONNECTED: return "APP_MSG_NOT_CONNECTED";
    case APP_MSG_APP_NOT_RUNNING: return "APP_MSG_APP_NOT_RUNNING";
    case APP_MSG_INVALID_ARGS: return "APP_MSG_INVALID_ARGS";
    case APP_MSG_BUSY: return "APP_MSG_BUSY";
    case APP_MSG_BUFFER_OVERFLOW: return "APP_MSG_BUFFER_OVERFLOW";
    case APP_MSG_ALREADY_RELEASED: return "APP_MSG_ALREADY_RELEASED";
    case APP_MSG_CALLBACK_ALREADY_REGISTERED: return "APP_MSG_CALLBACK_ALREADY_REGISTERED";
    case APP_MSG_CALLBACK_NOT_REGISTERED: return "APP_MSG_CALLBACK_NOT_REGISTERED";
    case APP_MSG_OUT_OF_MEMORY: return "APP_MSG_OUT_OF_MEMORY";
    case APP_MSG_CLOSED: return "APP_MSG_CLOSED";
    case APP_MSG_INTERNAL_ERROR: return "APP_MSG_INTERNAL_ERROR";
    default: return "UNKNOWN ERROR";
  }
}

static void prv_send_msg(void) {
  AppData *app_data = app_state_get_user_data();

  AppMessageResult result = app_message_outbox_send();
  if (result == APP_MSG_OK) {
    app_data->ready_to_send = false;
  } else {
    PBL_LOG_DBG("Error sending message: %s", prv_translate_error(result));
  }
}

static void prv_send_status_and_version(void) {
  AppData *app_data = app_state_get_user_data();
  PBL_LOG_DBG("Sending status and version to mobile app");

  AppMessageResult result = app_message_outbox_begin(&app_data->out_iter);
  if (result != APP_MSG_OK) {
    PBL_LOG_DBG("Failed to begin outbox - reason %i %s",
            result, prv_translate_error(result));
    return;
  }

  dict_write_uint8(app_data->out_iter, AppMessageKey_Status, AppStatus_Enabled_1HZ);

#if CAPABILITY_HAS_BUILTIN_HRM && PLATFORM_SILK
  if (mfg_info_is_hrm_present()) {
    AS7000InfoRecord hrm_info = {};
    as7000_get_version_info(HRM, &hrm_info);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMProtocolVersionMajor,
                     hrm_info.protocol_version_major);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMProtocolVersionMinor,
                     hrm_info.protocol_version_minor);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMSoftwareVersionMajor,
                     hrm_info.sw_version_major);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMSoftwareVersionMinor,
                     hrm_info.sw_version_minor);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMApplicationID,
                     hrm_info.application_id);
    dict_write_uint8(app_data->out_iter, AppMessageKey_HRMHardwareRevision,
                     hrm_info.hw_revision);
  }
#endif

  char serial_number_buffer[MFG_SERIAL_NUMBER_SIZE + 1];
  mfg_info_get_serialnumber(serial_number_buffer, sizeof(serial_number_buffer));
  dict_write_data(app_data->out_iter, AppMessageKey_SerialNumber,
                  (uint8_t*) serial_number_buffer, sizeof(serial_number_buffer));

#if IS_BIGBOARD
  WatchInfoColor watch_color = WATCH_INFO_MODEL_UNKNOWN;
#else
  WatchInfoColor watch_color = mfg_info_get_watch_color();
#endif // IS_BIGBOARD
  dict_write_uint32(app_data->out_iter, AppMessageKey_Model, watch_color);

  prv_send_msg();
}

static void prv_handle_hrm_data(PebbleEvent *e, void *context) {
  AppData *app_data = app_state_get_user_data();

  if (e->type == PEBBLE_HRM_EVENT) {
    PebbleHRMEvent *hrm = &e->hrm;

    static uint8_t bpm = 0;
    static uint8_t bpm_quality = 0;
    static uint16_t led_current = 0;

    if (hrm->event_type == HRMEvent_BPM) {
      snprintf(app_data->bpm_string, sizeof(app_data->bpm_string), "%"PRIu8" BPM", hrm->bpm.bpm);
      text_layer_set_text(&app_data->quality_text_layer, prv_get_quality_string(hrm->bpm.quality));
      layer_mark_dirty(&app_data->window.layer);

      bpm = hrm->bpm.bpm;
      bpm_quality = hrm->bpm.quality;
    } else if (hrm->event_type == HRMEvent_SubscriptionExpiring) {
      PBL_LOG_INFO("Got subscription expiring event");
      const uint32_t update_time_s = 1;
      app_data->session = sys_hrm_manager_app_subscribe(APP_ID_HRM_DEMO, update_time_s,
                                                        SECONDS_PER_HOUR, HRMFeature_BPM);
    }
  }
}

static void prv_enable_hrm(void) {
  AppData *app_data = app_state_get_user_data();

  app_data->hrm_event_info = (EventServiceInfo) {
    .type = PEBBLE_HRM_EVENT,
    .handler = prv_handle_hrm_data,
  };
  event_service_client_subscribe(&app_data->hrm_event_info);

  const uint32_t update_time_s = 1;
  app_data->session = sys_hrm_manager_app_subscribe(
      APP_ID_HRM_DEMO, update_time_s, SECONDS_PER_HOUR,
      HRMFeature_BPM);
}

static void prv_disable_hrm(void) {
  AppData *app_data = app_state_get_user_data();

  event_service_client_unsubscribe(&app_data->hrm_event_info);
  sys_hrm_manager_unsubscribe(app_data->session);
}

static void prv_handle_mobile_status_request(AppStatus status) {
  AppData *app_data = app_state_get_user_data();

  if (status == AppStatus_Stopped) {
    text_layer_set_text(&app_data->bpm_text_layer, "Paused");
    text_layer_set_text(&app_data->quality_text_layer, "Paused by mobile");
    prv_disable_hrm();
  } else {
    app_data->bpm_string[0] = '\0';
    text_layer_set_text(&app_data->bpm_text_layer, app_data->bpm_string);
    text_layer_set_text(&app_data->quality_text_layer, "Loading...");
    prv_enable_hrm();
  }
}

static void prv_message_received_cb(DictionaryIterator *iterator, void *context) {
  Tuple *status_tuple = dict_find(iterator, AppMessageKey_Status);

  if (status_tuple) {
    prv_handle_mobile_status_request(status_tuple->value->uint8);
  }
}

static void prv_message_sent_cb(DictionaryIterator *iterator, void *context) {
  AppData *app_data = app_state_get_user_data();

  app_data->ready_to_send = true;
}

static void prv_message_failed_cb(DictionaryIterator *iterator,
                               AppMessageResult reason, void *context) {
  PBL_LOG_DBG("Out message send failed - reason %i %s",
          reason, prv_translate_error(reason));
  AppData *app_data = app_state_get_user_data();
  app_data->ready_to_send = true;
}

static void prv_remote_notify_timer_cb(void *data) {
  prv_send_status_and_version();
}

static void prv_init(void) {
  AppData *app_data = app_malloc_check(sizeof(*app_data));
  *app_data = (AppData) {
    .session = (HRMSessionRef)app_data,
    .ready_to_send = false,
  };
  app_state_set_user_data(app_data);

  Window *window = &app_data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  GRect bounds = window->layer.bounds;

  bounds.origin.y += 40;
  TextLayer *bpm_tl = &app_data->bpm_text_layer;
  text_layer_init(bpm_tl, &bounds);
  text_layer_set_font(bpm_tl, fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD));
  text_layer_set_text_alignment(bpm_tl, GTextAlignmentCenter);
  text_layer_set_text(bpm_tl, app_data->bpm_string);
  layer_add_child(&window->layer, &bpm_tl->layer);

  bounds.origin.y += 35;
  TextLayer *quality_tl = &app_data->quality_text_layer;
  text_layer_init(quality_tl, &bounds);
  text_layer_set_font(quality_tl, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(quality_tl, GTextAlignmentCenter);
  text_layer_set_text(quality_tl, "Loading...");
  layer_add_child(&window->layer, &quality_tl->layer);

  const uint32_t inbox_size = 64;
  const uint32_t outbox_size = 256;
  AppMessageResult result = app_message_open(inbox_size, outbox_size);
  if (result != APP_MSG_OK) {
    PBL_LOG_ERR("Unable to open app message! %i %s",
            result, prv_translate_error(result));
  } else {
    PBL_LOG_DBG("Successfully opened app message");
  }

  if (!sys_hrm_manager_is_hrm_present()) {
    text_layer_set_text(quality_tl, "No HRM Present");
  } else {
    text_layer_set_text(quality_tl, "Loading...");
    prv_enable_hrm();
  }

  app_message_register_inbox_received(prv_message_received_cb);
  app_message_register_outbox_sent(prv_message_sent_cb);
  app_message_register_outbox_failed(prv_message_failed_cb);

  app_timer_register(1000, prv_remote_notify_timer_cb, NULL);

  app_window_stack_push(window, true);
}

static void prv_deinit(void) {
  AppData *app_data = app_state_get_user_data();
  sys_hrm_manager_unsubscribe(app_data->session);
}

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

#endif // PLATFORM_BANGLEJS2

const PebbleProcessMd* hrm_demo_get_app_info(void) {
  static const PebbleProcessMdSystem s_hrm_demo_app_info = {
    .name = "HRM Demo",
    .common.uuid = { 0xf8, 0x1b, 0x2a, 0xf8, 0x13, 0x0a, 0x11, 0xe6,
                     0x86, 0x9f, 0xa4, 0x5e, 0x60, 0xb9, 0x77, 0x3d },
    .common.main_func = &prv_main,
  };
  return (sys_hrm_manager_is_hrm_present()) ? (const PebbleProcessMd*)&s_hrm_demo_app_info : NULL;
}
