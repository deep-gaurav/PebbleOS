/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_hrm.h"

#include "applib/app.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "drivers/accel.h"
#include "drivers/hrm.h"
#if PLATFORM_BANGLEJS2
#include "drivers/hrm/vc31/vc31.h"
#endif
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "mfg/mfg_info.h"
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "process_management/process_manager.h"
#include "services/common/evented_timer.h"
#include "services/common/hrm/hrm_manager.h"
#include "util/bitset.h"
#include "util/size.h"
#include "util/trig.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#if CAPABILITY_HAS_BUILTIN_HRM

#define STATUS_STRING_LEN 48
#define BPM_STRING_LEN 48
#define SPO2_STRING_LEN 48

typedef struct {
  Window window;
  EventServiceInfo hrm_event_info;
  EventServiceInfo tick_event_info;

  TextLayer title_text_layer;
  TextLayer status_text_layer;
  TextLayer bpm_text_layer;
  TextLayer spo2_text_layer;
  char status_string[STATUS_STRING_LEN];
  char bpm_string[BPM_STRING_LEN];
  char spo2_string[SPO2_STRING_LEN];
  HRMSessionRef hrm_session;
} AppData;

static void prv_update_display(AppData *app_data) {
#if PLATFORM_BANGLEJS2
  uint16_t ppg = 0, env = 0, ps = 0, current = 0;
  uint32_t irq_count = 0, fifo_count = 0, sample_count = 0;
  vc31_get_raw_metrics(HRM, &ppg, &env, &ps, &current);
  vc31_get_counters(HRM, &irq_count, &fifo_count, &sample_count);

  const char *variant_str = "Unknown";
  VC31Type variant = vc31_get_variant(HRM);
  if (variant == VC31Type_VC31A) variant_str = "VC31A";
  if (variant == VC31Type_VC31B) variant_str = "VC31B";

  snprintf(app_data->status_string, STATUS_STRING_LEN,
           "%s %s %s",
           variant_str,
           hrm_is_enabled(HRM) ? "ON" : "OFF",
           vc31_is_wearing(HRM) ? "Worn" : "Off");
  snprintf(app_data->bpm_string, BPM_STRING_LEN,
           "PPG:%u Env:%u PS:%u", ppg, env, ps);
  snprintf(app_data->spo2_string, SPO2_STRING_LEN,
           "IRQ:%lu S:%lu C:%u", (unsigned long)irq_count, (unsigned long)sample_count, current);
#else
  (void)app_data;
#endif
}

static void prv_handle_hrm_data(PebbleEvent *e, void *context) {
  AppData *app_data = app_state_get_user_data();
  (void)context;

  if (e->type == PEBBLE_HRM_EVENT) {
    snprintf(app_data->status_string, STATUS_STRING_LEN, "Sampling...");

    if (e->hrm.event_type == HRMEvent_BPM) {
      snprintf(app_data->bpm_string, BPM_STRING_LEN,
              "HR:%d (quality:%d)", e->hrm.bpm.bpm, e->hrm.bpm.quality);
    } else if (e->hrm.event_type == HRMEvent_SpO2) {
      snprintf(app_data->spo2_string, SPO2_STRING_LEN,
              "SpO2:%d (quality:%d)", e->hrm.spo2.percent, e->hrm.spo2.quality);
    }

#if PLATFORM_BANGLEJS2
    prv_update_display(app_data);
#endif

    layer_mark_dirty(&app_data->window.layer);
  }
}

#if PLATFORM_BANGLEJS2
static void prv_handle_tick(PebbleEvent *e, void *context) {
  AppData *app_data = app_state_get_user_data();
  (void)e;
  (void)context;
  prv_update_display(app_data);
  layer_mark_dirty(&app_data->window.layer);

  uint16_t ppg = 0, env = 0, ps = 0, current = 0;
  uint32_t irq_count = 0, fifo_count = 0, sample_count = 0;
  vc31_get_raw_metrics(HRM, &ppg, &env, &ps, &current);
  vc31_get_counters(HRM, &irq_count, &fifo_count, &sample_count);
  PBL_LOG_INFO("VC31APP PPG=%u Env=%u PS=%u Cur=%u IRQ=%lu S=%lu Wear=%s",
               ppg, env, ps, current,
               (unsigned long)irq_count, (unsigned long)sample_count,
               vc31_is_wearing(HRM) ? "Y" : "N");
}

static void prv_toggle_hrm(void) {
  if (hrm_is_enabled(HRM)) {
    hrm_disable(HRM);
  } else {
    hrm_enable(HRM);
  }
}

static void prv_button_handler(ClickRecognizerRef recognizer, void *context) {
  (void)context;
  ButtonId button = click_recognizer_get_button_id(recognizer);
  if (button == BUTTON_ID_SELECT) {
    prv_toggle_hrm();
  } else if (button == BUTTON_ID_BACK) {
    // Force re-probe by disabling, waiting, then enabling
    hrm_disable(HRM);
    psleep(100);
    hrm_enable(HRM);
  }
}

static void prv_click_config_provider(void *context) {
  (void)context;
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_button_handler);
  window_single_click_subscribe(BUTTON_ID_BACK, prv_button_handler);
}
#endif

static void prv_handle_init(void) {
  const bool has_hrm = mfg_info_is_hrm_present();

  AppData *data = task_zalloc(sizeof(*data));
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

#if PLATFORM_BANGLEJS2
  window_set_click_config_provider(window, prv_click_config_provider);
#endif

  TextLayer *title = &data->title_text_layer;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
#if PLATFORM_BANGLEJS2
  text_layer_set_text(title, "VC31 Bring-Up");
#else
  text_layer_set_text(title, "HRM TEST");
#endif
  layer_add_child(&window->layer, &title->layer);

  if (has_hrm) {
    sniprintf(data->status_string, STATUS_STRING_LEN, "Starting...");
  } else {
    sniprintf(data->status_string, STATUS_STRING_LEN, "Not an HRM device");
  }

  snprintf(data->bpm_string, BPM_STRING_LEN, "HR:--");
  snprintf(data->spo2_string, SPO2_STRING_LEN, "SpO2:--");

  TextLayer *status = &data->status_text_layer;
  text_layer_init(status,
                  &GRect(5, 40, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 40));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  text_layer_set_text(status, data->status_string);
  layer_add_child(&window->layer, &status->layer);

  TextLayer *bpm = &data->bpm_text_layer;
  text_layer_init(bpm,
                  &GRect(5, 80, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 80));
  text_layer_set_font(bpm, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(bpm, GTextAlignmentCenter);
  text_layer_set_text(bpm, data->bpm_string);
  layer_add_child(&window->layer, &bpm->layer);

  TextLayer *spo2 = &data->spo2_text_layer;
  text_layer_init(spo2,
                  &GRect(5, 120, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 120));
  text_layer_set_font(spo2, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(spo2, GTextAlignmentCenter);
  text_layer_set_text(spo2, data->spo2_string);
  layer_add_child(&window->layer, &spo2->layer);

  if (has_hrm) {
    data->hrm_event_info = (EventServiceInfo){
      .type = PEBBLE_HRM_EVENT,
      .handler = prv_handle_hrm_data,
    };
    event_service_client_subscribe(&data->hrm_event_info);

#if PLATFORM_BANGLEJS2
    data->tick_event_info = (EventServiceInfo){
      .type = PEBBLE_TICK_EVENT,
      .handler = prv_handle_tick,
    };
    event_service_client_subscribe(&data->tick_event_info);

    // Auto-enable HRM on launch
    if (!hrm_is_enabled(HRM)) {
      hrm_enable(HRM);
    }
#endif

    // Use app data as session ref
    AppInstallId  app_id = 1;
#if PLATFORM_BANGLEJS2
    data->hrm_session = sys_hrm_manager_app_subscribe(app_id, 1, SECONDS_PER_HOUR,
                                                      HRMFeature_BPM);
#else
    data->hrm_session = sys_hrm_manager_app_subscribe(app_id, 1, SECONDS_PER_HOUR,
                                                      HRMFeature_BPM | HRMFeature_SpO2);
#endif
  }

  app_window_stack_push(window, true);
}

static void prv_handle_deinit(void) {
  AppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->hrm_event_info);
#if PLATFORM_BANGLEJS2
  event_service_client_unsubscribe(&data->tick_event_info);
#endif
  if (mfg_info_is_hrm_present()) {
    sys_hrm_manager_unsubscribe(data->hrm_session);
  }

  text_layer_deinit(&data->title_text_layer);
  text_layer_deinit(&data->status_text_layer);
  text_layer_deinit(&data->bpm_text_layer);
  text_layer_deinit(&data->spo2_text_layer);
  window_deinit(&data->window);
  app_free(data);
}

static void prv_main(void) {
  prv_handle_init();
  app_event_loop();
  prv_handle_deinit();
}

const PebbleProcessMd* mfg_hrm_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &prv_main,
    .name = "MfgHRM",
  };
  return (const PebbleProcessMd*) &s_app_info;
}

#endif // CAPABILITY_HAS_BUILTIN_HRM
