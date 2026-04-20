/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "vc31_algo_adapter.h"
#include "algo.h"

#include "services/common/hrm/hrm_manager.h"
#include "services/common/accel_manager_types.h"
#include "system/logging.h"

#include <string.h>

// Convert KX022 mg (1G = 1000) to blob scale (1G = 256)
#define ACCEL_MG_TO_BLOB(mg) ((int32_t)(mg) * 256 / 1000)

static struct {
  uint8_t last_bpm;
  uint8_t last_reliability;
  uint32_t ms_since_last_publish;
  bool initialized;
} s_algo_state;

void vc31_algo_init(void) {
  Algo_Init();
  memset(&s_algo_state, 0, sizeof(s_algo_state));
  s_algo_state.initialized = true;
}

void vc31_algo_reset(void) {
  if (!s_algo_state.initialized) {
    vc31_algo_init();
    return;
  }
  Algo_Init();
  s_algo_state.last_bpm = 0;
  s_algo_state.last_reliability = 0;
  s_algo_state.ms_since_last_publish = 0;
}

bool vc31_algo_feed_sample(int32_t ppg, int32_t env, uint32_t gap_ms,
                           bool was_adjusted,
                           uint8_t *out_bpm, uint8_t *out_reliability) {
  if (!s_algo_state.initialized) {
    vc31_algo_init();
  }

  // Gather accel data
  AlgoAxesData_t axes = {0};
  HRMAccelData *accel_data = hrm_manager_get_accel_data();
  if (accel_data && accel_data->num_samples > 0) {
    AccelRawData *sample = &accel_data->data[accel_data->num_samples - 1];
    axes.x = ACCEL_MG_TO_BLOB(sample->x);
    axes.y = ACCEL_MG_TO_BLOB(sample->y);
    axes.z = ACCEL_MG_TO_BLOB(sample->z);
  }
  hrm_manager_release_accel_data();

  // Build input
  AlgoInputData_t inputData;
  inputData.axes = axes;
  inputData.ppgSample = ppg | (was_adjusted ? 0x1000 : 0);
  inputData.envSample = env;

  // Feed algorithm
  Algo_Input(&inputData, (int32_t)gap_ms, SPORT_TYPE_NORMAL, 0, 0);

  AlgoOutputData_t outputData;
  Algo_Output(&outputData);

  s_algo_state.ms_since_last_publish += gap_ms;

  bool had_update = false;
  if (outputData.hrData != s_algo_state.last_bpm ||
      outputData.reliability != s_algo_state.last_reliability ||
      (s_algo_state.ms_since_last_publish > 2000 &&
       outputData.hrData != 0 && outputData.reliability != 0)) {
    s_algo_state.last_bpm = (uint8_t)outputData.hrData;
    s_algo_state.last_reliability = (uint8_t)outputData.reliability;
    s_algo_state.ms_since_last_publish = 0;
    had_update = true;
  }

  if (had_update && out_bpm && out_reliability) {
    *out_bpm = s_algo_state.last_bpm;
    *out_reliability = s_algo_state.last_reliability;
  }

  return had_update;
}
