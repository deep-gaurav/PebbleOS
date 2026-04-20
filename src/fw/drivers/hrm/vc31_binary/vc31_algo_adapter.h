/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//! Initialize the VC31 proprietary BPM algorithm.
void vc31_algo_init(void);

//! Reset algorithm state (call on off-wrist→on-wrist, enable, or re-probe).
void vc31_algo_reset(void);

//! Feed one PPG sample into the algorithm.
//! @param ppg        Raw PPG value from VC31 sensor.
//! @param env        ENV value from VC31 sensor.
//! @param gap_ms     Milliseconds since the previous sample.
//! @param was_adjusted true if LED/current was just adjusted.
//! @param out_bpm    Set to current BPM when function returns true.
//! @param out_reliability Set to reliability 0..100 when function returns true.
//! @return true if a new BPM reading should be published.
bool vc31_algo_feed_sample(int32_t ppg, int32_t env, uint32_t gap_ms,
                           bool was_adjusted,
                           uint8_t *out_bpm, uint8_t *out_reliability);
