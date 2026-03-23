/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#define LOG_BUFFER_MAGIC     0x4C4F474C  // "LOG" + 0x4C
#define LOG_BUFFER_SIZE     8192         // 8KB circular buffer
#define LOG_BUFFER_VERSION  1

typedef struct {
    uint32_t magic;           // Magic number to confirm valid structure
    uint32_t version;         // Structure version
    uint32_t write_idx;       // Next position to write (0 to BUFFER_SIZE-1)
    uint32_t total_written;   // Total bytes ever written (for sequence tracking)
    uint32_t wrapped;         // Number of times buffer has wrapped
    char data[LOG_BUFFER_SIZE]; // Circular log data
} LogBuffer;

extern LogBuffer* const g_log_buffer;

void log_init(void);
void log_write(const char* msg);
void log_write_formatted(const char* fmt, ...);