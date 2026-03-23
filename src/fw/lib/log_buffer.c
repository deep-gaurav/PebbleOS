/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "log_buffer.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

// Log buffer at fixed RAM address for easy host access via probe-rs
static LogBuffer s_log_buffer __attribute__((section(".log_buffer")));

// Symbol that host can use to find the log buffer
LogBuffer* const g_log_buffer = &s_log_buffer;

void log_init(void) {
    // Check if already initialized by checking magic
    if (s_log_buffer.magic == LOG_BUFFER_MAGIC) {
        // Already initialized
        return;
    }
    
    // Initialize the log buffer
    memset(&s_log_buffer, 0, sizeof(LogBuffer));
    s_log_buffer.magic = LOG_BUFFER_MAGIC;
    s_log_buffer.version = LOG_BUFFER_VERSION;
    s_log_buffer.write_idx = 0;
    s_log_buffer.total_written = 0;
    s_log_buffer.wrapped = 0;
}

void log_write(const char* msg) {
    if (!msg) {
        return;
    }
    
    // Ensure buffer is initialized
    if (s_log_buffer.magic != LOG_BUFFER_MAGIC) {
        log_init();
    }
    
    size_t len = strlen(msg);
    for (size_t i = 0; i < len; i++) {
        s_log_buffer.data[s_log_buffer.write_idx] = msg[i];
        s_log_buffer.write_idx = (s_log_buffer.write_idx + 1) % LOG_BUFFER_SIZE;
        if (s_log_buffer.write_idx == 0) {
            s_log_buffer.wrapped++;
        }
        s_log_buffer.total_written++;
    }
}

void log_write_formatted(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_write(buf);
}