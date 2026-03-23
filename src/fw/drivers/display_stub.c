// Stub display driver for banglejs2
#include "display/display.h"

void display_init(void) {}
uint32_t display_baud_rate_change(uint32_t new_frequency_hz) { return 0; }
void display_clear(void) {}
void display_set_enabled(bool enabled) {}
void display_set_rotated(bool rotated) {}
void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {}
bool display_update_in_progress(void) { return false; }
void display_pulse_vcom(void) {}
void display_show_panic_screen(uint32_t error_code) {}
void display_set_offset(GPoint offset) {}
GPoint display_get_offset(void) { GPoint p = {0,0}; return p; }
