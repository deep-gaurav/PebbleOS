// Stub vibe driver for banglejs2
#include "drivers/vibe.h"

void vibe_init(void) {}
void vibe_ctl(bool on) {}
void vibe_force_off(void) {}
void vibe_set_strength(int8_t strength) {}
int8_t vibe_get_braking_strength(void) {
  return VIBE_STRENGTH_MIN;
}
status_t vibe_calibrate(void) {
  return S_SUCCESS;
}
void command_vibe_ctl(const char *arg) {}
