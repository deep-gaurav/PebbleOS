// Stub ambient light driver for banglejs2
#include "drivers/ambient_light.h"

void ambient_light_init(void) {}
uint32_t ambient_light_get_light_level(void) { return 0; }
void ambient_light_set_dark_threshold(uint32_t value) {}
bool ambient_light_is_light(void) { return false; }
AmbientLightLevel ambient_light_level_to_enum(uint32_t light_level) { return AmbientLightLevelDark; }
void command_als_read(void) {}
