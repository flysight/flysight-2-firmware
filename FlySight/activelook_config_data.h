#ifndef ACTIVELOOK_CONFIG_DATA_H
#define ACTIVELOOK_CONFIG_DATA_H

#include <stdint.h>

#define AL_CONFIG_VERSION 4
#define AL_CONFIG_CMD_COUNT 65

extern const uint16_t al_config_cmd_lengths[AL_CONFIG_CMD_COUNT];
extern const uint16_t al_config_cmd_offsets[AL_CONFIG_CMD_COUNT];
extern const uint8_t al_config_cmd_data[];

#endif // ACTIVELOOK_CONFIG_DATA_H
