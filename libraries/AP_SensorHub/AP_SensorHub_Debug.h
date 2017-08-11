#pragma once

#include <AP_HAL/AP_HAL.h>

// NOTE: For now make debugging/error tracking features optional.
#define SENSORHUB_DEBUG_NONE 0
#define SENSORHUB_DEBUG_FILE 1
#define SENSORHUB_DEBUG_CONSOLE 2
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define SENSORHUB_DEBUG SENSORHUB_DEBUG_CONSOLE
#else
#define SENSORHUB_DEBUG SENSORHUB_DEBUG_NONE
#endif