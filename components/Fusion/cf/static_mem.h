/* Shim: the Crazyflie uses this macro to place buffers in STM32 CCM memory
 * that is not DMA capable. There is no equivalent constraint here.
 *
 * string.h is included because upstream kalman_core.c uses memset/memcpy and
 * receives the declarations transitively from headers that don't exist in
 * this port. */
#pragma once

#include <string.h>

#define NO_DMA_CCM_SAFE_ZERO_INIT
