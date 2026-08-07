/* Shim mapping Crazyflie's ASSERT onto the ESP-IDF assert. */
#pragma once

#include <assert.h>

#ifndef ASSERT
#define ASSERT(e) assert(e)
#endif
