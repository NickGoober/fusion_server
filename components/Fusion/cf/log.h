/* Shim: the Crazyflie logging framework (telemetry variables exposed over
 * the radio) does not exist here; the registration macros compile away. */
#pragma once

#define LOG_GROUP_START(name)
#define LOG_GROUP_STOP(name)
#define LOG_ADD(...)
#define LOG_ADD_CORE(...)
