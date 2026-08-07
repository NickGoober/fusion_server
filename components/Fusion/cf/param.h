/* Shim: the Crazyflie parameter framework (runtime-tunable values exposed
 * over the radio) does not exist here. Tunables are surfaced through
 * fusion_config_t instead, so the registration macros compile to nothing. */
#pragma once

#define PARAM_GROUP_START(name)
#define PARAM_GROUP_STOP(name)
#define PARAM_ADD(...)
#define PARAM_ADD_CORE(...)
