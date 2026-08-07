/* Shim for the Crazyflie Kbuild-generated configuration header.
 *
 * GENERAL_PURPOSE selects the estimator variant that never assumes the
 * platform is a flying quad (quadIsFlying stays false and thrust-specific
 * dynamics are disabled), which matches this project's use of the filter.
 */
#pragma once

#define CONFIG_ESTIMATOR_KALMAN_GENERAL_PURPOSE 1
