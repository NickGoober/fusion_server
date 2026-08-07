/* Shim for the Crazyflie per-platform defaults header.
 *
 * The flow sensor is assumed to sit at the body origin (adjust here if the
 * PMW3901 is mounted with a significant lever arm from the IMU). Drag and
 * center-of-pressure terms only apply to the "flying quad" dynamics which
 * are disabled in the general purpose configuration.
 */
#pragma once

#define FLOWDECK_POS_X (0.0f)
#define FLOWDECK_POS_Y (0.0f)
#define FLOWDECK_POS_Z (0.0f)

#define DRAG_B_X (0.0f)
#define DRAG_B_Y (0.0f)
#define DRAG_B_Z (0.0f)

#define CENTER_OF_PRESSURE_X (0.0f)
#define CENTER_OF_PRESSURE_Y (0.0f)
#define CENTER_OF_PRESSURE_Z (0.0f)
