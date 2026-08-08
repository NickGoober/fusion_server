#pragma once

/*
 * Component: Fusion
 * =================
 * Sensor fusion built around the Crazyflie extended Kalman filter
 * (Mueller/Hamer error-state EKF, vendored under cf/).
 *
 * Inputs (body frame unless noted):
 *  - BNO085 via CodeCell: game rotation vector (quaternion), calibrated
 *    gyro [rad/s], linear acceleration (gravity removed) [m/s^2]
 *  - PMW3901 optical flow: per-frame pixel deltas + surface quality
 *  - XM125 radar: filtered downward distance [mm]
 *
 * A fusion step runs once all *required* inputs are fresh (IMU quat+gyro+accel
 * always; flow and/or range depending on fusion_config_t.require_*).
 * (world-frame position, velocity, and rotation as quaternion / rotation
 * vector / Euler angles).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} fusion_vec3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} fusion_quat_t;

typedef struct {
    int64_t timestamp_us;             // esp_timer time of the fusion step
    uint32_t step_count;
    fusion_vec3_t position_m;         // world frame, origin at filter start
    fusion_vec3_t velocity_mps;       // world frame
    fusion_quat_t rotation;           // body -> world quaternion
    fusion_vec3_t rotation_vector_rad; // axis * angle representation of rotation
    fusion_vec3_t euler_rpy_rad;      // roll, pitch, yaw (ZYX convention)
    bool valid;                       // false once the solution has gone stale
} fusion_pose_t;

typedef struct {
    uint32_t steps;                   // completed fusion steps
    uint32_t flow_updates;
    uint32_t flow_skipped;            // flow present but rejected (quality)
    uint32_t range_updates;
    uint32_t range_rejected;          // outlier gate / geometry / limits
    uint32_t quat_updates;
    uint32_t attitude_snaps;          // hard attitude re-alignments (large residual)
    uint32_t supervisor_resets;       // full filter resets (state out of bounds)
    uint32_t rejected_inputs;         // NaN / out-of-range samples dropped at the boundary
} fusion_stats_t;

typedef struct {
    // --- Optical flow (PMW3901) ---
    float flow_std_pixels;            // measurement std dev, sensor pixel units
    float flow_scale;                 // scale applied to raw counts (flow X / dpixelx)
    float flow_scale_y;               // scale for flow Y; <=0 uses flow_scale
    bool flow_swap_xy;                // sensor -> body axis mapping (swap first)
    bool flow_invert_x;               // then invert
    bool flow_invert_y;
    uint8_t flow_min_quality;         // frames below this SQUAL are not fused
    int32_t flow_max_pixels_per_frame; // per-frame outlier limit on |dx|,|dy|

    // --- Radar range (XM125, downward facing) ---
    float range_std_m;                // measurement std dev
    float range_gate_sigma;           // innovation gate (sigmas)
    float range_min_m;                // plausible measurement window
    float range_max_m;

    // --- IMU attitude anchor (BNO085 game rotation vector) ---
    float quat_std_rad;               // attitude measurement std dev
    float attitude_snap_angle_rad;    // residual beyond this snaps attitude instead
    fusion_quat_t imu_to_body;        // rotates vectors from IMU frame to body frame

    // PMW3901 position relative to IMU [m] (lever arm for omega x r at flow sensor).
    fusion_vec3_t flow_lever_arm_m;
    // IMU position relative to device rotation center [m] (pivot -> IMU).
    fusion_vec3_t imu_lever_arm_m;

    // --- Step gating / timing ---
    bool require_flow;                // PMW3901 — disable when optical unavailable
    bool require_range;               // XM125 — disable when radar unavailable
    uint32_t max_sample_age_ms;       // samples older than this don't complete a set
    float max_predict_dt_s;           // clamp on prediction integration interval
    uint32_t pose_stale_ms;           // pose validity horizon without new steps

    // --- Supervisor bounds (filter resets beyond these) ---
    float max_position_m;
    float max_velocity_mps;

    // --- Kalman process noise overrides (0 = library default) ---
    float kalman_proc_noise_acc_xy;
    float kalman_proc_noise_vel;
} fusion_config_t;

typedef struct {
    bool quat_fresh;
    bool gyro_fresh;
    bool accel_fresh;
    bool flow_fresh;
    bool range_fresh;
    int32_t quat_age_ms;
    int32_t gyro_age_ms;
    int32_t accel_age_ms;
    int32_t flow_age_ms;
    int32_t range_age_ms;
    bool set_complete;                    // all slots fresh and within max_sample_age_ms
} fusion_input_status_t;

typedef enum {
    FUSION_CAL_AXIS_X = 0,   // body +X (right)
    FUSION_CAL_AXIS_Y = 1,
    FUSION_CAL_AXIS_Z = 2,
} fusion_cal_axis_t;

typedef struct {
    bool success;
    fusion_vec3_t flow_lever_arm_m;
    fusion_vec3_t imu_lever_arm_m;
    uint32_t samples_used;
    uint32_t samples_rejected;
    float residual_rms_mps;
    fusion_cal_axis_t axis;
    float omega_rad_s;
} fusion_lever_arm_cal_result_t;

typedef struct {
    bool active;
    fusion_cal_axis_t axis;
    float expected_omega_rad_s;
    uint32_t samples_used;
    uint32_t samples_rejected;
} fusion_lever_arm_cal_status_t;

void fusion_config_defaults(fusion_config_t *cfg);

bool fusion_init(void);                                // defaults
bool fusion_init_with_config(const fusion_config_t *cfg);
bool fusion_is_ready(void);
const char *fusion_status_reason(void);

// Input submission. Each call is cheap (copy under a mutex); when the call
// completes a fresh set of all inputs, the fusion step runs in the caller's
// context before returning. All submissions are NaN/range checked.
void fusion_submit_imu_quat(float w, float x, float y, float z, int64_t timestamp_us);
void fusion_submit_imu_gyro(float gx_rad_s, float gy_rad_s, float gz_rad_s, int64_t timestamp_us);
void fusion_submit_imu_accel(float ax_ms2, float ay_ms2, float az_ms2, int64_t timestamp_us);
void fusion_submit_flow(int16_t dx_pixels, int16_t dy_pixels, uint8_t quality, int64_t timestamp_us);
void fusion_submit_range(uint16_t distance_mm, int64_t timestamp_us);

// Returns false until the first fused solution exists. out->valid reflects
// whether the solution is recent enough to be trusted.
bool fusion_get_pose(fusion_pose_t *out);
void fusion_get_stats(fusion_stats_t *out);
void fusion_get_input_status(fusion_input_status_t *out);

// Re-initializes the filter (position returns to origin, attitude re-anchors
// to the next IMU quaternion).
void fusion_reset(void);
void fusion_set_debug_logging(bool enable);

// Flow lever arm (PMW3901 offset from IMU, body frame [m]).
void fusion_set_flow_lever_arm(float x_m, float y_m, float z_m);
void fusion_get_flow_lever_arm(fusion_vec3_t *out);

// IMU lever arm (rotation center -> IMU, body frame [m]).
void fusion_set_imu_lever_arm(float x_m, float y_m, float z_m);
void fusion_get_imu_lever_arm(fusion_vec3_t *out);

// Automatic lever-arm calibration: rotate device about a body axis.
// expected_omega_rad_s <= 0 enables variable-rate mode (any spin rate about axis).
// Otherwise the dominant gyro component must match expected_omega within omega_tol.
bool fusion_lever_arm_cal_start(
    fusion_cal_axis_t axis,
    float expected_omega_rad_s,
    float omega_tol_rad_s);
bool fusion_lever_arm_cal_feed(
    float gx_rad_s, float gy_rad_s, float gz_rad_s,
    float ax_mps2, float ay_mps2, float az_mps2,
    int16_t flow_dx, int16_t flow_dy,
    uint16_t range_mm, float dt_s);
bool fusion_lever_arm_cal_finish(fusion_lever_arm_cal_result_t *out);
void fusion_lever_arm_cal_cancel(void);
void fusion_lever_arm_cal_get_status(fusion_lever_arm_cal_status_t *out);

#ifdef __cplusplus
}
#endif
