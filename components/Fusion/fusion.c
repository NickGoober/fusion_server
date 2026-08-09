#include "fusion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
#include "pc_shim.h"

#include "kalman_core.h"
#include "kalman_supervisor.h"
#include "math3d.h"
#include "mm_flow.h"
#include "mm_tof.h"
#include "physicalConstants.h"
#include "fusion_lever_arm_cal.h"

/*
 * Glue between the Raedir sensor set and the vendored Crazyflie EKF.
 *
 * Data flow per fusion step (runs only when a complete fresh sensor set
 * exists — BNO085 quat + gyro + accel, PMW3901 flow, XM125 range):
 *
 *   1. kalmanCorePredict     gyro (rad/s) + reconstructed specific force
 *   2. kalmanCoreAddProcessNoise
 *   3. quaternion anchor     BNO085 attitude as measurement (mm_pose math)
 *   4. kalmanCoreUpdateWithFlow  accumulated PMW3901 pixels
 *   5. kalmanCoreUpdateWithTof    XM125 distance (innovation gated)
 *   6. kalmanCoreFinalize + supervisor bounds check + externalize
 *
 * The BNO085 streams *linear* acceleration (gravity already removed), while
 * the EKF prediction expects raw specific force. We reconstruct it as
 * a_spec = a_lin + g * R^T e3 using the filter's own attitude, which cancels
 * exactly against the gravity subtraction inside the prediction.
 */

// Bounds used to reject corrupt samples at the API boundary.
#define FUSION_MAX_GYRO_RAD_S   70.0f    // 2x a BNO085's 2000 dps limit
#define FUSION_MAX_ACCEL_MS2    400.0f   // ~40 g
#define FUSION_MIN_QUAT_NORM    0.5f
#define FUSION_MAX_QUAT_NORM    2.0f
#define FUSION_MIN_FLOW_DT_S    0.001f
#define FUSION_MAX_FLOW_DT_S    0.5f
#define FUSION_FLOW_FRAME_INTERVAL_US 10000  // PMW3901 nominal frame period

// Set by kalman_supervisor.c (vendored); we override them from the config.
extern float maxPosition;
extern float maxVelocity;

typedef struct {
    bool fresh;
    int64_t timestamp_us;
} fusion_slot_t;

typedef struct {
    // Latest attitude sample (already normalized, IMU frame).
    struct quat quat_imu;
    // Averaged rates/accelerations between steps (IMU frame).
    float gyro_sum[3];
    uint32_t gyro_n;
    float accel_sum[3];
    uint32_t accel_n;
    // Flow accumulation between steps (raw sensor counts).
    int32_t flow_acc_x;
    int32_t flow_acc_y;
    uint32_t flow_frames;
    uint8_t flow_min_quality_seen;
    int64_t flow_last_frame_us;
    int64_t flow_anchor_us;          // last frame of the previous window
    // Latest range sample.
    float range_m;
} fusion_inputs_t;

static bool s_initialized;
static const char *s_status_reason = "not initialized";
static SemaphoreHandle_t s_mutex;

static fusion_config_t s_cfg;
static kalmanCoreData_t s_core;
static kalmanCoreParams_t s_core_params;

static fusion_inputs_t s_in;
static fusion_slot_t s_slot_quat;
static fusion_slot_t s_slot_gyro;
static fusion_slot_t s_slot_accel;
static fusion_slot_t s_slot_flow;
static fusion_slot_t s_slot_range;

static bool s_attitude_aligned;
static bool s_has_pose;
static fusion_pose_t s_pose;
static fusion_stats_t s_stats;
static bool s_debug_log = true;

#define FUSION_DBG(...) do { if (s_debug_log) { printf(__VA_ARGS__); } } while (0)

void fusion_set_debug_logging(bool enable)
{
    s_debug_log = enable;
}

static bool fusion_lock(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE;
}

static void fusion_unlock(void)
{
    xSemaphoreGive(s_mutex);
}

static bool fusion_finite3(float a, float b, float c)
{
    return isfinite(a) && isfinite(b) && isfinite(c);
}

void fusion_config_defaults(fusion_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    cfg->flow_std_pixels = 1.878726f;          // matches the Crazyflie flow deck driver was 2.0f
    cfg->flow_scale = 1.0f;                    // HW counts * scale * FLOW_RESOLUTION -> pixels
    cfg->flow_scale_y = 1.0f;                  // 0 = same as flow_scale
    // Default axis mapping mirrors the Crazyflie flow deck mounting
    // (body_x = -sensor_y, body_y = -sensor_x). Adjust to actual mounting.
    cfg->flow_swap_xy = true;
    cfg->flow_invert_x = true;
    cfg->flow_invert_y = true;
    cfg->flow_min_quality = 0;            // module SQUAL semantics vary; off by default
    cfg->flow_max_pixels_per_frame = 200;

    cfg->range_std_m = 0.003332f;             // XM125 close-range accuracy
    cfg->range_gate_sigma = 5.0f;
    cfg->range_min_m = 0.03f;
    cfg->range_max_m = 10.0f;

    cfg->quat_std_rad = 0.02967f;            // ~1.7 deg, BNO085 dynamic accuracy class
    cfg->attitude_snap_angle_rad = 0.5f;  // beyond ~29 deg residual: snap, don't filter
    cfg->imu_to_body = (fusion_quat_t){ .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };

    cfg->require_flow = true;
    cfg->require_range = true;
    cfg->max_sample_age_ms = 1000;        // XM125 peaks can be sparse; allow wider alignment
    cfg->max_predict_dt_s = 0.2f;
    cfg->pose_stale_ms = 1000;

    cfg->max_position_m = 100.0f;
    cfg->max_velocity_mps = 10.0f;

    cfg->kalman_proc_noise_acc_xy = 2.926502f; //both used to be 0
    cfg->kalman_proc_noise_vel = 0.329756f;
}

static void fusion_clear_window_locked(void)
{
    memset(s_in.gyro_sum, 0, sizeof(s_in.gyro_sum));
    s_in.gyro_n = 0;
    memset(s_in.accel_sum, 0, sizeof(s_in.accel_sum));
    s_in.accel_n = 0;
    s_in.flow_acc_x = 0;
    s_in.flow_acc_y = 0;
    s_in.flow_frames = 0;
    s_in.flow_min_quality_seen = 0xFFU;

    s_slot_quat.fresh = false;
    s_slot_gyro.fresh = false;
    s_slot_accel.fresh = false;
    s_slot_flow.fresh = false;
    s_slot_range.fresh = false;
}

static void fusion_apply_attitude_locked(struct quat q)
{
    q = qnormalize(q);
    s_core.q[0] = q.w;
    s_core.q[1] = q.x;
    s_core.q[2] = q.y;
    s_core.q[3] = q.z;

    const float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    s_core.R[0][0] = qw * qw + qx * qx - qy * qy - qz * qz;
    s_core.R[0][1] = 2 * qx * qy - 2 * qw * qz;
    s_core.R[0][2] = 2 * qx * qz + 2 * qw * qy;
    s_core.R[1][0] = 2 * qx * qy + 2 * qw * qz;
    s_core.R[1][1] = qw * qw - qx * qx + qy * qy - qz * qz;
    s_core.R[1][2] = 2 * qy * qz - 2 * qw * qx;
    s_core.R[2][0] = 2 * qx * qz - 2 * qw * qy;
    s_core.R[2][1] = 2 * qy * qz + 2 * qw * qx;
    s_core.R[2][2] = qw * qw - qx * qx - qy * qy + qz * qz;

    s_core.S[KC_STATE_D0] = 0.0f;
    s_core.S[KC_STATE_D1] = 0.0f;
    s_core.S[KC_STATE_D2] = 0.0f;
}

static void fusion_core_reset_locked(int64_t now_us)
{
FUSION_DBG("[FUSION CORE RESET] Resetting filter state at %lld us\n", (long long)now_us);
    const uint32_t now_ms = (uint32_t)(now_us / 1000);
    kalmanCoreInit(&s_core, &s_core_params, now_ms);
    if (now_ms >= 10U) {
        s_core.lastPredictionMs = now_ms - 10U;
    }
    s_attitude_aligned = false;
    fusion_clear_window_locked();
    s_in.flow_anchor_us = 0;
}

static struct vec fusion_imu_to_body(struct vec v)
{
    const fusion_quat_t *m = &s_cfg.imu_to_body;
    if (m->w == 1.0f && m->x == 0.0f && m->y == 0.0f && m->z == 0.0f) {
        return v;
    }
    return qvrot(mkquat(m->x, m->y, m->z, m->w), v);
}

static struct quat fusion_measured_body_attitude(void)
{
    const fusion_quat_t *m = &s_cfg.imu_to_body;
    struct quat q_ws = s_in.quat_imu;
    if (m->w == 1.0f && m->x == 0.0f && m->y == 0.0f && m->z == 0.0f) {
        return q_ws;
    }
    return qnormalize(qqmul(q_ws, qinv(mkquat(m->x, m->y, m->z, m->w))));
}

static void fusion_update_with_quat_locked(void)
{
    const struct quat q_meas = fusion_measured_body_attitude();

    if (!s_attitude_aligned) {
    FUSION_DBG("[ATTITUDE ALIGN] Aligning filter attitude to initial measurement.\n");
        fusion_apply_attitude_locked(q_meas);
        s_attitude_aligned = true;
        return;
    }

    const struct quat q_ekf = mkquat(s_core.q[1], s_core.q[2], s_core.q[3], s_core.q[0]);
    struct quat q_residual = qqmul(q_meas, qinv(q_ekf));
    if (q_residual.w < 0.0f) {
        q_residual = mkquat(-q_residual.x, -q_residual.y, -q_residual.z, -q_residual.w);
    }

    const float residual_angle = 2.0f * acosf(fminf(q_residual.w, 1.0f));
    if (residual_angle > s_cfg.attitude_snap_angle_rad) {
    FUSION_DBG("[ATTITUDE SNAP] Residual angle (%.3f rad) > threshold (%.3f rad). Snapping attitude!\n",
               residual_angle, s_cfg.attitude_snap_angle_rad);
        fusion_apply_attitude_locked(q_meas);
        for (int i = KC_STATE_D0; i <= KC_STATE_D2; i++) {
            for (int j = 0; j < KC_STATE_DIM; j++) {
                s_core.P[i][j] = 0.0f;
                s_core.P[j][i] = 0.0f;
            }
        }
        const float rp_var = powf(s_core_params.stdDevInitialAttitude_rollpitch, 2);
        const float y_var = powf(s_core_params.stdDevInitialAttitude_yaw, 2);
        s_core.P[KC_STATE_D0][KC_STATE_D0] = rp_var;
        s_core.P[KC_STATE_D1][KC_STATE_D1] = rp_var;
        s_core.P[KC_STATE_D2][KC_STATE_D2] = y_var;
        s_core.isUpdated = true;
        s_stats.attitude_snaps++;
        return;
    }

    const struct vec err = vscl(2.0f / q_residual.w, quatimagpart(q_residual));

    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

    h[KC_STATE_D0] = 1;
    kalmanCoreScalarUpdate(&s_core, &H, err.x, s_cfg.quat_std_rad);
    h[KC_STATE_D0] = 0;

    h[KC_STATE_D1] = 1;
    kalmanCoreScalarUpdate(&s_core, &H, err.y, s_cfg.quat_std_rad);
    h[KC_STATE_D1] = 0;

    h[KC_STATE_D2] = 1;
    kalmanCoreScalarUpdate(&s_core, &H, err.z, s_cfg.quat_std_rad);

    s_stats.quat_updates++;
}

static void fusion_update_with_flow_locked(const Axis3f *gyro_avg_rad, float step_dt_s)
{
    if (s_in.flow_frames == 0) {
        return;
    }

    if (s_in.flow_min_quality_seen < s_cfg.flow_min_quality) {
    FUSION_DBG("[SKIP FLOW] Low quality: seen=%u < min=%u\n", s_in.flow_min_quality_seen, s_cfg.flow_min_quality);
        s_stats.flow_skipped++;
        return;
    }

    float raw_x = (float)s_in.flow_acc_x;
    float raw_y = (float)s_in.flow_acc_y;
    float bx = s_cfg.flow_swap_xy ? raw_y : raw_x;
    float by = s_cfg.flow_swap_xy ? raw_x : raw_y;
    if (s_cfg.flow_invert_x) {
        bx = -bx;
    }
    if (s_cfg.flow_invert_y) {
        by = -by;
    }

    float dt = step_dt_s;
    if (s_in.flow_anchor_us > 0 && s_in.flow_last_frame_us > s_in.flow_anchor_us) {
        dt = (float)(s_in.flow_last_frame_us - s_in.flow_anchor_us) / 1e6f;
    } else if (s_in.flow_frames > 0) {
        /* First window: anchor not set yet — match accumulated frame count. */
        dt = (float)s_in.flow_frames * (FUSION_FLOW_FRAME_INTERVAL_US / 1e6f);
    }
    if (dt < FUSION_MIN_FLOW_DT_S || dt > FUSION_MAX_FLOW_DT_S) {
    FUSION_DBG("[SKIP FLOW] Invalid dt: %.4fs (min=%.4f, max=%.4f)\n", dt, FUSION_MIN_FLOW_DT_S, FUSION_MAX_FLOW_DT_S);
        s_stats.flow_skipped++;
        return;
    }

    flowMeasurement_t flow = {
        .dpixelx = bx * s_cfg.flow_scale,
        .dpixely = by * (s_cfg.flow_scale_y > 0.0f ? s_cfg.flow_scale_y : s_cfg.flow_scale),
        .stdDevX = s_cfg.flow_std_pixels,
        .stdDevY = s_cfg.flow_std_pixels,
        .dt = dt,
    };

    Axis3f gyro_deg = {
        .x = gyro_avg_rad->x * RAD_TO_DEG,
        .y = gyro_avg_rad->y * RAD_TO_DEG,
        .z = gyro_avg_rad->z * RAD_TO_DEG,
    };

    kalmanCoreUpdateWithFlow(&s_core, &flow, &gyro_deg);
    s_stats.flow_updates++;
}

static void fusion_update_with_range_locked(void)
{
    const float dist = s_in.range_m;
    if (dist < s_cfg.range_min_m || dist > s_cfg.range_max_m) {
    FUSION_DBG("[REJECT RANGE] Distance out of bounds: %.3fm (min=%.3fm, max=%.3fm)\n", dist, s_cfg.range_min_m, s_cfg.range_max_m);
        s_stats.range_rejected++;
        return;
    }

    const float r22 = s_core.R[2][2];
    if (r22 < 0.5f) {
    FUSION_DBG("[REJECT RANGE] Excessive tilt: r22=%.3f < 0.5\n", r22);
        s_stats.range_rejected++;
        return;
    }

    const float cos_a = r22;
    const float predicted = s_core.S[KC_STATE_Z] / cos_a;
    const float h_z = 1.0f / cos_a;
    
    const float innovation = dist - predicted;
    const float innovation_var = h_z * h_z * s_core.P[KC_STATE_Z][KC_STATE_Z]
                               + s_cfg.range_std_m * s_cfg.range_std_m;
    const float gate = s_cfg.range_gate_sigma;
    
    if (s_stats.range_updates > 10) {
        if (innovation * innovation > gate * gate * innovation_var) {
        FUSION_DBG("[REJECT RANGE] Innovation gate failed: dist=%.3fm, pred=%.3fm, inn=%.3fm, limit=%.4f\n",
                   dist, predicted, innovation, gate * gate * innovation_var);
            s_stats.range_rejected++;
            return;
        }
    }

    tofMeasurement_t tof = {
        .distance = dist,
        .stdDev = s_cfg.range_std_m,
    };
    kalmanCoreUpdateWithTof(&s_core, &tof);
    s_stats.range_updates++;
}

static void fusion_externalize_locked(int64_t now_us, const Axis3f *acc_spec_ms2)
{
    Axis3f acc_g = {
        .x = acc_spec_ms2->x / GRAVITY_MAGNITUDE,
        .y = acc_spec_ms2->y / GRAVITY_MAGNITUDE,
        .z = acc_spec_ms2->z / GRAVITY_MAGNITUDE,
    };
    state_t st;
    memset(&st, 0, sizeof(st));
    kalmanCoreExternalizeState(&s_core, &st, &acc_g);

    fusion_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    pose.timestamp_us = now_us;
    pose.step_count = s_stats.steps + 1;
    pose.position_m = (fusion_vec3_t){ st.position.x, st.position.y, st.position.z };
    pose.velocity_mps = (fusion_vec3_t){ st.velocity.x, st.velocity.y, st.velocity.z };
    pose.rotation = (fusion_quat_t){
        .w = st.attitudeQuaternion.w,
        .x = st.attitudeQuaternion.x,
        .y = st.attitudeQuaternion.y,
        .z = st.attitudeQuaternion.z,
    };

    float qw = pose.rotation.w;
    float qx = pose.rotation.x;
    float qy = pose.rotation.y;
    float qz = pose.rotation.z;
    if (qw < 0.0f) {
        qw = -qw;
        qx = -qx;
        qy = -qy;
        qz = -qz;
    }
    const float sin_half = sqrtf(qx * qx + qy * qy + qz * qz);
    if (sin_half > 1e-6f) {
        const float rv_angle = 2.0f * atan2f(sin_half, qw);
        const float k = rv_angle / sin_half;
        pose.rotation_vector_rad = (fusion_vec3_t){ qx * k, qy * k, qz * k };
    }

    pose.euler_rpy_rad = (fusion_vec3_t){
        .x = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)),
        .y = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (qw * qy - qz * qx)))),
        .z = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)),
    };
    pose.valid = true;

    s_pose = pose;
    s_has_pose = true;
    s_stats.steps++;
}

static bool fusion_slot_ready_locked(const fusion_slot_t *slot, int64_t now_us, int64_t max_age_us)
{
    if (!slot->fresh) {
        return false;
    }
    return (now_us - slot->timestamp_us) <= max_age_us;
}

static int64_t fusion_reference_time_us_locked(void)
{
    int64_t t = 0;
    if (s_slot_quat.timestamp_us > t) {
        t = s_slot_quat.timestamp_us;
    }
    if (s_slot_gyro.timestamp_us > t) {
        t = s_slot_gyro.timestamp_us;
    }
    if (s_slot_accel.timestamp_us > t) {
        t = s_slot_accel.timestamp_us;
    }
    if (s_slot_flow.timestamp_us > t) {
        t = s_slot_flow.timestamp_us;
    }
    if (s_slot_range.timestamp_us > t) {
        t = s_slot_range.timestamp_us;
    }
    if (t == 0) {
        t = esp_timer_get_time();
    }
    return t;
}

static bool fusion_set_complete_locked(int64_t now_us)
{
    const int64_t max_age_us = (int64_t)s_cfg.max_sample_age_ms * 1000;

    bool q_ok = fusion_slot_ready_locked(&s_slot_quat, now_us, max_age_us);
    bool g_ok = fusion_slot_ready_locked(&s_slot_gyro, now_us, max_age_us);
    bool a_ok = fusion_slot_ready_locked(&s_slot_accel, now_us, max_age_us);
    bool f_ok = !s_cfg.require_flow || fusion_slot_ready_locked(&s_slot_flow, now_us, max_age_us);
    bool r_ok = !s_cfg.require_range || fusion_slot_ready_locked(&s_slot_range, now_us, max_age_us);

    if (!q_ok || !g_ok || !a_ok || !f_ok || !r_ok) {
        return false;
    }
    return true;
}

static void fusion_step_locked(int64_t now_us)
{
    const uint32_t now_ms = (uint32_t)(now_us / 1000);

    const uint32_t max_dt_ms = (uint32_t)(s_cfg.max_predict_dt_s * 1000.0f);
    if ((uint32_t)(now_ms - s_core.lastPredictionMs) > max_dt_ms) {
    FUSION_DBG("[STEP DT CLAMP] Large gap detected! Clamping dt to %u ms\n", max_dt_ms);
        s_core.lastPredictionMs = now_ms - max_dt_ms;
    }
    const float step_dt_s = (float)(now_ms - s_core.lastPredictionMs) / 1000.0f;

    Axis3f gyro_imu = {0};
    if (s_in.gyro_n > 0) {
        gyro_imu.x = s_in.gyro_sum[0] / (float)s_in.gyro_n;
        gyro_imu.y = s_in.gyro_sum[1] / (float)s_in.gyro_n;
        gyro_imu.z = s_in.gyro_sum[2] / (float)s_in.gyro_n;
    }
    Axis3f accel_imu = {0};
    if (s_in.accel_n > 0) {
        accel_imu.x = s_in.accel_sum[0] / (float)s_in.accel_n;
        accel_imu.y = s_in.accel_sum[1] / (float)s_in.accel_n;
        accel_imu.z = s_in.accel_sum[2] / (float)s_in.accel_n;
    }

    struct vec gyro_v = fusion_imu_to_body(mkvec(gyro_imu.x, gyro_imu.y, gyro_imu.z));
    struct vec accel_v = fusion_imu_to_body(mkvec(accel_imu.x, accel_imu.y, accel_imu.z));
    Axis3f gyro_body = { .x = gyro_v.x, .y = gyro_v.y, .z = gyro_v.z };

    Axis3f acc_spec = {
        .x = accel_v.x + GRAVITY_MAGNITUDE * s_core.R[2][0],
        .y = accel_v.y + GRAVITY_MAGNITUDE * s_core.R[2][1],
        .z = accel_v.z + GRAVITY_MAGNITUDE * s_core.R[2][2],
    };

    kalmanCorePredict(&s_core, &s_core_params, &acc_spec, &gyro_body, now_ms, false);
    kalmanCoreAddProcessNoise(&s_core, &s_core_params, now_ms);

    fusion_update_with_quat_locked();
    if (s_cfg.require_flow) {
        fusion_update_with_flow_locked(&gyro_body, step_dt_s);
    }
    if (s_cfg.require_range) {
        fusion_update_with_range_locked();
    }

    (void)kalmanCoreFinalize(&s_core);

    if (!kalmanSupervisorIsStateWithinBounds(&s_core)) {
    FUSION_DBG("[SUPERVISOR RESET] State out of bounds! Resetting core. Pos: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f)\n",
               s_core.S[KC_STATE_X], s_core.S[KC_STATE_Y], s_core.S[KC_STATE_Z],
               s_core.S[KC_STATE_PX], s_core.S[KC_STATE_PY], s_core.S[KC_STATE_PZ]);
        s_stats.supervisor_resets++;
        s_pose.valid = false;
        fusion_core_reset_locked(now_us);
        s_status_reason = "state out of bounds - filter reset";
        return;
    }

    fusion_externalize_locked(now_us, &acc_spec);
    s_status_reason = "fusing";

    if (s_in.flow_frames > 0) {
        s_in.flow_anchor_us = s_in.flow_last_frame_us;
    }
    fusion_clear_window_locked();
}

static void fusion_maybe_step_locked(void)
{
    if (!s_attitude_aligned) {
        return;
    }
    const int64_t now_us = fusion_reference_time_us_locked();
    if (fusion_set_complete_locked(now_us)) {
        fusion_step_locked(now_us);
    }
}

static bool fusion_init_internal(const fusion_config_t *cfg)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            s_status_reason = "mutex allocation failed";
            return false;
        }
    }

    if (!fusion_lock()) {
        s_status_reason = "lock failed";
        return false;
    }

    s_cfg = *cfg;

    kalmanCoreDefaultParams(&s_core_params);
    s_core_params.attitudeReversion = 0.0f;
    if (s_cfg.kalman_proc_noise_acc_xy > 0.0f) {
        s_core_params.procNoiseAcc_xy = s_cfg.kalman_proc_noise_acc_xy;
    }
    if (s_cfg.kalman_proc_noise_vel > 0.0f) {
        s_core_params.procNoiseVel = s_cfg.kalman_proc_noise_vel;
    }

    maxPosition = s_cfg.max_position_m;
    maxVelocity = s_cfg.max_velocity_mps;

    memset(&s_in, 0, sizeof(s_in));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_pose, 0, sizeof(s_pose));
    s_has_pose = false;

    fusion_core_reset_locked(esp_timer_get_time());

    mm_flow_set_position(
        cfg->flow_lever_arm_m.x,
        cfg->flow_lever_arm_m.y,
        cfg->flow_lever_arm_m.z);
    mm_flow_set_imu_pivot_offset(
        cfg->imu_lever_arm_m.x,
        cfg->imu_lever_arm_m.y,
        cfg->imu_lever_arm_m.z);

    s_initialized = true;
    s_status_reason = "waiting for sensor data";
    fusion_unlock();
    return true;
}

bool fusion_init(void)
{
    fusion_config_t cfg;
    fusion_config_defaults(&cfg);
    return fusion_init_internal(&cfg);
}

bool fusion_init_imu_only(void)
{
    fusion_config_t cfg;
    fusion_config_defaults(&cfg);
    cfg.require_flow = false;
    cfg.require_range = false;
    return fusion_init_internal(&cfg);
}

bool fusion_init_with_config(const fusion_config_t *cfg)
{
    if (cfg == NULL) {
        s_status_reason = "null config";
        return false;
    }
    return fusion_init_internal(cfg);
}

bool fusion_is_ready(void)
{
    return s_initialized;
}

const char *fusion_status_reason(void)
{
    return s_status_reason;
}

void fusion_submit_imu_quat(float w, float x, float y, float z, int64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }
    if (!isfinite(w) || !fusion_finite3(x, y, z)) {
    FUSION_DBG("[REJECT QUAT] Non-finite value: w=%.2f x=%.2f y=%.2f z=%.2f\n", w, x, y, z);
        s_stats.rejected_inputs++;
        return;
    }
    const float norm = sqrtf(w * w + x * x + y * y + z * z);
    if (norm < FUSION_MIN_QUAT_NORM || norm > FUSION_MAX_QUAT_NORM) {
    FUSION_DBG("[REJECT QUAT] Norm out of bounds: %.4f (min=%.2f, max=%.2f)\n", norm, FUSION_MIN_QUAT_NORM, FUSION_MAX_QUAT_NORM);
        s_stats.rejected_inputs++;
        return;
    }

    if (!fusion_lock()) {
        return;
    }
    s_in.quat_imu = mkquat(x / norm, y / norm, z / norm, w / norm);
    s_slot_quat.fresh = true;
    s_slot_quat.timestamp_us = timestamp_us;

    if (!s_attitude_aligned) {
        fusion_apply_attitude_locked(fusion_measured_body_attitude());
        s_attitude_aligned = true;
    }

    fusion_maybe_step_locked();
    fusion_unlock();
}

void fusion_submit_imu_gyro(float gx_rad_s, float gy_rad_s, float gz_rad_s, int64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }
    if (!fusion_finite3(gx_rad_s, gy_rad_s, gz_rad_s) ||
        fabsf(gx_rad_s) > FUSION_MAX_GYRO_RAD_S ||
        fabsf(gy_rad_s) > FUSION_MAX_GYRO_RAD_S ||
        fabsf(gz_rad_s) > FUSION_MAX_GYRO_RAD_S) {
    FUSION_DBG("[REJECT GYRO] Out of bounds or non-finite: [%.2f, %.2f, %.2f]\n", gx_rad_s, gy_rad_s, gz_rad_s);
        s_stats.rejected_inputs++;
        return;
    }

    if (!fusion_lock()) {
        return;
    }
    s_in.gyro_sum[0] += gx_rad_s;
    s_in.gyro_sum[1] += gy_rad_s;
    s_in.gyro_sum[2] += gz_rad_s;
    s_in.gyro_n++;
    s_slot_gyro.fresh = true;
    s_slot_gyro.timestamp_us = timestamp_us;

    fusion_maybe_step_locked();
    fusion_unlock();
}

void fusion_submit_imu_accel(float ax_ms2, float ay_ms2, float az_ms2, int64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }
    if (!fusion_finite3(ax_ms2, ay_ms2, az_ms2) ||
        fabsf(ax_ms2) > FUSION_MAX_ACCEL_MS2 ||
        fabsf(ay_ms2) > FUSION_MAX_ACCEL_MS2 ||
        fabsf(az_ms2) > FUSION_MAX_ACCEL_MS2) {
    FUSION_DBG("[REJECT ACCEL] Out of bounds or non-finite: [%.2f, %.2f, %.2f]\n", ax_ms2, ay_ms2, az_ms2);
        s_stats.rejected_inputs++;
        return;
    }

    if (!fusion_lock()) {
        return;
    }
    s_in.accel_sum[0] += ax_ms2;
    s_in.accel_sum[1] += ay_ms2;
    s_in.accel_sum[2] += az_ms2;
    s_in.accel_n++;
    s_slot_accel.fresh = true;
    s_slot_accel.timestamp_us = timestamp_us;

    fusion_maybe_step_locked();
    fusion_unlock();
}

void fusion_submit_flow(int16_t dx_pixels, int16_t dy_pixels, uint8_t quality, int64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }
    if ((int32_t)abs(dx_pixels) > s_cfg.flow_max_pixels_per_frame ||
        (int32_t)abs(dy_pixels) > s_cfg.flow_max_pixels_per_frame) {
    FUSION_DBG("[REJECT FLOW] Pixel delta limit exceeded: dx=%d, dy=%d (max=%d)\n",
               dx_pixels, dy_pixels, s_cfg.flow_max_pixels_per_frame);
        s_stats.rejected_inputs++;
        return;
    }

    if (!fusion_lock()) {
        return;
    }
    s_in.flow_acc_x += dx_pixels;
    s_in.flow_acc_y += dy_pixels;
    s_in.flow_frames++;
    if (quality < s_in.flow_min_quality_seen) {
        s_in.flow_min_quality_seen = quality;
    }
    s_in.flow_last_frame_us = timestamp_us;
    s_slot_flow.fresh = true;
    s_slot_flow.timestamp_us = timestamp_us;

    fusion_maybe_step_locked();
    fusion_unlock();
}

void fusion_submit_range(uint16_t distance_mm, int64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }

    if (!fusion_lock()) {
        return;
    }
    s_in.range_m = (float)distance_mm / 1000.0f;
    s_slot_range.fresh = true;
    s_slot_range.timestamp_us = timestamp_us;

    fusion_maybe_step_locked();
    fusion_unlock();
}

bool fusion_get_pose(fusion_pose_t *out)
{
    if (out == NULL || !s_initialized) {
        return false;
    }
    if (!fusion_lock()) {
        return false;
    }
    const bool has_pose = s_has_pose;
    *out = s_pose;
    if (has_pose) {
        const int64_t age_us = esp_timer_get_time() - s_pose.timestamp_us;
        if (age_us > (int64_t)s_cfg.pose_stale_ms * 1000) {
            out->valid = false;
        }
    }
    fusion_unlock();
    return has_pose;
}

void fusion_get_stats(fusion_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    if (!fusion_lock()) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_stats;
    fusion_unlock();
}

static int32_t fusion_age_ms(int64_t timestamp_us, int64_t now_us)
{
    if (!timestamp_us) {
        return -1;
    }
    return (int32_t)((now_us - timestamp_us) / 1000LL);
}

void fusion_get_input_status(fusion_input_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (!s_initialized || !fusion_lock()) {
        return;
    }

    const int64_t now_us = fusion_reference_time_us_locked();
    out->quat_fresh = s_slot_quat.fresh;
    out->gyro_fresh = s_slot_gyro.fresh;
    out->accel_fresh = s_slot_accel.fresh;
    out->flow_fresh = s_slot_flow.fresh;
    out->range_fresh = s_slot_range.fresh;
    out->quat_age_ms = fusion_age_ms(s_slot_quat.timestamp_us, now_us);
    out->gyro_age_ms = fusion_age_ms(s_slot_gyro.timestamp_us, now_us);
    out->accel_age_ms = fusion_age_ms(s_slot_accel.timestamp_us, now_us);
    out->flow_age_ms = fusion_age_ms(s_slot_flow.timestamp_us, now_us);
    out->range_age_ms = fusion_age_ms(s_slot_range.timestamp_us, now_us);
    out->set_complete = fusion_set_complete_locked(now_us);
    fusion_unlock();
}

void fusion_reset(void)
{
    if (!s_initialized) {
        return;
    }
    if (!fusion_lock()) {
        return;
    }
    s_pose.valid = false;
    s_has_pose = false;
    fusion_core_reset_locked(esp_timer_get_time());
    s_status_reason = "reset - waiting for sensor data";
    fusion_unlock();
}

void fusion_set_flow_lever_arm(float x_m, float y_m, float z_m)
{
    if (!fusion_lock()) {
        return;
    }
    s_cfg.flow_lever_arm_m.x = x_m;
    s_cfg.flow_lever_arm_m.y = y_m;
    s_cfg.flow_lever_arm_m.z = z_m;
    mm_flow_set_position(x_m, y_m, z_m);
    fusion_unlock();
}

void fusion_get_flow_lever_arm(fusion_vec3_t *out)
{
    if (out == NULL) {
        return;
    }
    if (!fusion_lock()) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_cfg.flow_lever_arm_m;
    fusion_unlock();
}

void fusion_set_imu_lever_arm(float x_m, float y_m, float z_m)
{
    if (!fusion_lock()) {
        return;
    }
    s_cfg.imu_lever_arm_m.x = x_m;
    s_cfg.imu_lever_arm_m.y = y_m;
    s_cfg.imu_lever_arm_m.z = z_m;
    mm_flow_set_imu_pivot_offset(x_m, y_m, z_m);
    fusion_unlock();
}

void fusion_get_imu_lever_arm(fusion_vec3_t *out)
{
    if (out == NULL) {
        return;
    }
    if (!fusion_lock()) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_cfg.imu_lever_arm_m;
    fusion_unlock();
}

void fusion_set_imu_to_body(float w, float x, float y, float z)
{
    if (!fusion_lock()) {
        return;
    }
    const float norm = sqrtf(w * w + x * x + y * y + z * z);
    if (norm < FUSION_MIN_QUAT_NORM || norm > FUSION_MAX_QUAT_NORM) {
        fusion_unlock();
        return;
    }
    s_cfg.imu_to_body.w = w / norm;
    s_cfg.imu_to_body.x = x / norm;
    s_cfg.imu_to_body.y = y / norm;
    s_cfg.imu_to_body.z = z / norm;
    s_attitude_aligned = false;
    fusion_unlock();
}

void fusion_get_imu_to_body(fusion_quat_t *out)
{
    if (out == NULL) {
        return;
    }
    if (!fusion_lock()) {
        memset(out, 0, sizeof(*out));
        out->w = 1.0f;
        return;
    }
    *out = s_cfg.imu_to_body;
    fusion_unlock();
}

bool fusion_lever_arm_cal_start(
    fusion_cal_axis_t axis,
    float expected_omega_rad_s,
    float omega_tol_rad_s)
{
    fusion_vec3_t prior_imu = {0};
    fusion_vec3_t prior_flow = {0};
    fusion_get_imu_lever_arm(&prior_imu);
    fusion_get_flow_lever_arm(&prior_flow);
    if (!lever_arm_cal_begin(
            axis,
            expected_omega_rad_s,
            omega_tol_rad_s,
            &prior_imu,
            &prior_flow)) {
        return false;
    }
    if (!fusion_lock()) {
        lever_arm_cal_cancel();
        return false;
    }
    lever_arm_cal_set_flow_mapping(
        s_cfg.flow_swap_xy,
        s_cfg.flow_invert_x,
        s_cfg.flow_invert_y,
        s_cfg.flow_scale,
        s_cfg.flow_scale_y);
    lever_arm_cal_set_imu_only(!s_cfg.require_flow);
    fusion_unlock();
    return true;
}

bool fusion_lever_arm_cal_feed(
    float gx_rad_s,
    float gy_rad_s,
    float gz_rad_s,
    float ax_mps2,
    float ay_mps2,
    float az_mps2,
    int16_t flow_dx,
    int16_t flow_dy,
    uint16_t range_mm,
    float dt_s)
{
    if (!fusion_lock()) {
        return false;
    }
    struct vec gyro_v = fusion_imu_to_body(mkvec(gx_rad_s, gy_rad_s, gz_rad_s));
    struct vec accel_v = fusion_imu_to_body(mkvec(ax_mps2, ay_mps2, az_mps2));
    fusion_unlock();
    return lever_arm_cal_feed(
        gyro_v.x,
        gyro_v.y,
        gyro_v.z,
        accel_v.x,
        accel_v.y,
        accel_v.z,
        flow_dx,
        flow_dy,
        range_mm,
        dt_s);
}

bool fusion_lever_arm_cal_finish(fusion_lever_arm_cal_result_t *out)
{
    if (!lever_arm_cal_finish(out)) {
        return false;
    }
    if (out->success) {
        fusion_set_imu_lever_arm(
            out->imu_lever_arm_m.x,
            out->imu_lever_arm_m.y,
            out->imu_lever_arm_m.z);
        fusion_set_flow_lever_arm(
            out->flow_lever_arm_m.x,
            out->flow_lever_arm_m.y,
            out->flow_lever_arm_m.z);
    }
    return out->success;
}

void fusion_lever_arm_cal_cancel(void)
{
    lever_arm_cal_cancel();
}

void fusion_lever_arm_cal_get_status(fusion_lever_arm_cal_status_t *out)
{
    lever_arm_cal_get_status(out);
}

bool fusion_lever_arm_cal_get_running_imu_arm(fusion_vec3_t *out)
{
    return lever_arm_cal_get_running_imu_arm(out);
}