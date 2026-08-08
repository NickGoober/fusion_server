#include "fusion_lever_arm_cal.h"

#include <math.h>
#include <string.h>

#define CAL_FLOW_RESOLUTION 0.10f
#define CAL_NPIX 35.0f
#define CAL_THETAPIX 0.71674f
#define CAL_MIN_RANGE_M 0.02f
#define CAL_MIN_OMEGA_RAD_S 0.005f
#define CAL_MIN_SAMPLES 30U
#define CAL_CROSS_AXIS_MAX_FRAC 0.35f
#define CAL_MAX_ACCEL_MS2 50.0f
#define CAL_AXIS_PROBE_MIN 20U

typedef struct {
    bool active;
    fusion_cal_axis_t axis;
    float expected_omega_rad_s;
    float omega_tol_rad_s;
    bool variable_rate;
    bool axis_auto;
    bool axis_locked;
    double probe_sum_abs_gx;
    double probe_sum_abs_gy;
    double probe_sum_abs_gz;
    uint32_t probe_samples;
    bool flow_swap_xy;
    bool flow_invert_x;
    bool flow_invert_y;
    float flow_scale;
    float flow_scale_y;
    fusion_vec3_t prior_imu_arm_m;
    fusion_vec3_t prior_flow_arm_m;
    uint32_t samples_used;
    uint32_t samples_rejected;
    double sum_imu_rx;
    double sum_imu_ry;
    double sum_imu_rz;
    uint32_t imu_rx_count;
    uint32_t imu_ry_count;
    uint32_t imu_rz_count;
    double sum_flow_rx;
    double sum_flow_ry;
    double sum_flow_rz;
    uint32_t flow_rx_count;
    uint32_t flow_ry_count;
    uint32_t flow_rz_count;
    double sum_residual_sq;
    double sum_omega_abs;
    uint32_t omega_count;
} lever_arm_cal_state_t;

static lever_arm_cal_state_t s_cal;

static float cal_flow_dt_s(float dt_s)
{
    if (dt_s < 0.001f || dt_s > 0.5f) {
        return 0.01f;
    }
    return dt_s;
}

static void cal_remap_flow(
    int16_t raw_dx,
    int16_t raw_dy,
    const lever_arm_cal_state_t *st,
    float *bx,
    float *by)
{
    float x = (float)raw_dx;
    float y = (float)raw_dy;
    float out_x = st->flow_swap_xy ? y : x;
    float out_y = st->flow_swap_xy ? x : y;
    if (st->flow_invert_x) {
        out_x = -out_x;
    }
    if (st->flow_invert_y) {
        out_y = -out_y;
    }
    *bx = out_x;
    *by = out_y;
}

static bool cal_flow_to_body_velocity(
    float bx,
    float by,
    float gx_rad_s,
    float gy_rad_s,
    float range_m,
    float dt_s,
    const lever_arm_cal_state_t *st,
    float *v_cam_bx,
    float *v_cam_by)
{
    float z_g = range_m;
    if (z_g < CAL_MIN_RANGE_M) {
        z_g = CAL_MIN_RANGE_M;
    }

    const float scale = st->flow_scale;
    const float scale_y = st->flow_scale_y > 0.0f ? st->flow_scale_y : st->flow_scale;
    const float dpixelx = bx * scale;
    const float dpixely = by * scale_y;
    const float meas_x = dpixelx * CAL_FLOW_RESOLUTION;
    const float meas_y = dpixely * CAL_FLOW_RESOLUTION;
    const float flow_scale = (dt_s * CAL_NPIX / CAL_THETAPIX);
    if (flow_scale < 1e-6f) {
        return false;
    }

    *v_cam_bx = z_g * (meas_x / flow_scale + gy_rad_s);
    *v_cam_by = z_g * (meas_y / flow_scale - gx_rad_s);
    return isfinite(*v_cam_bx) && isfinite(*v_cam_by);
}

static bool cal_rotation_ok(
    fusion_cal_axis_t axis,
    float gx,
    float gy,
    float gz,
    float expected_omega,
    float omega_tol,
    bool variable_rate)
{
    const float abs_x = fabsf(gx);
    const float abs_y = fabsf(gy);
    const float abs_z = fabsf(gz);
    float dominant = 0.0f;
    float cross_max = 0.0f;

    switch (axis) {
    case FUSION_CAL_AXIS_X:
        dominant = abs_x;
        cross_max = fmaxf(abs_y, abs_z);
        if (!variable_rate && expected_omega > 0.0f
            && fabsf(dominant - fabsf(expected_omega)) > omega_tol) {
            return false;
        }
        break;
    case FUSION_CAL_AXIS_Y:
        dominant = abs_y;
        cross_max = fmaxf(abs_x, abs_z);
        if (!variable_rate && expected_omega > 0.0f
            && fabsf(dominant - fabsf(expected_omega)) > omega_tol) {
            return false;
        }
        break;
    case FUSION_CAL_AXIS_Z:
        dominant = abs_z;
        cross_max = fmaxf(abs_x, abs_y);
        if (!variable_rate && expected_omega > 0.0f
            && fabsf(dominant - fabsf(expected_omega)) > omega_tol) {
            return false;
        }
        break;
    default:
        return false;
    }

    if (dominant < CAL_MIN_OMEGA_RAD_S) {
        return false;
    }
    if (cross_max > dominant * CAL_CROSS_AXIS_MAX_FRAC) {
        return false;
    }
    return true;
}

static bool cal_rotation_ok_any_axis(float gx, float gy, float gz)
{
    const float abs_x = fabsf(gx);
    const float abs_y = fabsf(gy);
    const float abs_z = fabsf(gz);
    const float dominant = fmaxf(abs_x, fmaxf(abs_y, abs_z));
    if (dominant < CAL_MIN_OMEGA_RAD_S) {
        return false;
    }
    float cross_max = 0.0f;
    if (dominant == abs_x) {
        cross_max = fmaxf(abs_y, abs_z);
    } else if (dominant == abs_y) {
        cross_max = fmaxf(abs_x, abs_z);
    } else {
        cross_max = fmaxf(abs_x, abs_y);
    }
    if (cross_max > dominant * CAL_CROSS_AXIS_MAX_FRAC) {
        return false;
    }
    return true;
}

static fusion_cal_axis_t cal_axis_from_probe_sums(
    double sum_x,
    double sum_y,
    double sum_z)
{
    if (sum_x >= sum_y && sum_x >= sum_z) {
        return FUSION_CAL_AXIS_X;
    }
    if (sum_y >= sum_x && sum_y >= sum_z) {
        return FUSION_CAL_AXIS_Y;
    }
    return FUSION_CAL_AXIS_Z;
}

static bool cal_try_lock_axis(lever_arm_cal_state_t *st)
{
    if (!st->axis_auto || st->axis_locked) {
        return st->axis_locked;
    }
    if (st->probe_samples < CAL_AXIS_PROBE_MIN) {
        return false;
    }
    st->axis = cal_axis_from_probe_sums(
        st->probe_sum_abs_gx,
        st->probe_sum_abs_gy,
        st->probe_sum_abs_gz);
    st->axis_locked = true;
    return true;
}

static float cal_signed_omega(fusion_cal_axis_t axis, float gx, float gy, float gz)
{
    switch (axis) {
    case FUSION_CAL_AXIS_X:
        return gx;
    case FUSION_CAL_AXIS_Y:
        return gy;
    case FUSION_CAL_AXIS_Z:
        return gz;
    default:
        return 0.0f;
    }
}

static void cal_omega_cross_r(
    float gx,
    float gy,
    float gz,
    float rx,
    float ry,
    float rz,
    float *vx,
    float *vy,
    float *vz)
{
    *vx = gy * rz - gz * ry;
    *vy = gz * rx - gx * rz;
    *vz = gx * ry - gy * rx;
}

static bool cal_estimate_imu_from_accel(
    fusion_cal_axis_t axis,
    float omega,
    float ax,
    float ay,
    float az,
    float *rx,
    float *ry,
    float *rz)
{
    if (fabsf(omega) < CAL_MIN_OMEGA_RAD_S) {
        return false;
    }
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
        return false;
    }
    if (fabsf(ax) > CAL_MAX_ACCEL_MS2 || fabsf(ay) > CAL_MAX_ACCEL_MS2 || fabsf(az) > CAL_MAX_ACCEL_MS2) {
        return false;
    }

    const float inv_omega2 = 1.0f / (omega * omega);
    *rx = 0.0f;
    *ry = 0.0f;
    *rz = 0.0f;

    switch (axis) {
    case FUSION_CAL_AXIS_X:
        *ry = -az * inv_omega2;
        *rz = ay * inv_omega2;
        return true;
    case FUSION_CAL_AXIS_Y:
        *rx = ax * inv_omega2;
        *rz = az * inv_omega2;
        return true;
    case FUSION_CAL_AXIS_Z:
        *rx = -ax * inv_omega2;
        *ry = -ay * inv_omega2;
        return true;
    default:
        return false;
    }
}

static bool cal_estimate_flow_from_velocity(
    fusion_cal_axis_t axis,
    float gx,
    float gy,
    float gz,
    float v_cam_bx,
    float v_cam_by,
    float imu_rx,
    float imu_ry,
    float imu_rz,
    float *rx,
    float *ry,
    float *rz,
    float *residual_sq)
{
    float v_imu_x = 0.0f;
    float v_imu_y = 0.0f;
    float v_imu_z = 0.0f;
    cal_omega_cross_r(gx, gy, gz, imu_rx, imu_ry, imu_rz, &v_imu_x, &v_imu_y, &v_imu_z);

    const float v_flow_x = v_cam_bx - v_imu_x;
    const float v_flow_y = v_cam_by - v_imu_y;
    const float omega = cal_signed_omega(axis, gx, gy, gz);
    if (fabsf(omega) < CAL_MIN_OMEGA_RAD_S) {
        return false;
    }

    *rx = 0.0f;
    *ry = 0.0f;
    *rz = 0.0f;
    *residual_sq = 0.0f;

    switch (axis) {
    case FUSION_CAL_AXIS_X:
        *rz = -v_flow_y / omega;
        *residual_sq = v_flow_x * v_flow_x;
        return true;
    case FUSION_CAL_AXIS_Y:
        *rz = v_flow_x / omega;
        *residual_sq = v_flow_y * v_flow_y;
        return true;
    case FUSION_CAL_AXIS_Z:
        *ry = -v_flow_x / omega;
        *rx = v_flow_y / omega;
        return true;
    default:
        return false;
    }
}

static fusion_vec3_t cal_running_imu_arm(const lever_arm_cal_state_t *st)
{
    fusion_vec3_t arm = st->prior_imu_arm_m;
    if (st->imu_rx_count > 0U) {
        arm.x = (float)(st->sum_imu_rx / (double)st->imu_rx_count);
    }
    if (st->imu_ry_count > 0U) {
        arm.y = (float)(st->sum_imu_ry / (double)st->imu_ry_count);
    }
    if (st->imu_rz_count > 0U) {
        arm.z = (float)(st->sum_imu_rz / (double)st->imu_rz_count);
    }
    return arm;
}

bool lever_arm_cal_begin(
    fusion_cal_axis_t axis,
    float expected_omega_rad_s,
    float omega_tol_rad_s,
    const fusion_vec3_t *prior_imu_arm_m,
    const fusion_vec3_t *prior_flow_arm_m)
{
    if (axis > FUSION_CAL_AXIS_AUTO || prior_imu_arm_m == NULL || prior_flow_arm_m == NULL) {
        return false;
    }
    if (!isfinite(expected_omega_rad_s)) {
        return false;
    }
    const bool variable_rate = expected_omega_rad_s <= 0.0f;
    if (!variable_rate && omega_tol_rad_s <= 0.0f) {
        omega_tol_rad_s = 0.25f * fabsf(expected_omega_rad_s);
        if (omega_tol_rad_s < 0.002f) {
            omega_tol_rad_s = 0.002f;
        }
    }

    memset(&s_cal, 0, sizeof(s_cal));
    s_cal.active = true;
    s_cal.axis = axis;
    s_cal.expected_omega_rad_s = expected_omega_rad_s;
    s_cal.omega_tol_rad_s = omega_tol_rad_s;
    s_cal.variable_rate = variable_rate;
    s_cal.axis_auto = (axis == FUSION_CAL_AXIS_AUTO);
    s_cal.axis_locked = !s_cal.axis_auto;
    if (s_cal.axis_auto) {
        s_cal.axis = FUSION_CAL_AXIS_AUTO;
    }
    s_cal.prior_imu_arm_m = *prior_imu_arm_m;
    s_cal.prior_flow_arm_m = *prior_flow_arm_m;
    s_cal.flow_scale = 1.0f;
    return true;
}

void lever_arm_cal_set_flow_mapping(
    bool flow_swap_xy,
    bool flow_invert_x,
    bool flow_invert_y,
    float flow_scale,
    float flow_scale_y)
{
    s_cal.flow_swap_xy = flow_swap_xy;
    s_cal.flow_invert_x = flow_invert_x;
    s_cal.flow_invert_y = flow_invert_y;
    s_cal.flow_scale = flow_scale > 0.0f ? flow_scale : 1.0f;
    s_cal.flow_scale_y = flow_scale_y;
}

bool lever_arm_cal_feed(
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
    if (!s_cal.active) {
        return false;
    }

    if (s_cal.axis_auto && !s_cal.axis_locked) {
        if (!cal_rotation_ok_any_axis(gx_rad_s, gy_rad_s, gz_rad_s)) {
            s_cal.samples_rejected++;
            return false;
        }
        s_cal.probe_sum_abs_gx += fabsf(gx_rad_s);
        s_cal.probe_sum_abs_gy += fabsf(gy_rad_s);
        s_cal.probe_sum_abs_gz += fabsf(gz_rad_s);
        s_cal.probe_samples++;
        if (!cal_try_lock_axis(&s_cal)) {
            return true;
        }
    }

    if (!cal_rotation_ok(
            s_cal.axis,
            gx_rad_s,
            gy_rad_s,
            gz_rad_s,
            s_cal.expected_omega_rad_s,
            s_cal.omega_tol_rad_s,
            s_cal.variable_rate)) {
        s_cal.samples_rejected++;
        return false;
    }

    const float omega = cal_signed_omega(s_cal.axis, gx_rad_s, gy_rad_s, gz_rad_s);

    float imu_rx = 0.0f;
    float imu_ry = 0.0f;
    float imu_rz = 0.0f;
    if (!cal_estimate_imu_from_accel(s_cal.axis, omega, ax_mps2, ay_mps2, az_mps2, &imu_rx, &imu_ry, &imu_rz)) {
        s_cal.samples_rejected++;
        return false;
    }

    switch (s_cal.axis) {
    case FUSION_CAL_AXIS_X:
        s_cal.sum_imu_ry += imu_ry;
        s_cal.sum_imu_rz += imu_rz;
        s_cal.imu_ry_count++;
        s_cal.imu_rz_count++;
        break;
    case FUSION_CAL_AXIS_Y:
        s_cal.sum_imu_rx += imu_rx;
        s_cal.sum_imu_rz += imu_rz;
        s_cal.imu_rx_count++;
        s_cal.imu_rz_count++;
        break;
    case FUSION_CAL_AXIS_Z:
        s_cal.sum_imu_rx += imu_rx;
        s_cal.sum_imu_ry += imu_ry;
        s_cal.imu_rx_count++;
        s_cal.imu_ry_count++;
        break;
    default:
        break;
    }

    float bx = 0.0f;
    float by = 0.0f;
    cal_remap_flow(flow_dx, flow_dy, &s_cal, &bx, &by);

    float v_cam_bx = 0.0f;
    float v_cam_by = 0.0f;
    const float range_m = (float)range_mm / 1000.0f;
    if (!cal_flow_to_body_velocity(
            bx,
            by,
            gx_rad_s,
            gy_rad_s,
            range_m,
            cal_flow_dt_s(dt_s),
            &s_cal,
            &v_cam_bx,
            &v_cam_by)) {
        s_cal.samples_rejected++;
        return false;
    }

    const fusion_vec3_t imu_arm = cal_running_imu_arm(&s_cal);
    float flow_rx = 0.0f;
    float flow_ry = 0.0f;
    float flow_rz = 0.0f;
    float residual_sq = 0.0f;
    if (!cal_estimate_flow_from_velocity(
            s_cal.axis,
            gx_rad_s,
            gy_rad_s,
            gz_rad_s,
            v_cam_bx,
            v_cam_by,
            imu_arm.x,
            imu_arm.y,
            imu_arm.z,
            &flow_rx,
            &flow_ry,
            &flow_rz,
            &residual_sq)) {
        s_cal.samples_rejected++;
        return false;
    }

    switch (s_cal.axis) {
    case FUSION_CAL_AXIS_X:
    case FUSION_CAL_AXIS_Y:
        s_cal.sum_flow_rz += flow_rz;
        s_cal.flow_rz_count++;
        break;
    case FUSION_CAL_AXIS_Z:
        s_cal.sum_flow_rx += flow_rx;
        s_cal.sum_flow_ry += flow_ry;
        s_cal.flow_rx_count++;
        s_cal.flow_ry_count++;
        break;
    default:
        break;
    }

    s_cal.sum_residual_sq += residual_sq;
    s_cal.sum_omega_abs += fabsf(omega);
    s_cal.omega_count++;
    s_cal.samples_used++;
    return true;
}

bool lever_arm_cal_finish(fusion_lever_arm_cal_result_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->axis = s_cal.axis;
    if (s_cal.omega_count > 0U) {
        out->omega_rad_s = (float)(s_cal.sum_omega_abs / (double)s_cal.omega_count);
    } else {
        out->omega_rad_s = s_cal.expected_omega_rad_s;
    }
    out->samples_used = s_cal.samples_used;
    out->samples_rejected = s_cal.samples_rejected;

    if (!s_cal.active) {
        return false;
    }

    if (s_cal.samples_used < CAL_MIN_SAMPLES) {
        lever_arm_cal_cancel();
        return false;
    }

    fusion_vec3_t imu_arm = s_cal.prior_imu_arm_m;
    fusion_vec3_t flow_arm = s_cal.prior_flow_arm_m;
    if (s_cal.imu_rx_count > 0U) {
        imu_arm.x = (float)(s_cal.sum_imu_rx / (double)s_cal.imu_rx_count);
    }
    if (s_cal.imu_ry_count > 0U) {
        imu_arm.y = (float)(s_cal.sum_imu_ry / (double)s_cal.imu_ry_count);
    }
    if (s_cal.imu_rz_count > 0U) {
        imu_arm.z = (float)(s_cal.sum_imu_rz / (double)s_cal.imu_rz_count);
    }
    if (s_cal.flow_rx_count > 0U) {
        flow_arm.x = (float)(s_cal.sum_flow_rx / (double)s_cal.flow_rx_count);
    }
    if (s_cal.flow_ry_count > 0U) {
        flow_arm.y = (float)(s_cal.sum_flow_ry / (double)s_cal.flow_ry_count);
    }
    if (s_cal.flow_rz_count > 0U) {
        flow_arm.z = (float)(s_cal.sum_flow_rz / (double)s_cal.flow_rz_count);
    }

    out->success = true;
    out->imu_lever_arm_m = imu_arm;
    out->flow_lever_arm_m = flow_arm;
    if (s_cal.samples_used > 0U) {
        out->residual_rms_mps = (float)sqrt(s_cal.sum_residual_sq / (double)s_cal.samples_used);
    }

    memset(&s_cal, 0, sizeof(s_cal));
    return true;
}

void lever_arm_cal_cancel(void)
{
    memset(&s_cal, 0, sizeof(s_cal));
}

void lever_arm_cal_get_status(fusion_lever_arm_cal_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->active = s_cal.active;
    out->axis = s_cal.axis;
    out->axis_auto = s_cal.axis_auto;
    out->axis_locked = s_cal.axis_locked;
    out->expected_omega_rad_s = s_cal.expected_omega_rad_s;
    out->samples_used = s_cal.samples_used;
    out->samples_rejected = s_cal.samples_rejected;
}
