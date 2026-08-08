#pragma once

#include "fusion.h"
#include <stdbool.h>
#include <stdint.h>

bool lever_arm_cal_begin(
    fusion_cal_axis_t axis,
    float expected_omega_rad_s,
    float omega_tol_rad_s,
    const fusion_vec3_t *prior_imu_arm_m,
    const fusion_vec3_t *prior_flow_arm_m);

void lever_arm_cal_set_flow_mapping(
    bool flow_swap_xy,
    bool flow_invert_x,
    bool flow_invert_y,
    float flow_scale,
    float flow_scale_y);

bool lever_arm_cal_feed(
    float gx_rad_s, float gy_rad_s, float gz_rad_s,
    float ax_mps2, float ay_mps2, float az_mps2,
    int16_t flow_dx, int16_t flow_dy,
    uint16_t range_mm, float dt_s);

bool lever_arm_cal_finish(fusion_lever_arm_cal_result_t *out);
void lever_arm_cal_cancel(void);
void lever_arm_cal_get_status(fusion_lever_arm_cal_status_t *out);
