/**
 * World-gravity helpers for collar fusion (arbitrary gravity vector, default +X).
 */
#pragma once

#include <stdbool.h>

typedef struct {
    float wx;
    float wy;
    float wz;
} collar_gravity_world_t;

/** Set world-frame gravity acceleration vector (m/s²). */
void collarGravitySetWorld(collar_gravity_world_t *g, float wx, float wy, float wz);

/** Unit vector in the direction gravity pulls (world frame). Returns false if |g|≈0. */
bool collarGravityHat(const collar_gravity_world_t *g, float *hx, float *hy, float *hz);

/**
 * Expected gravity-acceleration vector in body frame: g_body = R^T * g_world.
 * R maps body→world (same layout as kalmanCoreData_t::R).
 */
void collarGravityBodyFromR(const float R[3][3], const collar_gravity_world_t *g, float gb[3]);

/** Altitude above origin along the “up” axis (−ĝ). */
float collarGravityHeightM(const float pos_xyz[3], const collar_gravity_world_t *g);

/**
 * Jacobian of height w.r.t. (X,Y,Z) world states: ∂h/∂pos = −ĝ.
 * jac_out[0..2] are multiplied by a caller scale (e.g. 1/coupling or flow term).
 */
void collarGravityHeightStateJacobian(
    const collar_gravity_world_t *g,
    float scale,
    float jac_out[3]);

/**
 * Flow / ToF coupling: |body_down · ĝ|.
 * Collar Y-up: down is −Y (R column 1). Replaces Crazyflie R[2][2] (body +Z).
 */
float collarGravityBodyZCoupling(const float R[3][3], const collar_gravity_world_t *g);
