#include "collar_gravity.h"

#include <math.h>
#include <stddef.h>

void collarGravitySetWorld(collar_gravity_world_t *g, float wx, float wy, float wz)
{
    if (g == NULL) {
        return;
    }
    g->wx = wx;
    g->wy = wy;
    g->wz = wz;
}

bool collarGravityHat(const collar_gravity_world_t *g, float *hx, float *hy, float *hz)
{
    if (g == NULL || hx == NULL || hy == NULL || hz == NULL) {
        return false;
    }
    const float mag = sqrtf(g->wx * g->wx + g->wy * g->wy + g->wz * g->wz);
    if (mag < 1e-4f) {
        *hx = 0.0f;
        *hy = 0.0f;
        *hz = 1.0f;
        return false;
    }
    const float inv = 1.0f / mag;
    *hx = g->wx * inv;
    *hy = g->wy * inv;
    *hz = g->wz * inv;
    return true;
}

void collarGravityBodyFromR(const float R[3][3], const collar_gravity_world_t *g, float gb[3])
{
    if (R == NULL || g == NULL || gb == NULL) {
        return;
    }
  // v_world = R * v_body  =>  v_body = R^T * v_world
    gb[0] = R[0][0] * g->wx + R[1][0] * g->wy + R[2][0] * g->wz;
    gb[1] = R[0][1] * g->wx + R[1][1] * g->wy + R[2][1] * g->wz;
    gb[2] = R[0][2] * g->wx + R[1][2] * g->wy + R[2][2] * g->wz;
}

float collarGravityHeightM(const float pos_xyz[3], const collar_gravity_world_t *g)
{
    float hx = 0.0f;
    float hy = 0.0f;
    float hz = 1.0f;
    collarGravityHat(g, &hx, &hy, &hz);
    const float along = pos_xyz[0] * hx + pos_xyz[1] * hy + pos_xyz[2] * hz;
    float h = -along;
    if (h < 0.02f) {
        h = 0.02f;
    }
    return h;
}

void collarGravityHeightStateJacobian(
    const collar_gravity_world_t *g,
    float scale,
    float jac_out[3])
{
    if (g == NULL || jac_out == NULL) {
        return;
    }
    float hx = 0.0f;
    float hy = 0.0f;
    float hz = 1.0f;
    collarGravityHat(g, &hx, &hy, &hz);
    jac_out[0] = -hx * scale;
    jac_out[1] = -hy * scale;
    jac_out[2] = -hz * scale;
}

float collarGravityBodyZCoupling(const float R[3][3], const collar_gravity_world_t *g)
{
    float hx = 0.0f;
    float hy = 0.0f;
    float hz = 1.0f;
    collarGravityHat(g, &hx, &hy, &hz);
    /*
     * |body_down · ĝ|. v_world = R * v_body, so body axis j in world is column j.
     * Collar Y-up: radar/flow look along −Y (down) → column 1.
     * (Crazyflie used body +Z / R[2][2], which is only correct when ĝ is world +Z.)
     */
    const float dot = R[0][1] * hx + R[1][1] * hy + R[2][1] * hz;
    const float c = fabsf(dot);
    return c < 0.05f ? 0.05f : c;
}
