/* Subset of the Crazyflie stabilizer_types.h / imu_types.h definitions,
 * limited to the types referenced by the vendored Kalman core and the
 * flow / ToF measurement models. Layouts are kept identical to upstream. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef struct attitude_s {
    uint32_t timestamp;

    float roll;
    float pitch;
    float yaw;
} attitude_t;

struct vec3_s {
    uint32_t timestamp;

    float x;
    float y;
    float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct quaternion_s {
    union {
        struct {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

typedef struct state_s {
    attitude_t attitude;      // deg (legacy CF2 body coordinate system, pitch inverted)
    quaternion_t attitudeQuaternion;
    point_t position;         // m
    velocity_t velocity;      // m/s
    acc_t acc;                // Gs (acc.z without gravity)
} state_t;

/** Flow measurement **/
typedef struct flowMeasurement_s {
    uint32_t timestamp;
    union {
        struct {
            float dpixelx;  // Accumulated pixel count x
            float dpixely;  // Accumulated pixel count y
        };
        float dpixel[2];
    };
    float stdDevX;
    float stdDevY;
    float dt;               // Time during which pixels were accumulated
} flowMeasurement_t;

/** TOF measurement **/
typedef struct tofMeasurement_s {
    uint32_t timestamp;
    float distance;         // m
    float stdDev;
} tofMeasurement_t;

#ifdef __cplusplus
}
#endif
