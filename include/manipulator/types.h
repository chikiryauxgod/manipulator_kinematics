#ifndef MANIPULATOR_TYPES_H
#define MANIPULATOR_TYPES_H
#include <stddef.h>
#include <manipulator/config.h>
#if K_USE_FLOAT
typedef float k_real;
#else
typedef double k_real;
#endif
typedef enum {
    K_OK = 0, K_INVALID_ARGUMENT = 1, K_INVALID_MODEL = 2,
    K_SIZE_MISMATCH = 3, K_BUFFER_TOO_SMALL = 4,
    K_NONFINITE_INPUT = 5, K_NUMERIC_ERROR = 6, K_INVALID_TRANSFORM = 7
} k_status;
typedef enum { K_REVOLUTE = 0, K_PRISMATIC = 1 } k_joint_type;

typedef struct { k_real m[4][4]; } k_transform;

typedef struct {
    k_joint_type type;
    k_real a, alpha, d_offset, theta_offset;
} k_dh_joint;

typedef struct {
    const k_dh_joint *joints;
    size_t joint_count;
    k_transform world_base, last_tool;
} k_model;
#endif
