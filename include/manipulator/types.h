#ifndef MANIPULATOR_TYPES_H
#define MANIPULATOR_TYPES_H
#include <stddef.h>
#include <manipulator/config.h>
#if MANIPULATOR_USE_FLOAT
typedef float manipulator_real;
#else
typedef double manipulator_real;
#endif
typedef enum {
    MANIPULATOR_OK = 0, MANIPULATOR_INVALID_ARGUMENT = 1, MANIPULATOR_INVALID_MODEL = 2,
    MANIPULATOR_SIZE_MISMATCH = 3, MANIPULATOR_BUFFER_TOO_SMALL = 4,
    MANIPULATOR_NONFINITE_INPUT = 5, MANIPULATOR_NUMERIC_ERROR = 6, MANIPULATOR_INVALID_TRANSFORM = 7
} manipulator_status;
typedef enum { MANIPULATOR_REVOLUTE = 0, MANIPULATOR_PRISMATIC = 1 } manipulator_joint_type;

typedef struct { manipulator_real m[4][4]; } manipulator_transform;

typedef struct {
    manipulator_joint_type type;
    manipulator_real a, alpha, d_offset, theta_offset;
} manipulator_dh_joint;

typedef struct {
    const manipulator_dh_joint *joints;
    size_t joint_count;
    manipulator_transform world_base, last_tool;
} manipulator_model;
#endif
