#ifndef MANIPULATOR_TYPES_H
#define MANIPULATOR_TYPES_H
#include <stddef.h>
#include <manipulator/config.h>
#if K_USE_FLOAT
typedef float k_real;
#else
typedef double k_real;
#endif
typedef enum
{
    K_OK = 0,
    K_INVALID_ARGUMENT = 1,
    K_INVALID_MODEL = 2,
    K_SIZE_MISMATCH = 3,
    K_BUFFER_TOO_SMALL = 4,
    K_NONFINITE_INPUT = 5,
    K_NUMERIC_ERROR = 6,
    K_INVALID_TRANSFORM = 7,
    K_NOT_READY = 8,
    K_INVALID_STATE = 9,
    K_INCOMPLETE_CONFIGURATION = 10,
    K_JOINT_LIMIT = 11,
    K_INVALID_OPTIONS = 12,
    K_NO_CONVERGENCE = 13,
    K_STALLED = 14
} k_status;

typedef enum
{
    K_REVOLUTE = 0,
    K_PRISMATIC = 1
} k_joint_type;

typedef struct
{
    k_real m[4][4];
} k_transform;

typedef struct
{
    k_joint_type type;
    k_real a, alpha, d_offset, theta_offset;
} k_dh_joint;

typedef enum
{
    K_UNBOUNDED = 0,
    K_BOUNDED = 1
} k_limit_mode;

typedef struct
{
    k_limit_mode mode;
    k_real min, max;
} k_joint_limits;

typedef struct
{
    k_dh_joint geometry;
    k_joint_limits limits;
    unsigned configured;
} k_joint_config;

typedef enum
{
    K_EMPTY = 0,
    K_CONFIGURING = 1,
    K_READY = 2
} k_model_state;

typedef struct
{
    k_joint_config *joints;
    size_t joint_count, storage_capacity;
    k_transform world_base, last_tool;
    k_model_state state;
    unsigned configured;
} k_model;

typedef struct
{
    k_real x, y, z;
} k_vec3;
#endif
