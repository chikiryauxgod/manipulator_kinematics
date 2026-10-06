#ifndef MANIPULATOR_TYPES_H
#define MANIPULATOR_TYPES_H
#include <stddef.h>
#include <manipulator/config.h>
#if MK_USE_FLOAT
typedef float mk_real;
#else
typedef double mk_real;
#endif
typedef enum {
    MK_OK = 0, MK_INVALID_ARGUMENT = 1, MK_INVALID_MODEL = 2,
    MK_SIZE_MISMATCH = 3, MK_BUFFER_TOO_SMALL = 4,
    MK_NONFINITE_INPUT = 5, MK_NUMERIC_ERROR = 6, MK_INVALID_TRANSFORM = 7
} mk_status;
typedef enum { MK_REVOLUTE = 0, MK_PRISMATIC = 1 } mk_joint_type;
/* Row-major; maps column vectors from child coordinates to parent coordinates. */
typedef struct { mk_real m[4][4]; } mk_transform;
/* Standard DH: Rz(theta) Tz(d) Tx(a) Rx(alpha). Lengths in m, angles in rad. */
typedef struct {
    mk_joint_type type;
    mk_real a, alpha, d_offset, theta_offset;
} mk_dh_joint;
/* Borrows joints; caller keeps the array alive and immutable during calls. */
typedef struct {
    const mk_dh_joint *joints;
    size_t joint_count;
    mk_transform world_base, last_tool;
} mk_model;
#endif
