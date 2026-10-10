#include "internal.h"

manipulator_status manipulator_internal_dh(const manipulator_dh_joint *j, manipulator_real q, manipulator_transform *out)
{
    manipulator_real theta = j->theta_offset, d = j->d_offset;
    if (j->type == MANIPULATOR_REVOLUTE) {
        if (!manipulator_checked_add(theta, q, &theta)) return MANIPULATOR_NUMERIC_ERROR;
    } else if (!manipulator_checked_add(d, q, &d)) return MANIPULATOR_NUMERIC_ERROR;
    manipulator_real ct = MANIPULATOR_COS(theta), st = MANIPULATOR_SIN(theta);
    manipulator_real ca = MANIPULATOR_COS(j->alpha), sa = MANIPULATOR_SIN(j->alpha);
    if (!isfinite(ct) || !isfinite(st) || !isfinite(ca) || !isfinite(sa)) return MANIPULATOR_NUMERIC_ERROR;
    manipulator_transform r = {{{ct, -st*ca, st*sa, 0},
                       {st, ct*ca, -ct*sa, 0},
                       {0, sa, ca, d}, {0, 0, 0, 1}}};
    if (!manipulator_checked_mul(j->a, ct, &r.m[0][3])
        || !manipulator_checked_mul(j->a, st, &r.m[1][3])) return MANIPULATOR_NUMERIC_ERROR;
    if (manipulator_transform_validate(&r) != MANIPULATOR_OK) return MANIPULATOR_NUMERIC_ERROR;
    *out = r;
    return MANIPULATOR_OK;
}
