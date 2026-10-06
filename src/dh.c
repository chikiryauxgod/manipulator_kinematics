#include "internal.h"

mk_status mk_internal_dh(const mk_dh_joint *j, mk_real q, mk_transform *out)
{
    mk_real theta = j->theta_offset, d = j->d_offset;
    if (j->type == MK_REVOLUTE) {
        if (!mk_checked_add(theta, q, &theta)) return MK_NUMERIC_ERROR;
    } else if (!mk_checked_add(d, q, &d)) return MK_NUMERIC_ERROR;
    mk_real ct = MK_COS(theta), st = MK_SIN(theta);
    mk_real ca = MK_COS(j->alpha), sa = MK_SIN(j->alpha);
    if (!isfinite(ct) || !isfinite(st) || !isfinite(ca) || !isfinite(sa)) return MK_NUMERIC_ERROR;
    mk_transform r = {{{ct, -st*ca, st*sa, 0},
                       {st, ct*ca, -ct*sa, 0},
                       {0, sa, ca, d}, {0, 0, 0, 1}}};
    if (!mk_checked_mul(j->a, ct, &r.m[0][3])
        || !mk_checked_mul(j->a, st, &r.m[1][3])) return MK_NUMERIC_ERROR;
    if (mk_transform_validate(&r) != MK_OK) return MK_NUMERIC_ERROR;
    *out = r;
    return MK_OK;
}
