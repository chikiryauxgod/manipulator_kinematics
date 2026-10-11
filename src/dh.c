#include "internal.h"

k_status k_internal_dh(const k_dh_joint *j, k_real q, k_transform *out)
{
    k_real theta = j->theta_offset, d = j->d_offset;
    if (j->type == K_REVOLUTE)
    {
        if (!k_checked_add(theta, q, &theta))
        {
            return K_NUMERIC_ERROR;
        }
    }
    else if (!k_checked_add(d, q, &d))
    {
        return K_NUMERIC_ERROR;
    }

    k_real ct = K_COS(theta), st = K_SIN(theta);
    k_real ca = K_COS(j->alpha), sa = K_SIN(j->alpha);
    if (!isfinite(ct) || !isfinite(st) || !isfinite(ca) || !isfinite(sa))
        return K_NUMERIC_ERROR;
    k_transform r = {
        {{ct, -st * ca, st * sa, 0}, {st, ct * ca, -ct * sa, 0}, {0, sa, ca, d}, {0, 0, 0, 1}}};
    if (!k_checked_mul(j->a, ct, &r.m[0][3]) || !k_checked_mul(j->a, st, &r.m[1][3]))
    {
        return K_NUMERIC_ERROR;
    }

    if (k_transform_validate(&r) != K_OK)
    {
        return K_NUMERIC_ERROR;
    }

    *out = r;
    return K_OK;
}
