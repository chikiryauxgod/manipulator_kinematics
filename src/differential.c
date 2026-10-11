#include "internal.h"

k_status k_internal_jacobian(const k_model *model, const k_real *joint_positions,
                             const k_transform *tool, k_real *columns)
{
    k_transform current = model->world_base;
    for (size_t i = 0; i < model->joint_count; ++i)
    {
        const k_joint_config *joint = &model->joints[i];
        k_real *column = columns + 3 * i;
        if (joint->limits.mode == K_BOUNDED && joint->limits.min == joint->limits.max)
            column[0] = column[1] = column[2] = 0;
        else if (joint->geometry.type == K_PRISMATIC)
            for (size_t r = 0; r < 3; ++r)
                column[r] = current.m[r][2];
        else
        {
            k_real difference[3];
            for (size_t r = 0; r < 3; ++r)
                if (!k_checked_add(tool->m[r][3], -current.m[r][3], &difference[r]))
                    return K_NUMERIC_ERROR;
            for (size_t r = 0; r < 3; ++r)
            {
                size_t a = (r + 1) % 3, b = (r + 2) % 3;
                k_real first, second;
                if (!k_checked_mul(current.m[a][2], difference[b], &first) ||
                    !k_checked_mul(current.m[b][2], difference[a], &second) ||
                    !k_checked_add(first, -second, &column[r]))
                    return K_NUMERIC_ERROR;
            }
        }
        k_transform local;
        k_status status = k_internal_dh(&joint->geometry, joint_positions[i], &local);
        if (status != K_OK)
            return status;
        status = k_internal_compose(&current, &local, &current);
        if (status != K_OK)
            return status;
    }
    return K_OK;
}
