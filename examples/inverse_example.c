#include <manipulator/inverse.h>
#include <manipulator/transform.h>
#include <stdio.h>

int main(void)
{
    const k_real pi = (k_real)3.14159265358979323846;
    const k_dh_joint geometry[3] = {{K_REVOLUTE, 0, pi / 2, (k_real)0.1, 0},
                                    {K_REVOLUTE, (k_real)0.3, 0, 0, 0},
                                    {K_REVOLUTE, (k_real)0.2, 0, 0, 0}};
    static k_joint_config storage[3];
    static k_real workspace[18];
    k_model model = {0};
    k_transform unit;
    const k_joint_limits limits = {K_BOUNDED, -pi, pi};
    k_status status = k_model_begin(&model, storage, 3, 3);
    for (size_t i = 0; status == K_OK && i < 3; ++i)
    {
        status = k_model_set_joint(&model, i, &geometry[i]);
        if (status == K_OK)
            status = k_model_set_limits(&model, i, &limits);
    }
    if (status == K_OK)
        status = k_transform_identity(&unit);
    if (status == K_OK)
        status = k_model_set_base(&model, &unit);
    if (status == K_OK)
        status = k_model_set_tool(&model, &unit);
    if (status == K_OK)
        status = k_model_finalize(&model);
    const k_real joint_scales[3] = {1, 1, 1};
    const k_real max_joint_steps[3] = {(k_real)0.2, (k_real)0.2, (k_real)0.2};
    const k_real initial_joint_positions[3] = {(k_real)0.1, (k_real)0.2, (k_real)0.3};
    k_real joint_positions[3];
    const k_vec3 target = {(k_real)0.3, 0, (k_real)0.3};
    const k_ik_options options = {300,           16,           (k_real)2e-5,    1,
                                  (k_real)0.005, joint_scales, max_joint_steps, 3};
    size_t needed;
    if (status == K_OK)
        status = k_ik_workspace_size(3, &needed);
    if (status == K_OK && needed > sizeof(workspace) / sizeof(workspace[0]))
        status = K_BUFFER_TOO_SMALL;
    k_ik_report report = {0};
    if (status == K_OK)
        status = k_inverse_position(&model, &target, initial_joint_positions, 3, &options,
                                    workspace, sizeof(workspace) / sizeof(workspace[0]),
                                    joint_positions, 3, &report);
    if (status != K_OK)
    {
        fprintf(stderr, "IK failed: %d\n", (int)status);
        return 1;
    }
    printf("joint positions (rad): %.9g %.9g %.9g; residual (m): %.9g\n",
           (double)joint_positions[0], (double)joint_positions[1], (double)joint_positions[2],
           (double)report.residual);
    return 0;
}
