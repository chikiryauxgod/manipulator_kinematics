#include <manipulator/forward.h>
#include <stdio.h>

int main(void)
{
    const k_real pi = (k_real)3.14159265358979323846;
    const k_dh_joint joints[3] = {
        {K_REVOLUTE, 0, pi/2, (k_real)0.1, 0},
        {K_REVOLUTE, (k_real)0.3, 0, 0, 0},
        {K_REVOLUTE, (k_real)0.2, 0, 0, 0}};
    k_real q[3] = {0, 0, pi/2};
    k_model model;
    k_transform world_tool;
    k_transform frames[5], workspace[5];
    k_status status = k_model_init(&model, joints, 3);
    if (status == K_OK) status = k_forward(&model, q, 3, &world_tool);
    if (status == K_OK) status = k_forward_frames(&model, q, 3, frames, 5, workspace, 5);
    if (status != K_OK) { fprintf(stderr, "FK failed: %d\n", (int)status); return 1; }
    printf("position (m): %.9g %.9g %.9g\n", (double)world_tool.m[0][3],
           (double)world_tool.m[1][3], (double)world_tool.m[2][3]);
    return 0;
}
