#include <manipulator/forward.h>
#include <stdio.h>

int main(void)
{
    const mk_real pi = (mk_real)3.14159265358979323846;
    const mk_dh_joint joints[3] = {
        {MK_REVOLUTE, 0, pi/2, (mk_real)0.1, 0},
        {MK_REVOLUTE, (mk_real)0.3, 0, 0, 0},
        {MK_REVOLUTE, (mk_real)0.2, 0, 0, 0}};
    mk_real q[3] = {0, 0, pi/2};
    mk_model model;
    mk_transform world_tool;
    mk_transform frames[5], workspace[5];
    mk_status status = mk_model_init(&model, joints, 3);
    if (status == MK_OK) status = mk_forward(&model, q, 3, &world_tool);
    if (status == MK_OK) status = mk_forward_frames(&model, q, 3, frames, 5, workspace, 5);
    if (status != MK_OK) { fprintf(stderr, "FK failed: %d\n", (int)status); return 1; }
    printf("position (m): %.9g %.9g %.9g\n", (double)world_tool.m[0][3],
           (double)world_tool.m[1][3], (double)world_tool.m[2][3]);
    return 0;
}
