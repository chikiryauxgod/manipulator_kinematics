#include <manipulator/forward.h>
#include <stdio.h>

int main(void)
{
    const manipulator_real pi = (manipulator_real)3.14159265358979323846;
    const manipulator_dh_joint joints[3] = {
        {MANIPULATOR_REVOLUTE, 0, pi/2, (manipulator_real)0.1, 0},
        {MANIPULATOR_REVOLUTE, (manipulator_real)0.3, 0, 0, 0},
        {MANIPULATOR_REVOLUTE, (manipulator_real)0.2, 0, 0, 0}};
    manipulator_real q[3] = {0, 0, pi/2};
    manipulator_model model;
    manipulator_transform world_tool;
    manipulator_transform frames[5], workspace[5];
    manipulator_status status = manipulator_model_init(&model, joints, 3);
    if (status == MANIPULATOR_OK) status = manipulator_forward(&model, q, 3, &world_tool);
    if (status == MANIPULATOR_OK) status = manipulator_forward_frames(&model, q, 3, frames, 5, workspace, 5);
    if (status != MANIPULATOR_OK) { fprintf(stderr, "FK failed: %d\n", (int)status); return 1; }
    printf("position (m): %.9g %.9g %.9g\n", (double)world_tool.m[0][3],
           (double)world_tool.m[1][3], (double)world_tool.m[2][3]);
    return 0;
}
